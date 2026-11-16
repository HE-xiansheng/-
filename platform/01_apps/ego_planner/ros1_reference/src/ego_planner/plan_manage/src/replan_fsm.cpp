#include <traj_utils/diagnostics.hpp>
#include <optimizer/continuous_voxel_check.hpp>
#include <plan_manage/candidate_preview_io.h>

#include <plan_manage/replan_fsm.h>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
#include <cerrno>
#include <climits>
#include <fcntl.h>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

extern char **environ;

namespace ego_planner
{

  EGOReplanFSM::~EGOReplanFSM()
  {
    if (candidate_worker_.joinable()) candidate_worker_.join();
  }

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    node_ = nh;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    nh.param("fsm/mondify_final_goal", mondify_final_goal_, true);
    nh.param("fsm/enable_stuck_detect", enable_stuck_detect_, true);
    nh.param("fsm/odom_vel_is_body", odom_vel_is_body_, false);
    nh.param("fsm/manual_preview", manual_preview_, false);
    nh.param("fsm/manual_replan_period", manual_replan_period_, 0.5);
    nh.param("fsm/manual_arrival_radius", manual_arrival_radius_, 0.25);
    nh.param("fsm/manual_arrival_speed", manual_arrival_speed_, 0.15);
    nh.param("fsm/manual_arrival_hold", manual_arrival_hold_, 1.0);
    have_new_target_ = false;
    if (manual_preview_)
    {
      if (target_type_ != TARGET_TYPE::MANUAL_TARGET ||
          !std::isfinite(manual_replan_period_) || manual_replan_period_ < 0.1 ||
          !std::isfinite(manual_arrival_radius_) || manual_arrival_radius_ <= 0 ||
          !std::isfinite(manual_arrival_speed_) || manual_arrival_speed_ <= 0 ||
          !std::isfinite(manual_arrival_hold_) || manual_arrival_hold_ < 0.1)
        throw std::runtime_error("Invalid manual-preview parameters");
      mondify_final_goal_ = false;
      manual_session_id_ = std::to_string(ros::WallTime::now().toNSec());
      native_execution_trajectory_pub_ = nh.advertise<traj_utils::PolyTraj>("/ego_planner/flight_execution/trajectory", 1);
      manual_trajectory_pub_ = nh.advertise<traj_utils::PolyTraj>("/ego_planner/manual_preview/trajectory", 1, true);
      manual_status_pub_ = nh.advertise<std_msgs::String>("/ego_planner/manual_preview/status", 1, true);
      candidate_trajectory_pub_ = nh.advertise<traj_utils::PolyTraj>("/ego_planner/navigation/candidate_trajectory", 1, true);
      manual_cloud_sub_ = nh.subscribe("grid_map/cloud", 1, &EGOReplanFSM::manualCloudCallback, this);
      flight_execution_sub_ = nh.subscribe("/ego_planner/flight_execution", 1, &EGOReplanFSM::flightExecutionCallback, this);
      navigation_goal_sub_ = nh.subscribe("/ego_planner/navigation/preview_goal", 1, &EGOReplanFSM::navigationGoalCallback, this);
      navigation_state_sub_ = nh.subscribe("/mavros/state", 1, &EGOReplanFSM::navigationStateCallback, this);
    }

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_;
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* initialize  Anomaly Detection Parameters */
    last_local_target_pos_.setZero();
    last_target_change_time_ = ros::Time::now().toSec();
    replan_fail_count_ = 0;
    TARGET_STUCK_TIME = 1.5 * planning_horizen_ / planner_manager_->pp_.max_vel_;
    need_hover_stop_ = false;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    if (manual_preview_) candidatePoll();
    if (manual_preview_ && !flight_execution_)
    {
      manualPreviewTick();
      exec_timer_.start();
      return;
    }
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (!mondify_final_goal_ && planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
        }
        else
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            replan_fail_count_ = 0;
            changeFSMExecState(EXEC_TRAJ, "FSM");
          }
          else
          {
            ROS_WARN("Failed to generate the first trajectory, keep trying");
            replan_fail_count_++;
            changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
          }
        }
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (!mondify_final_goal_ && planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
      {
        ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
        need_hover_stop_ = true;
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
      }
      else
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true;
        }
        else
        {
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);
      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

      const PtsChk_t *chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;
      bool close_to_current_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? chk_ptr->back().back().first - t_cur < emergency_time_ : 0; // In case of empty vector

      if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
      {
        if (!mondify_final_goal_)
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
        }
        else if (mondifyInCollisionFinalGoal())
        {
          ROS_WARN("Successfully modified final_goal in EXEC_TRAJ !!!");
          changeFSMExecState(REPLAN_TRAJ, "mondify_FSM");
        }
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) // case 2: assign the next waypoint
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_], true);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
      {
        if (manual_preview_ && flight_execution_) flight_completed_ = true;
        have_target_ = false;
        have_trigger_ = false;
        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          // prepare for next round
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_], true);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) // case 3: time to perform next replan
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");
      if (enable_stuck_detect_)
      {
        /* Avoid getting stuck wandering around large obstacles */
        static bool baseline_initialized = false;
        if (touch_the_goal)
        {
          static double last_proj_len = 0.0;
          static Eigen::Vector3d baseline_origin = odom_pos_;
          static Eigen::Vector3d last_goal_when_baseline = final_goal_;
          if (!baseline_initialized || (last_goal_when_baseline - final_goal_).norm() > 0.1)
          {
            baseline_origin = odom_pos_;
            last_goal_when_baseline = final_goal_;
            last_proj_len = 0.0;
            baseline_initialized = true;
          }
          Eigen::Vector3d cur_pos = odom_pos_;
          Eigen::Vector3d global2cur = cur_pos - baseline_origin;
          Eigen::Vector3d proj_pos = projectPointToLineSegment(baseline_origin, final_goal_, cur_pos);
          double proj_len = (proj_pos - baseline_origin).norm();
          if (proj_len - last_proj_len < TARGET_STUCK_THRESH)
          {
            if (ros::Time::now().toSec() - last_target_change_time_ > TARGET_STUCK_TIME)
            {
              ROS_WARN("Drone stuck! Obstacle too large and near final goal. Emergency stop.");
              need_hover_stop_ = true;
              flag_escape_emergency_ = true;
              changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
            }
          }
          else
          {
            last_proj_len = proj_len;
            last_target_change_time_ = ros::Time::now().toSec();
          }

          if (global2cur.norm() > planning_horizen_ * M_SQRT2)
          {
            ROS_WARN("Drone stuck! The drone flew too far out of its way . Emergency stop.");
            need_hover_stop_ = true;
            flag_escape_emergency_ = true;
            changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
          }
        }
        else
        {
          baseline_initialized = false;
          if ((local_target_pt_ - last_local_target_pos_).norm() < TARGET_STUCK_THRESH)
          {
            if (ros::Time::now().toSec() - last_target_change_time_ > TARGET_STUCK_TIME)
            {
              ROS_WARN("Drone stuck! Obstacle too large. Emergency stop.");
              need_hover_stop_ = true;
              flag_escape_emergency_ = true;
              changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
            }
          }
          else
          {
            last_local_target_pos_ = local_target_pt_;
            last_target_change_time_ = ros::Time::now().toSec();
          }
        }
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          last_target_change_time_ = ros::Time::now().toSec();
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point !!!");
          need_hover_stop_ = false;
          have_target_ = false;
          have_trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT"); 
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }
    finishProcess();
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    if (manual_preview_ && flight_execution_)
    {
      const char *phase = exec_state_ == EMERGENCY_STOP ? "EMERGENCY_STOP" :
          exec_state_ == WAIT_TARGET ? "WAIT_TARGET" :
          manual_published_trajectory_ > 0 ? "ACTIVE" : "PLANNING";
      publishManualStatus(phase);
    }
    exec_timer_.start();
  }
  void EGOReplanFSM::finishProcess()
  {
    if (replan_fail_count_ > MAX_REPLAN_FAIL_COUNT)
    {
      ROS_WARN("replan fail too much. Emergency stop.");
      replan_fail_count_ = 0; 
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == EMERGENCY_STOP) { flight_completed_ = false; }

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    if (manual_preview_ && !flight_execution_) return;
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    const double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || exec_state_ ==  EMERGENCY_STOP || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= (int)pts_chk.size())
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            if (dist < allowed_dist)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    publishFlightTrajectory(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);
    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (mondify_final_goal_ && mondifyInCollisionFinalGoal()) 
    {
      ROS_WARN("Successfully modified final_goal in callReboundReplan !!!");
    }
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      publishFlightTrajectory(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = std::max(0.0, std::min(info->duration, ros::Time::now().toSec() - info->start_time));

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

    bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
    if (success)
    {
      final_goal_ = next_wp;
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }
      have_target_ = true;
      have_new_target_ = true;
      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET && flag_2replan && exec_state_ != EMERGENCY_STOP)
      {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(0.5); 
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
          if (ros::Time::now() - start_time > timeout)
          {
            ROS_WARN("Timeout waiting for state to change to EXEC_TRAJ.");
            return false; 
          }
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else if(exec_state_ == EMERGENCY_STOP)
      {
        return true;
      }
      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
       visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
    return success;
  }

  bool EGOReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          for (int i = 6; i > 0; i--)
          {
            if (t - i * t_step > 0)
            {
              Eigen::Vector3d pt_tmp = planner_manager_->traj_.global_traj.traj.getPos(t - i * t_step);
              if (!planner_manager_->grid_map_->getInflateOccupancy(pt_tmp))
              {
                pt = pt_tmp;
                break;
              }
            }
          }
          if (planNextWaypoint(pt, false)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= t_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (manual_preview_)
    {
      const auto &p = msg->pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      { navigation_message_ = "INVALID_GOAL"; publishManualStatus(manual_phase_); return; }
      const Eigen::Vector3d next(p.x,p.y,p.z);
      if (flight_execution_)
      {
        // 原生全局参考更新会重置local容器；保留当前PVA以衔接原生局部重规划。
        const auto saved = planner_manager_->traj_;
        const auto saved_goal = final_goal_;
        const bool saved_target = have_target_, saved_new = have_new_target_;
        if (!planNextWaypoint(next,false))
        {
          planner_manager_->traj_ = saved; final_goal_ = saved_goal;
          have_target_ = saved_target; have_new_target_ = saved_new;
          navigation_message_ = "GLOBAL_PLAN_FAILED"; publishManualStatus(manual_phase_); return;
        }
        planner_manager_->traj_.local_traj = saved.local_traj;
        have_trigger_ = true;
        changeFSMExecState(saved.local_traj.duration > 0 ? REPLAN_TRAJ : GEN_NEW_TRAJ,"MAP_GOAL_UPDATE");
        last_target_change_time_ = ros::Time::now().toSec();
      }
      navigation_required_ = false;
      navigation_confirmed_ = true;
      navigation_message_ = "GOAL_ACCEPTED";
      flight_completed_ = false;
      candidate_phase_ = "NONE"; candidate_reason_.clear();
      manual_has_goal_ = true; manual_arrived_ = false;
      manual_inside_since_ = ros::WallTime(); manual_last_plan_ = ros::WallTime();
      ++manual_goal_id_;
      manual_requested_goal_ = next; final_goal_ = next;
      if (!flight_execution_) manual_published_trajectory_ = 0;
      publishManualStatus("GOAL_ACCEPTED");
      return;
    }
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (planner_manager_->grid_map_->getInflateOccupancy(end_wp) == -1)
    {
      ROS_WARN("The goal is outside the safe fence, ignore this goal!");
      return;
    }
    ROS_INFO("Received goal: %f, %f, %f", end_wp(0), end_wp(1), end_wp(2));
    if (planNextWaypoint(end_wp, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
      have_trigger_ = true;
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_], true);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    if (manual_preview_ && !flight_execution_)
    {
      manual_has_goal_ = false;
      manual_arrived_ = false;
      manual_inside_since_ = ros::WallTime();
      publishManualStatus("STOPPED");
      return;
    }
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (manual_preview_)
    {
      have_odom_ = false;
      const auto &p = msg->pose.pose.position;
      const auto &v = msg->twist.twist.linear;
      const auto &q = msg->pose.pose.orientation;
      Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
          !orientation.coeffs().allFinite() || orientation.norm() <= 1e-9)
      { manual_inside_since_ = ros::WallTime(); return; }
      odom_pos_ = Eigen::Vector3d(p.x, p.y, p.z);
      odom_orient_ = orientation.normalized();
      odom_vel_ = Eigen::Vector3d(v.x, v.y, v.z);
      if (odom_vel_is_body_) odom_vel_ = odom_orient_ * odom_vel_;
      manual_odom_stamp_ = msg->header.stamp;
      manual_odom_receipt_ = ros::WallTime::now();
      have_odom_ = true;
      return;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    if (odom_vel_is_body_)
    {
      Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z);
      odom_vel_ = odom_orient_ * v_body;
    }
    else
    {
      odom_vel_(0) = msg->twist.twist.linear.x;
      odom_vel_(1) = msg->twist.twist.linear.y;
      odom_vel_(2) = msg->twist.twist.linear.z;
    }

    have_odom_ = true;
  }


  void EGOReplanFSM::manualCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    manual_cloud_stamp_ = msg->header.stamp;
    manual_cloud_receipt_ = ros::WallTime::now();
  }

  void EGOReplanFSM::navigationStateCallback(const mavros_msgs::StateConstPtr &msg)
  {
    navigation_fc_mode_ = msg->mode;
    navigation_fc_connected_ = msg->connected;
    navigation_fc_armed_ = msg->armed;
    navigation_fc_receipt_ = ros::WallTime::now();
  }

  std::string EGOReplanFSM::candidateToken() const
  {
    return manual_session_id_ + ":candidate:" + std::to_string(candidate_id_);
  }

  bool EGOReplanFSM::candidateAvailable() const
  {
    return false; // 自动目标更新不再使用候选授权工作流。
  }

  bool EGOReplanFSM::candidatePathClear(const poly_traj::Trajectory &path, std::string &reason) const
  {
    reason.clear(); return true; // 删除后加复查；轨迹可行性由原生优化与碰撞检查处理。
  }

  bool EGOReplanFSM::candidateCurrent(std::string &reason) const
  {
    reason.clear(); return true;
  }

  std::vector<std::tuple<int,int,double>> EGOReplanFSM::candidateSwarmSignature() const
  {
    std::vector<std::tuple<int,int,double>> signature;
    const auto &swarm=planner_manager_->traj_.swarm_traj;
    for (size_t i=0; i<swarm.size(); ++i)
      if (swarm[i].drone_id==static_cast<int>(i) && swarm[i].drone_id!=planner_manager_->pp_.drone_id)
        signature.emplace_back(swarm[i].drone_id,swarm[i].traj_id,swarm[i].start_time);
    return signature;
  }

  void EGOReplanFSM::candidateStart(const geometry_msgs::PoseStampedPtr &msg)
  {
    waypointCallback(msg);
  }

  void EGOReplanFSM::candidatePoll()
  {
    // 已停用候选子进程。
  }

  void EGOReplanFSM::candidateConfirm(const std::string &token)
  {
    // 已删除人工候选确认。
  }

  void EGOReplanFSM::navigationGoalCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    // 地图选点直接交给原生目标更新，不经过候选、人工令牌、到达或位移门槛。
    navigation_ingress_ = false;
    waypointCallback(msg);
  }

  std::string EGOReplanFSM::navigationGoalReason()
  {
    if (!manual_has_goal_) return "NO_GOAL";
    if (!have_odom_ || manual_cloud_receipt_.isZero()) return "MAP_NOT_READY";
    const auto map = planner_manager_->grid_map_;
    if (map->getInflateOccupancy(manual_requested_goal_) < 0) return "VIRTUAL_WALL";
    if (map->getOccupancy(manual_requested_goal_) > 0) return "OCCUPIED";
    if (map->getInflateOccupancy(manual_requested_goal_) > 0) return "INFLATED";
    // Preserve the original rolling-map semantics, including distant targets.
    if (!map->isInCurrentMap(manual_requested_goal_)) return "OUTSIDE_LOCAL_MAP";
    return "FREE_IN_LOCAL_MAP";
  }

  void EGOReplanFSM::navigationConfirmCallback(const std_msgs::StringConstPtr &)
  {
    // 兼容旧符号；自动目标入口不再接收人工确认。
  }

  void EGOReplanFSM::flightExecutionCallback(const std_msgs::BoolConstPtr &msg)
  {
    if (flight_execution_ == msg->data) return;
    flight_execution_ = msg->data;
    flight_completed_ = false;
    if (candidate_phase_ != "NONE") { candidate_phase_ = "INVALID"; candidate_reason_ = "FLIGHT_MODE_CHANGED"; }
    navigation_arrival_since_ = ros::WallTime();
    manual_last_status_ = ros::WallTime();
    manual_last_plan_ = ros::WallTime();
    manual_arrived_ = false;
    manual_published_trajectory_ = 0;
    have_target_ = false;
    have_trigger_ = false;
    planner_manager_->traj_.local_traj.traj_id = 0;
    if (flight_execution_)
    {
      flag_escape_emergency_ = true;
      need_hover_stop_ = false;
      mandatory_stop_ = false;
      replan_fail_count_ = 0;
      ros::NodeHandle private_nh("~");
      private_nh.param("fsm/fail_safe", enable_fail_safe_, true);
    }
    changeFSMExecState(WAIT_TARGET, flight_execution_ ? "PILOT_OFFBOARD" : "PILOT_MANUAL");
    if (flight_execution_ && manual_has_goal_ && have_odom_)
    {
      if (planNextWaypoint(manual_requested_goal_, false))
      {
        have_trigger_ = true;
        last_target_change_time_ = ros::Time::now().toSec();
        changeFSMExecState(SEQUENTIAL_START, "PILOT_OFFBOARD");
      }
    }
    publishManualStatus(flight_execution_ ? "PLANNING" : "GOAL_ACCEPTED");
  }

  void EGOReplanFSM::publishFlightTrajectory(const traj_utils::PolyTraj &poly)
  {
    if (!manual_preview_ || !flight_execution_) return;
    traj_utils::PolyTraj output = poly;
    output.traj_id = ++manual_trajectory_sequence_;
    manual_published_trajectory_ = output.traj_id;
    manual_trajectory_pub_.publish(output);
    native_execution_trajectory_pub_.publish(output);
    flight_diag::event("execution_trajectory", ",\"trajectory_id\":"+std::to_string(output.traj_id)+
      ",\"start\":"+flight_diag::number(output.start_time.toSec()));
  }

  void EGOReplanFSM::publishManualStatus(const std::string &phase)
  {
    const auto wall = ros::WallTime::now();
    if (phase == manual_phase_ && !manual_last_status_.isZero() &&
        (wall - manual_last_status_).toSec() < 0.2) return;
    manual_last_status_ = wall;
    if (phase != manual_phase_) {
      const Eigen::Vector3d diagnostic_position = have_odom_ ? odom_pos_ : Eigen::Vector3d::Zero();
      flight_diag::event("fsm_phase", ",\"phase\":\""+phase+"\",\"goal_id\":"+std::to_string(manual_goal_id_)+
        ",\"position\":"+flight_diag::matrix(diagnostic_position)+",\"goal\":"+flight_diag::matrix(manual_requested_goal_));
      if (phase == "START_IN_COLLISION" || phase == "GOAL_IN_COLLISION")
        planner_manager_->grid_map_->saveDiagnosticSnapshot(flight_diag::plan());
    }
    manual_phase_ = phase;
    // Pose and requested goal are finite by callback validation before serialization.
    const Eigen::Vector3d p = have_odom_ ? odom_pos_ : Eigen::Vector3d::Zero();
    const double distance = have_odom_ && manual_has_goal_ ? (p - manual_requested_goal_).norm() : -1.0;
    bool measured_arrived = manual_arrived_;
    if (flight_execution_)
    {
      const double age = (ros::Time::now()-manual_odom_stamp_).toSec();
      if (manual_has_goal_ && distance >= 0.0 && distance <= manual_arrival_radius_ &&
          odom_vel_.norm() <= manual_arrival_speed_ && age >= -0.05 && age <= 0.5 &&
          !manual_odom_receipt_.isZero() && (wall-manual_odom_receipt_).toSec() <= 0.5)
      {
        if (navigation_arrival_since_.isZero()) navigation_arrival_since_ = wall;
        measured_arrived = (wall-navigation_arrival_since_).toSec() >= manual_arrival_hold_;
      }
      else { navigation_arrival_since_ = ros::WallTime(); measured_arrived = false; }
    }
    std::ostringstream out;
    out << std::setprecision(15) << "{\"mode\":\"manual_preview\",\"phase\":\"" << phase
        << "\",\"session_id\":\"" << manual_session_id_ << "\",\"stamp\":" << ros::Time::now().toSec()
        << ",\"goal_id\":" << manual_goal_id_
        << ",\"has_goal\":" << (manual_has_goal_ ? "true" : "false")
        << ",\"goal\":[" << manual_requested_goal_.x() << "," << manual_requested_goal_.y() << "," << manual_requested_goal_.z()
        << "],\"position\":[" << p.x() << "," << p.y() << "," << p.z()
        << "],\"distance_m\":" << distance
        << ",\"trajectory_id\":" << manual_published_trajectory_
        << ",\"planning_mode\":\"" << (flight_execution_ ? "upstream_fsm" : "measured_preview") << "\""
        << ",\"confirmation_required\":" << (navigation_required_ ? "true" : "false")
        << ",\"goal_confirmed\":" << (navigation_confirmed_ ? "true" : "false")
        << ",\"measured_arrived\":" << (measured_arrived ? "true" : "false")
        << ",\"confirm_token\":\"" << manual_session_id_ << ":" << manual_goal_id_ << "\""
        << ",\"goal_frame\":\"world\",\"height_reference\":\"fastlio_session_origin\""
        << ",\"goal_validation\":\"" << navigationGoalReason() << "\""
        << ",\"navigation_message\":\"" << navigation_message_ << "\""
        << ",\"candidate\":{\"available\":" << (candidateAvailable() ? "true" : "false")
        << ",\"id\":" << candidate_id_ << ",\"phase\":\"" << candidate_phase_
        << "\",\"reason\":\"" << candidate_reason_ << "\",\"confirm_token\":\"" << candidateToken()
        << "\",\"goal\":[" << candidate_goal_.x() << "," << candidate_goal_.y() << "," << candidate_goal_.z()
        << "],\"trajectory_id\":" << (candidate_phase_ == "READY" ? candidate_id_ : 0) << "}"
        << ",\"command_output\":false}";
    std_msgs::String msg;
    msg.data = out.str();
    manual_status_pub_.publish(msg);
  }

  void EGOReplanFSM::manualPreviewTick()
  {
    const auto wall = ros::WallTime::now();
    const auto now = ros::Time::now();
    const double odom_age = (now - manual_odom_stamp_).toSec();
    const double cloud_age = (now - manual_cloud_stamp_).toSec();
    if (!have_odom_ || manual_cloud_receipt_.isZero())
    { publishManualStatus("WAIT_INPUT"); return; }
    if (!manual_has_goal_) { publishManualStatus(manual_phase_ == "STOPPED" || manual_phase_ == "INVALID_GOAL" ? manual_phase_ : "WAIT_GOAL"); return; }
    if (manual_arrived_) { publishManualStatus("ARRIVED"); return; }
    auto map = planner_manager_->grid_map_;
    if (map->getInflateOccupancy(manual_requested_goal_) != 0)
    { manual_inside_since_ = ros::WallTime(); publishManualStatus("GOAL_IN_COLLISION"); return; }
    if (map->getInflateOccupancy(odom_pos_) != 0)
    { manual_inside_since_ = ros::WallTime(); publishManualStatus("START_IN_COLLISION"); return; }
    if ((odom_pos_ - manual_requested_goal_).norm() <= manual_arrival_radius_)
    {
      // Do not optimize a near-zero-length trajectory while being carried through the goal.
      if (odom_vel_.norm() > manual_arrival_speed_)
      { manual_inside_since_ = ros::WallTime(); publishManualStatus("WAIT_STOP"); return; }
      if (manual_inside_since_.isZero()) manual_inside_since_ = wall;
      if ((wall - manual_inside_since_).toSec() >= manual_arrival_hold_)
      { manual_arrived_ = true; publishManualStatus("ARRIVED"); }
      else publishManualStatus("SETTLING");
      return;
    }
    manual_inside_since_ = ros::WallTime();
    if (!manual_last_plan_.isZero() && (wall - manual_last_plan_).toSec() < manual_replan_period_)
    { publishManualStatus(manual_phase_); return; }
    manual_last_plan_ = wall;
    flight_diag::event("fsm_plan_input", ",\"next_plan_id\":"+std::to_string(flight_diag::plan()+1)+
      ",\"goal_id\":"+std::to_string(manual_goal_id_)+",\"goal\":"+flight_diag::matrix(manual_requested_goal_)+
      ",\"odom_stamp\":"+flight_diag::number(manual_odom_stamp_.toSec())+
      ",\"cloud_stamp\":"+flight_diag::number(manual_cloud_stamp_.toSec())+
      ",\"odom_age\":"+flight_diag::number(odom_age)+",\"cloud_age\":"+flight_diag::number(cloud_age));
    // Rebuild the global guide at the measured pose, resetting its progress cursor.
    const std::vector<Eigen::Vector3d> waypoints{manual_requested_goal_};
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    flight_diag::event("global_guide_result", ",\"success\":"+std::string(success?"true":"false")+
      ",\"next_plan_id\":"+std::to_string(flight_diag::plan()+1));
    if (!success) map->saveDiagnosticSnapshot(flight_diag::plan());
    if (success)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      final_goal_ = manual_requested_goal_;
      planner_manager_->getLocalTarget(planning_horizen_, start_pt_, final_goal_, local_target_pt_, local_target_vel_, touch_goal_);
      success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, true, false, touch_goal_);
    }
    if (!success) { publishManualStatus("PLANNING_FAILED"); return; }
    traj_utils::PolyTraj poly;
    traj_utils::MINCOTraj unused;
    polyTraj2ROSMsg(poly, unused);
    // Global-guide regeneration resets upstream's trajectory ID; keep preview IDs unique.
    poly.traj_id = ++manual_trajectory_sequence_;
    manual_published_trajectory_ = poly.traj_id;
    manual_trajectory_pub_.publish(poly);
    publishManualStatus("ACTIVE");
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if ( msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true;
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // Accept a far traj if no previous agent received
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (planner_manager_->checkCollision(recv_id))
      {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
            {
              break;
            }

            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; // Means this trajectory is invalid
    }
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
  Eigen::Vector3d EGOReplanFSM::projectPointToLineSegment(const Eigen::Vector3d& a,
                                                          const Eigen::Vector3d& b,
                                                          const Eigen::Vector3d& p)
  {
      double t = 0.0;
      Eigen::Vector3d ab = b - a;
      double ab2 = ab.squaredNorm();   
      double ab_norm = ab.norm();
      if (ab2 < 1e-8)                  
      {
        t = 0.0;
        return a;
      }
      t = (p - a).dot(ab) / ab2;       
      if (t < 0.0)                     
      {
        t = 0.0;
        return a;
      }
      else if (t > 1.0)                
      {
        t = 1.0;
        return b;
      }
      return a + t * ab;
  }
} // namespace ego_planner
