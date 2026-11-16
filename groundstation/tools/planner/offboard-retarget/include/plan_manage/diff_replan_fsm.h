#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <thread>
#include <mutex>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace diff_planner
{

  class DiffReplanFSM
  {
  public:
    DiffReplanFSM() {}
    ~DiffReplanFSM();

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };
    /* Anomaly Detection Parameters */
    Eigen::Vector3d last_local_target_pos_;
    double last_target_change_time_;
    int replan_fail_count_;
    static constexpr double TARGET_STUCK_THRESH = 0.3;  // Threshold for target movement below which it's considered "stuck"
    double TARGET_STUCK_TIME;                           // Default time threshold (seconds) for being considered stuck before reinitialization
    static constexpr int MAX_REPLAN_FAIL_COUNT = 10;    // Threshold for maximum optimization failure count
    /* planning utils */
    DiffPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;
    bool need_hover_stop_;
    bool mondify_final_goal_;
    bool enable_stuck_detect_; // Whether to enable stuck detection
    bool odom_vel_is_body_{false}; // If true, transform body-frame linear velocity to world frame: v_world = R_wb * v_body
    // Standalone handheld preview: never advances or publishes executable references.
    bool manual_preview_{false}, manual_has_goal_{false}, manual_arrived_{false};
    double manual_replan_period_{0.5}, manual_arrival_radius_{0.25};
    double manual_arrival_speed_{0.15}, manual_arrival_hold_{1.0};
    ros::Time manual_odom_stamp_, manual_cloud_stamp_;
    ros::WallTime manual_odom_receipt_, manual_cloud_receipt_, manual_last_plan_;
    ros::WallTime manual_inside_since_;
    ros::WallTime manual_last_status_;
    std::string manual_session_id_;
    Eigen::Vector3d manual_requested_goal_{Eigen::Vector3d::Zero()};
    unsigned int manual_goal_id_{0};
    int manual_trajectory_sequence_{0}, manual_published_trajectory_{0};
    std::string manual_phase_{"WAIT_ODOM"};
    ros::Publisher manual_trajectory_pub_, manual_status_pub_;
    ros::Subscriber manual_cloud_sub_;
    bool flight_execution_{false};
    ros::Subscriber flight_execution_sub_;
    // Navigation candidates reuse measured preview; legacy /goal keeps its contract.
    bool navigation_ingress_{false}, navigation_required_{false}, navigation_confirmed_{false};
    std::string navigation_message_, navigation_fc_mode_;
    bool navigation_fc_connected_{false}, navigation_fc_armed_{false};
    ros::WallTime navigation_fc_receipt_;
    ros::WallTime navigation_arrival_since_;
    ros::Subscriber navigation_goal_sub_, navigation_confirm_sub_, navigation_state_sub_;
    void navigationGoalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void navigationConfirmCallback(const std_msgs::StringConstPtr &msg);
    void navigationStateCallback(const mavros_msgs::StateConstPtr &msg);
    std::string navigationGoalReason();
    void flightExecutionCallback(const std_msgs::BoolConstPtr &msg);
    void publishFlightTrajectory(const traj_utils::PolyTraj &poly);
    void manualPreviewTick();
    void publishManualStatus(const std::string &phase);
    void manualCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    // OFFBOARD候选与正在执行的目标、轨迹和确认状态完全分离。
    struct CandidateResult {
      bool ready{false}, success{false};
      std::string reason;
      traj_utils::PolyTraj poly;
      poly_traj::Trajectory path;
    };
    bool flight_completed_{false};
    unsigned int candidate_id_{0};
    unsigned int candidate_active_goal_{0};
    Eigen::Vector3d candidate_goal_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d candidate_start_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d candidate_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d candidate_acceleration_{Eigen::Vector3d::Zero()};
    std::vector<std::tuple<int,int,double>> candidate_swarm_signature_;
    std::string candidate_phase_{"NONE"}, candidate_reason_;
    ros::WallTime candidate_last_validation_, candidate_last_publish_;
    poly_traj::Trajectory candidate_path_;
    traj_utils::PolyTraj candidate_poly_;
    ros::Publisher candidate_trajectory_pub_;
    std::thread candidate_worker_;
    std::mutex candidate_mutex_;
    CandidateResult candidate_result_;
    bool candidateAvailable() const;
    bool candidateCurrent(std::string &reason) const;
    bool candidatePathClear(const poly_traj::Trajectory &path, std::string &reason) const;
    std::vector<std::tuple<int,int,double>> candidateSwarmSignature() const;
    std::string candidateToken() const;
    void candidateStart(const geometry_msgs::PoseStampedPtr &msg);
    void candidatePoll();
    void candidateConfirm(const std::string &token);

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    Eigen::Quaterniond odom_orient_{1.0, 0.0, 0.0, 0.0};
    std::vector<Eigen::Vector3d> wps_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, DiffReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan);
    bool mondifyInCollisionFinalGoal();
    void finishProcess();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
    Eigen::Vector3d projectPointToLineSegment(const Eigen::Vector3d& a,
                                              const Eigen::Vector3d& b,
                                              const Eigen::Vector3d& p);
  };

} // namespace diff_planner

#endif
