#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/msg/poly_traj.hpp>
#include <traj_utils/msg/minco_traj.hpp>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM();

    void init(const rclcpp::Node::SharedPtr &nh);

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
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;

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
    rclcpp::Time manual_odom_stamp_{0, 0, RCL_ROS_TIME}, manual_cloud_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time manual_odom_receipt_{0, 0, RCL_STEADY_TIME}, manual_cloud_receipt_{0, 0, RCL_STEADY_TIME}, manual_last_plan_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time manual_inside_since_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time manual_last_status_{0, 0, RCL_STEADY_TIME};
    std::string manual_session_id_;
    Eigen::Vector3d manual_requested_goal_{Eigen::Vector3d::Zero()};
    unsigned int manual_goal_id_{0};
    int manual_trajectory_sequence_{0}, manual_published_trajectory_{0};
    std::string manual_phase_{"WAIT_ODOM"};
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr native_execution_trajectory_pub_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr manual_trajectory_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr manual_cloud_sub_;
    bool flight_execution_{false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flight_execution_sub_;
    // Navigation candidates reuse measured preview; legacy /goal keeps its contract.
    bool navigation_ingress_{false}, navigation_required_{false}, navigation_confirmed_{false};
    std::string navigation_message_, navigation_fc_mode_;
    bool navigation_fc_connected_{false}, navigation_fc_armed_{false};
    rclcpp::Time navigation_fc_receipt_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time navigation_arrival_since_{0, 0, RCL_STEADY_TIME};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_sub_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_confirm_sub_;
rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr navigation_state_sub_;
    void navigationGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void navigationConfirmCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
    void navigationStateCallback(const mavros_msgs::msg::State::ConstSharedPtr &msg);
    std::string navigationGoalReason();
    void flightExecutionCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void publishFlightTrajectory(const traj_utils::msg::PolyTraj &poly);
    void manualPreviewTick();
    void publishManualStatus(const std::string &phase);
    void manualCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    // OFFBOARD候选与正在执行的目标、轨迹和确认状态完全分离。
    struct CandidateResult {
      bool ready{false}, success{false};
      std::string reason;
      traj_utils::msg::PolyTraj poly;
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
    rclcpp::Time candidate_last_validation_{0, 0, RCL_STEADY_TIME}, candidate_last_publish_{0, 0, RCL_STEADY_TIME};
    poly_traj::Trajectory candidate_path_;
    traj_utils::msg::PolyTraj candidate_poly_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr candidate_trajectory_pub_;
    std::thread candidate_worker_;
    std::mutex candidate_mutex_;
    CandidateResult candidate_result_;
    bool candidateAvailable() const;
    bool candidateCurrent(std::string &reason) const;
    bool candidatePathClear(const poly_traj::Trajectory &path, std::string &reason) const;
    std::vector<std::tuple<int,int,double>> candidateSwarmSignature() const;
    std::string candidateToken() const;
    void candidateStart(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void candidatePoll();
    void candidateConfirm(const std::string &token);

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()}, odom_vel_{Eigen::Vector3d::Zero()}, odom_acc_{Eigen::Vector3d::Zero()};     // odometry state
    Eigen::Quaterniond odom_orient_{1.0, 0.0, 0.0, 0.0};
    std::vector<Eigen::Vector3d> wps_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    std::string frame_id_;
    bool preset_pending_{false};
    rclcpp::Time preset_ready_{0, 0, RCL_STEADY_TIME};
    bool inputFresh() const;

    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
rclcpp::Subscription<traj_utils::msg::MINCOTraj>::SharedPtr broadcast_ploytraj_sub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mandatory_stop_sub_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_pub_;
rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr broadcast_ploytraj_pub_;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_height_pub_;

    /* state machine functions */
    void execFSMCallback();
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback();
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan);
    bool mondifyInCollisionFinalGoal();
    void finishProcess();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::msg::Empty &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::msg::MINCOTraj::ConstSharedPtr &msg);
    void polyTraj2ROSMsg(traj_utils::msg::PolyTraj &poly_msg, traj_utils::msg::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
    Eigen::Vector3d projectPointToLineSegment(const Eigen::Vector3d& a,
                                              const Eigen::Vector3d& b,
                                              const Eigen::Vector3d& p);
  };

} // namespace ego_planner

#endif
