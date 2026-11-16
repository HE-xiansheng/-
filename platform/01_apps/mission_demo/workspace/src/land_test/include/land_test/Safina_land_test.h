#ifndef SAFINA_LAND_TEST_H
#define SAFINA_LAND_TEST_H
#include "land_test/landing_task.h"
#include "land_test/mode_start_gate.h"
#include "gcu/gcu_control.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <future>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <memory>

class Safina_land {
public:
    explicit Safina_land(rclcpp::Node::SharedPtr node);
    ~Safina_land();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_,marker_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_,header_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_,target_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr consumer_client_;
    std::shared_future<rcl_interfaces::srv::GetParameters::Response::SharedPtr> consumer_probe_;
    int64_t consumer_request_id_=0;
    bool consumer_ready_=false;
    double consumer_probe_at_=-1,consumer_check_at_=-1;
    std::string consumer_node_="/mavros/setpoint_raw",consumer_transport_="/uas1/mavlink_sink";
    void updateConsumer(double now);
    land_task::Config config_;
    std::unique_ptr<land_task::Task> task_;
    land_task::ModeStartGate mode_gate_;
    std::string start_trigger_="px4_mode";
    bool standby_clear_=false;
    double standby_check_at_=-1,mode_tick_at_=-1;
    std::mutex mutex_;
    std::condition_variable jobs_;
    std::thread worker_;
    bool stopping_=false,pending_=false,worker_busy_=false,gimbal_ready_=false;
    std::string job_,job_mode_;
    unsigned long generation_=0,job_generation_=0;
    mavros_msgs::msg::State state_;
    mavros_msgs::msg::ExtendedState extended_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::TwistStamped velocity_;
    struct TimedPose { geometry_msgs::msg::PoseStamped message; double received; };
    std::deque<TimedPose> poses_;
    double state_at_=-1,pose_at_=-1,velocity_at_=-1,extended_at_=-1,marker_rx_at_=-1;
    double marker_valid_at_=-1,target_x_=0,target_y_=0,target_body_x_=0,target_body_y_=0,marker_yaw_=0,marker_distance_=-1;
    std::string marker_reason_="no_marker_messages",diagnostic_header_;
    double marker_source_age_=-1,marker_pair_dt_=-1,marker_jump_=0,target_z_=0;
    unsigned long marker_received_=0,marker_accepted_=0;
    bool pose_valid_=false,velocity_valid_=false,marker_valid_=false,target_filter_ready_=false;
    rclcpp::Time last_marker_stamp_{0,0,RCL_ROS_TIME};
    double yaw_=0,yaw_rate_=0,last_pose_yaw_=0,last_pose_time_=-1;
    double state_timeout_=1.5,pose_timeout_=0.3,velocity_timeout_=0.3,extended_timeout_=1.5;
    double source_stamp_timeout_=0.4,pose_image_tolerance_=0.15,target_alpha_=0.4,image_latency_=0.0;
    double max_marker_range_=10.0,max_target_jump_=0.5;
    double camera_offset_x_=0,camera_offset_y_=0,camera_offset_z_=0;
    std::string local_frame_="map",marker_frame_="gimbal_camera_optical_frame";
    bool command_gimbal_down_=true,gimbal_down_on_startup_=true;
    std::string gimbal_status_="not_requested";
    std::string gimbal_ip_="192.168.144.108",diagnostic_path_;
    int gimbal_port_=2332;
    double diagnostic_hz_=5,last_diagnostic_=-1;
    std::ofstream diagnostic_file_;
    land_task::Phase last_phase_=land_task::Phase::IDLE;
    void loadParameters();
    void openDiagnostics();
    land_task::Input input(double now) const;
    bool enable(std_srvs::srv::SetBool::Request&,std_srvs::srv::SetBool::Response&);
    void tick();
    bool competingPublisher(std::string& reason) const;
    bool sourceFresh(const builtin_interfaces::msg::Time&, double timeout) const;
    void stateCallback(const mavros_msgs::msg::State::ConstSharedPtr&);
    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&);
    void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr&);
    void markerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&);
    void extendedCallback(const mavros_msgs::msg::ExtendedState::ConstSharedPtr&);
    bool startTask(double now,std::string& reason);
    void queueJob(const std::string&,double now);
    void workerLoop();
    void diagnostics(const land_task::Input&,const land_task::Command&);
};
#endif
