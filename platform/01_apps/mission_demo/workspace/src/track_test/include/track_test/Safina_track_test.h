#ifndef SAFINA_TRACK_TEST_H
#define SAFINA_TRACK_TEST_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <future>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <yolo_ultra_test/msg/tracked_target.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cmath>
#include <string>



class Safina_Track
{
public:
    Safina_Track(rclcpp::Node::SharedPtr node_track);
    ~Safina_Track();

private:
    enum class Swa
    {
        Up = 0,
        Mid = 1,
        Down = 2
    };

    enum class State
    {
        WaitOffboard,  // PX4 is not in OFFBOARD: never act on SWA
        Hold,          // OFFBOARD: hold the position captured on entry
        Track,         // OFFBOARD + SWA mid: follow the primary target
        Land           // OFFBOARD: descend in place, then AUTO.LAND
    };

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_rc_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_track_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr sub_extended_state_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setraw_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr consumer_client_;
    std::shared_future<rcl_interfaces::srv::GetParameters::Response::SharedPtr> consumer_probe_;
    int64_t consumer_request_id_=0;
    bool consumer_ready_=false;
    double consumer_probe_at_=-1,consumer_check_at_=-1;
    std::string consumer_node_="/mavros/setpoint_raw",consumer_transport_="/uas1/mavlink_sink";
    void updateConsumer(double now);

    // Must be declared after node_ (constructed with node_).
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
    rclcpp::Subscription<yolo_ultra_test::msg::TrackedTarget>::SharedPtr sub_stamped_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    mavros_msgs::msg::State fc_state_;
    rclcpp::Time last_state_time_{0,0,RCL_STEADY_TIME};
    rclcpp::Time state_stamp_{0,0,RCL_ROS_TIME}, pose_stamp_{0,0,RCL_ROS_TIME},
        rc_stamp_{0,0,RCL_ROS_TIME}, velocity_stamp_{0,0,RCL_ROS_TIME},
        landed_stamp_{0,0,RCL_ROS_TIME}, target_stamp_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_velocity_time_{0,0,RCL_STEADY_TIME}, last_tick_{0,0,RCL_STEADY_TIME};
    bool legacy_target_{false}, mode_pending_{false};
    int64_t mode_request_id_{0};
    rclcpp::Time mode_request_at_{0,0,RCL_STEADY_TIME};
    double state_timeout_s_{2.0};
    std::string pose_frame_, velocity_frame_;
    static rclcpp::Time steadyNow() { return rclcpp::Clock(RCL_STEADY_TIME).now(); }
    bool sourceFresh(const rclcpp::Time &stamp, double limit) const;
    bool stateFresh() const;
    void stateCb(mavros_msgs::msg::State::ConstSharedPtr msg);
    void stampedCb(yolo_ultra_test::msg::TrackedTarget::ConstSharedPtr msg);
    void requestAutoLand();

    // ---- parameters (config/track.yaml) ----
    double control_rate_hz_ = 20.0;
    bool require_offboard_ = true;
    double target_timeout_s_ = 0.5;
    double target_loss_grace_s_ = 0.3;
    double pose_timeout_s_ = 0.5;
    double rc_timeout_s_ = 1.0;

    double tracking_vlxy_ = 0.8;
    double max_forward_speed_ = 0.3;
    double max_backward_speed_ = 0.15;
    double tracking_ryaw_ = 1.5;
    double max_yaw_rate_ = 1.2;
    // Refuse to start tracking below this height (prevents ground dragging).
    double min_tracking_height_m_ = 0.3;

    // Metric standoff from the yolo node (depth fusion, bbox fallback).
    bool use_metric_distance_ = true;
    double target_distance_m_ = 3.0;
    double distance_kp_ = 0.5;
    double distance_deadband_m_ = 0.2;

    double altitude_kp_ = 0.8;
    double altitude_kd_ = 0.2;
    double altitude_offset_m_ = 0.0;
    double max_climb_speed_ = 0.4;
    double max_descent_speed_ = 0.3;

    double max_xy_accel_ = 0.6;
    double max_z_accel_ = 0.5;
    double max_yaw_accel_ = 1.0;

    double landing_descent_speed_ = 0.2;
    double landing_switch_height_ = 0.1;
    double landing_timeout_s_ = 20.0;

    // Ground guard: /mavros/extended_state landed_state + minimum height.
    bool use_landed_state_ = true;
    double landed_state_timeout_s_ = 1.0;
    int landed_state_ = 0;
    bool landed_state_valid_ = false;
    rclcpp::Time last_landed_state_time_{0,0,RCL_STEADY_TIME};

    int swa_up_max_pwm_ = 1211;
    int swa_mid_max_pwm_ = 1757;

    // ---- runtime state ----
    State state_ = State::WaitOffboard;
    Swa swa_ = Swa::Down;
    bool rc_valid_ = false;
    rclcpp::Time last_rc_time_{0,0,RCL_STEADY_TIME};

    bool offboard_ = false;
    bool offboard_prev_ = false;
    bool armed_ = false;
    bool swa_left_down_ = false;

    geometry_msgs::msg::PoseStamped current_pos_;
    bool pose_valid_ = false;
    rclcpp::Time last_pose_time_{0,0,RCL_STEADY_TIME};
    double current_vz_ = 0.0;
    bool velocity_valid_ = false;

    rclcpp::Time last_track_msg_{0,0,RCL_STEADY_TIME};
    int target_state_ = -1;
    double target_x_offset_ = 0.0;
    double target_y_offset_ = 0.0;
    int target_id_ = -1;
    double measured_distance_m_ = 0.0;
    int distance_source_ = 0;
    bool distance_valid_ = false;
    rclcpp::Time last_target_visible_time_{0,0,RCL_STEADY_TIME};
    bool target_was_valid_ = false;

    bool hold_valid_ = false;
    double hold_x_ = 0.0;
    double hold_y_ = 0.0;
    double hold_z_ = 0.0;

    bool lost_hold_valid_ = false;
    double lost_hold_x_ = 0.0;
    double lost_hold_y_ = 0.0;
    double lost_hold_z_ = 0.0;

    rclcpp::Time land_start_time_{0,0,RCL_STEADY_TIME};
    double land_x_ = 0.0;
    double land_y_ = 0.0;
    double land_start_z_ = 0.0;
    double land_target_z_ = 0.0;
    bool auto_land_requested_ = false;
    rclcpp::Time last_auto_land_try_{0,0,RCL_STEADY_TIME};

    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_vz_ = 0.0;
    double cmd_yaw_rate_ = 0.0;

    void loadParams();
    void initializeSubscribers();
    void initializePublishers();

    void localPosCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void velocityCb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg);
    void rcCb(const mavros_msgs::msg::RCIn::ConstSharedPtr &msg);
    void trackCb(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg);
    void extendedStateCb(const mavros_msgs::msg::ExtendedState::ConstSharedPtr &msg);
    void controlTimer();

    bool poseFresh() const;
    bool rcFresh() const;
    bool targetFresh() const;
    bool landedStateFresh() const;
    bool landedStateBlocksTracking() const;

    void enterOffboard();
    void leaveOffboard();
    void captureHold();
    void captureLostHold();
    void enterLanding();

    void publishHold();
    void publishLostHold();
    void publishTrackCommand(double dt);
    void publishLandCommand(double dt);
    void publishPosition(double x, double y, double z);
    void publishVelocity(double vx, double vy, double vz, double yaw_rate);
    void resetCmd();

    static double slew(double current, double target, double max_rate, double dt);
    static const char *swaName(Swa swa);
    static const char *stateName(State state);
};

#endif // SAFINA_TRACK_TEST_H
