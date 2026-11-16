#include "track_test/Safina_track_test.h"

#include <algorithm>
#include <cmath>

Safina_Track::Safina_Track(rclcpp::Node::SharedPtr node_track)
    : node_(node_track)
{
    loadParams();
    initializeSubscribers();
    initializePublishers();

    timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_hz_),
                               std::bind(&Safina_Track::controlTimer, this));

    RCLCPP_INFO(node_->get_logger(), "track_test ready: control_rate=%.1f Hz require_offboard=%d",
             control_rate_hz_, require_offboard_ ? 1 : 0);
    RCLCPP_INFO(node_->get_logger(), "The node never arms and never requests OFFBOARD; the pilot does that.");
    RCLCPP_INFO(node_->get_logger(), "SWA mapping: up<=%d pwm -> protection, <=%d -> follow, above -> land",
             swa_up_max_pwm_, swa_mid_max_pwm_);
}

Safina_Track::~Safina_Track() {}

void Safina_Track::loadParams()
{
    control_rate_hz_ = node_->declare_parameter<double>("control_rate_hz", 20.0);
    require_offboard_ = node_->declare_parameter<bool>("require_offboard", true);
    target_timeout_s_ = node_->declare_parameter<double>("target_timeout_s", 0.5);
    target_loss_grace_s_ = node_->declare_parameter<double>("target_loss_grace_s", 0.3);
    pose_timeout_s_ = node_->declare_parameter<double>("pose_timeout_s", 0.5);
    rc_timeout_s_ = node_->declare_parameter<double>("rc_timeout_s", 1.0);

    tracking_vlxy_ = node_->declare_parameter<double>("tracking_vlxy", 0.8);
    max_forward_speed_ = node_->declare_parameter<double>("max_forward_speed", 0.3);
    max_backward_speed_ = node_->declare_parameter<double>("max_backward_speed", 0.15);
    tracking_ryaw_ = node_->declare_parameter<double>("tracking_ryaw", 1.5);
    max_yaw_rate_ = node_->declare_parameter<double>("max_yaw_rate", 1.2);
    min_tracking_height_m_ = node_->declare_parameter<double>("min_tracking_height_m", 0.3);

    use_metric_distance_ = node_->declare_parameter<bool>("use_metric_distance", true);
    target_distance_m_ = node_->declare_parameter<double>("target_distance_m", 3.0);
    distance_kp_ = node_->declare_parameter<double>("distance_kp", 0.5);
    distance_deadband_m_ = node_->declare_parameter<double>("distance_deadband_m", 0.2);

    altitude_kp_ = node_->declare_parameter<double>("altitude_kp", 0.8);
    altitude_kd_ = node_->declare_parameter<double>("altitude_kd", 0.2);
    altitude_offset_m_ = node_->declare_parameter<double>("altitude_offset_m", 0.0);
    max_climb_speed_ = node_->declare_parameter<double>("max_climb_speed", 0.4);
    max_descent_speed_ = node_->declare_parameter<double>("max_descent_speed", 0.3);

    max_xy_accel_ = node_->declare_parameter<double>("max_xy_accel", 0.6);
    max_z_accel_ = node_->declare_parameter<double>("max_z_accel", 0.5);
    max_yaw_accel_ = node_->declare_parameter<double>("max_yaw_accel", 1.0);

    landing_descent_speed_ = node_->declare_parameter<double>("landing_descent_speed", 0.2);
    landing_switch_height_ = node_->declare_parameter<double>("landing_switch_height", 0.1);
    landing_timeout_s_ = node_->declare_parameter<double>("landing_timeout_s", 20.0);

    use_landed_state_ = node_->declare_parameter<bool>("use_landed_state", true);
    landed_state_timeout_s_ = node_->declare_parameter<double>("landed_state_timeout_s", 1.0);

    swa_up_max_pwm_ = node_->declare_parameter<int>("swa.up_max_pwm", 1211);
    swa_mid_max_pwm_ = node_->declare_parameter<int>("swa.mid_max_pwm", 1757);

    legacy_target_ = node_->declare_parameter<bool>("accept_legacy_target", false);
    state_timeout_s_ = node_->declare_parameter<double>("state_timeout_s", 2.0);
    // ---- validation ----
    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ < 2.0)
        control_rate_hz_ = 20.0;
    if (target_timeout_s_ <= 0.0)
        target_timeout_s_ = 0.5;
    if (target_loss_grace_s_ < 0.0)
        target_loss_grace_s_ = 0.0;
    if (pose_timeout_s_ <= 0.0)
        pose_timeout_s_ = 0.5;
    if (rc_timeout_s_ <= 0.0)
        rc_timeout_s_ = 1.0;
    if (max_forward_speed_ < 0.0)
        max_forward_speed_ = 0.0;
    if (max_backward_speed_ < 0.0)
        max_backward_speed_ = 0.0;
    if (max_yaw_rate_ < 0.0)
        max_yaw_rate_ = 0.0;
    if (target_distance_m_ < 0.5)
        target_distance_m_ = 0.5;
    if (distance_kp_ <= 0.0)
        distance_kp_ = 0.5;
    if (distance_deadband_m_ < 0.0)
        distance_deadband_m_ = 0.0;
    if (max_climb_speed_ < 0.0)
        max_climb_speed_ = 0.0;
    if (max_descent_speed_ < 0.0)
        max_descent_speed_ = 0.0;
    if (landing_descent_speed_ <= 0.0)
        landing_descent_speed_ = 0.2;
    if (landing_switch_height_ < 0.0)
        landing_switch_height_ = 0.1;
    if (landing_timeout_s_ <= 0.0)
        landing_timeout_s_ = 20.0;
    if (landed_state_timeout_s_ <= 0.0)
        landed_state_timeout_s_ = 1.0;
    for (double value : {control_rate_hz_,target_timeout_s_,target_loss_grace_s_,pose_timeout_s_,rc_timeout_s_,
        tracking_vlxy_,max_forward_speed_,max_backward_speed_,tracking_ryaw_,max_yaw_rate_,min_tracking_height_m_,
        target_distance_m_,distance_kp_,distance_deadband_m_,altitude_kp_,altitude_kd_,altitude_offset_m_,
        max_climb_speed_,max_descent_speed_,max_xy_accel_,max_z_accel_,max_yaw_accel_,landing_descent_speed_,
        landing_switch_height_,landing_timeout_s_,landed_state_timeout_s_,state_timeout_s_})
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite tracking parameter");
    if (state_timeout_s_<=0 || max_xy_accel_<=0 || max_z_accel_<=0 || max_yaw_accel_<=0 ||
        control_rate_hz_>200 || tracking_vlxy_<0 || tracking_ryaw_<0 || min_tracking_height_m_<0 || altitude_kp_<0 || altitude_kd_<0)
        throw std::invalid_argument("Invalid tracking dynamics or freshness parameter");
    if (swa_up_max_pwm_ >= swa_mid_max_pwm_)
    {
        RCLCPP_WARN(node_->get_logger(), "SWA thresholds invalid (up_max=%d mid_max=%d), using 1211/1757",
                 swa_up_max_pwm_, swa_mid_max_pwm_);
        swa_up_max_pwm_ = 1211;
        swa_mid_max_pwm_ = 1757;
    }
}

void Safina_Track::initializeSubscribers()
{
    auto qos=rclcpp::SensorDataQoS();
    sub_local_pos_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",qos,std::bind(&Safina_Track::localPosCb,this,std::placeholders::_1));
    sub_velocity_=node_->create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/local_position/velocity_local",qos,std::bind(&Safina_Track::velocityCb,this,std::placeholders::_1));
    sub_rc_=node_->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",qos,std::bind(&Safina_Track::rcCb,this,std::placeholders::_1));
    sub_extended_state_=node_->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",qos,std::bind(&Safina_Track::extendedStateCb,this,std::placeholders::_1));
    sub_state_=node_->create_subscription<mavros_msgs::msg::State>("/mavros/state",qos,std::bind(&Safina_Track::stateCb,this,std::placeholders::_1));
    if (legacy_target_)
        sub_track_=node_->create_subscription<std_msgs::msg::Float32MultiArray>("/Safina_track",1,std::bind(&Safina_Track::trackCb,this,std::placeholders::_1));
    else
        sub_stamped_=node_->create_subscription<yolo_ultra_test::msg::TrackedTarget>("/Safina_track_stamped",1,std::bind(&Safina_Track::stampedCb,this,std::placeholders::_1));
    mode_client_=node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void Safina_Track::initializePublishers()
{
    pub_setraw_=node_->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local",10);
}

bool Safina_Track::sourceFresh(const rclcpp::Time &stamp,double limit) const
{
    const double age=(node_->now()-stamp).seconds();
    return stamp.nanoseconds()>0 && age>=-0.05 && age<=limit;
}

bool Safina_Track::stateFresh() const
{
    return last_state_time_.nanoseconds()>0 && (steadyNow()-last_state_time_).seconds()<=state_timeout_s_ &&
        sourceFresh(state_stamp_,state_timeout_s_) && fc_state_.connected;
}

void Safina_Track::stateCb(mavros_msgs::msg::State::ConstSharedPtr msg)
{
    state_stamp_=msg->header.stamp; last_state_time_=steadyNow(); fc_state_=*msg;
}

void Safina_Track::stampedCb(yolo_ultra_test::msg::TrackedTarget::ConstSharedPtr msg)
{
    target_state_=-1; distance_valid_=false;
    target_stamp_=msg->header.stamp;
    if (!msg->session_active || msg->header.frame_id.empty() || !sourceFresh(target_stamp_,target_timeout_s_) ||
        !std::isfinite(msg->offset_x) || !std::isfinite(msg->offset_y) ||
        std::abs(msg->offset_x)>1.0 || std::abs(msg->offset_y)>1.0 ||
        msg->state!=0 || msg->target_id<1) return;
    last_track_msg_=steadyNow(); target_state_=0; target_id_=msg->target_id;
    target_x_offset_=msg->offset_x; target_y_offset_=msg->offset_y;
    measured_distance_m_=msg->distance_m; distance_source_=msg->distance_source;
    distance_valid_=msg->distance_valid && std::isfinite(measured_distance_m_) && measured_distance_m_>0 &&
        (distance_source_==1 || distance_source_==2);
}

void Safina_Track::requestAutoLand()
{
    if (mode_pending_ || !stateFresh() || !fc_state_.armed || fc_state_.mode!="OFFBOARD" ||
        !poseFresh() || !rcFresh() || swa_!=Swa::Down || !mode_client_->service_is_ready()) return;
    auto request=std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode="AUTO.LAND"; mode_pending_=true;
    mode_request_at_=steadyNow();
    auto pending=mode_client_->async_send_request(request,[this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture result) {
        mode_pending_=false;
        try { auto_land_requested_=result.get()->mode_sent; }
        catch (const std::exception &e) { RCLCPP_WARN(node_->get_logger(),"AUTO.LAND response: %s",e.what()); }
    });
    mode_request_id_=pending.request_id;
}

void Safina_Track::localPosCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
    pose_stamp_=msg->header.stamp;
    const auto &p=msg->pose.position; const auto &q=msg->pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    pose_valid_=false;
    if (msg->header.frame_id.empty() || !sourceFresh(pose_stamp_,pose_timeout_s_) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm-1.0)>0.05) return;
    if (!pose_frame_.empty() && pose_frame_!=msg->header.frame_id) return;
    pose_frame_=msg->header.frame_id;
    current_pos_ = *msg;
    pose_valid_ = true;
    last_pose_time_ = steadyNow();
}

void Safina_Track::velocityCb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
{
    velocity_valid_=false;
    velocity_stamp_=msg->header.stamp; velocity_frame_=msg->header.frame_id;
    if (!sourceFresh(velocity_stamp_,pose_timeout_s_) || !std::isfinite(msg->twist.linear.z)) return;
    last_velocity_time_=steadyNow();
    current_vz_ = msg->twist.linear.z;  // ENU: positive up
    velocity_valid_ = true;
}

void Safina_Track::rcCb(const mavros_msgs::msg::RCIn::ConstSharedPtr &msg)
{
    rc_valid_=false; rc_stamp_=msg->header.stamp;
    if (msg->channels.size() <= 8 || !sourceFresh(rc_stamp_,rc_timeout_s_)) return;

    const int pwm = msg->channels[8];
    if (pwm<800 || pwm>2200) return;
    if (pwm <= swa_up_max_pwm_)
        swa_ = Swa::Up;
    else if (pwm <= swa_mid_max_pwm_)
        swa_ = Swa::Mid;
    else
        swa_ = Swa::Down;

    rc_valid_ = true;
    last_rc_time_ = steadyNow();
}

void Safina_Track::trackCb(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg)
{
    target_state_=-1; distance_valid_=false;
    if (msg->data.size()!=7) return;
    for (float v : msg->data) if (!std::isfinite(v)) return;
    if (std::abs(msg->data[1])>1 || std::abs(msg->data[2])>1 || msg->data[0]!=0 || msg->data[3]<1) return;
    last_track_msg_ = steadyNow();
    target_stamp_=node_->now();
    if (msg->data.size() >= 3)
    {
        target_state_ = static_cast<int>(msg->data[0]);
        target_x_offset_ = msg->data[1];
        target_y_offset_ = msg->data[2];
    }
    if (msg->data.size() >= 4)
        target_id_ = static_cast<int>(msg->data[3]);
    if (msg->data.size() >= 7)
    {
        measured_distance_m_ = msg->data[4];
        distance_source_ = static_cast<int>(msg->data[5]);
        distance_valid_ = msg->data[6] > 0.5f && measured_distance_m_>0 && (distance_source_==1 || distance_source_==2);
    }
    else
    {
        measured_distance_m_ = 0.0;
        distance_source_ = 0;
        distance_valid_ = false;
    }
}

void Safina_Track::extendedStateCb(const mavros_msgs::msg::ExtendedState::ConstSharedPtr &msg)
{
    landed_stamp_=msg->header.stamp;
    landed_state_ = msg->landed_state;
    landed_state_valid_ = true;
    last_landed_state_time_ = steadyNow();
}

bool Safina_Track::poseFresh() const
{
    return pose_valid_ && sourceFresh(pose_stamp_,pose_timeout_s_) && (steadyNow() - last_pose_time_).seconds() <= pose_timeout_s_;
}

bool Safina_Track::rcFresh() const
{
    return rc_valid_ && sourceFresh(rc_stamp_,rc_timeout_s_) && (steadyNow() - last_rc_time_).seconds() <= rc_timeout_s_;
}

bool Safina_Track::targetFresh() const
{
    return (last_track_msg_.nanoseconds()!=0) &&
           (steadyNow() - last_track_msg_).seconds() <= target_timeout_s_ &&
           sourceFresh(target_stamp_,target_timeout_s_) && target_state_ == 0;
}

bool Safina_Track::landedStateFresh() const
{
    return landed_state_valid_ && sourceFresh(landed_stamp_,landed_state_timeout_s_) &&
           (steadyNow() - last_landed_state_time_).seconds() <= landed_state_timeout_s_;
}

bool Safina_Track::landedStateBlocksTracking() const
{
    if (!use_landed_state_) return false;
    if (!landedStateFresh()) return true;
    return landed_state_ != mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
}

void Safina_Track::controlTimer()
{
    const auto tick=steadyNow();
    double dt = last_tick_.nanoseconds()>0 ? (tick-last_tick_).seconds() : 1.0/control_rate_hz_;
    last_tick_=tick;
    if (mode_pending_ && (tick-mode_request_at_).seconds()>2.0) {
        mode_client_->remove_pending_request(mode_request_id_); mode_pending_=false;
    }
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5)
        dt = 1.0 / control_rate_hz_;

    if (!stateFresh()) {
        if (offboard_prev_) leaveOffboard();
        offboard_prev_=false; offboard_=false; resetCmd(); return;
    }
    offboard_ = fc_state_.mode=="OFFBOARD";
    if (!require_offboard_)
        offboard_ = true;

    const bool new_armed = fc_state_.armed;
    if (new_armed != armed_)
    {
        armed_ = new_armed;
        RCLCPP_INFO(node_->get_logger(), "PX4 armed=%d", armed_ ? 1 : 0);
    }

    if (offboard_ != offboard_prev_)
    {
        if (offboard_)
            enterOffboard();
        else
            leaveOffboard();
        offboard_prev_ = offboard_;
    }

    if (!offboard_)
    {
        // Requirement: if the RC is not in OFFBOARD, never execute SWA
        // actions. We only keep a neutral position setpoint stream alive so
        // that PX4 accepts the pilot's switch into OFFBOARD. The pilot has
        // full control in Position/Altitude/Stabilized modes.
        state_ = State::WaitOffboard;
        hold_valid_ = false;
        lost_hold_valid_ = false;
        target_was_valid_ = false;
        swa_left_down_ = false;
        resetCmd();

        if (poseFresh())
            publishPosition(current_pos_.pose.position.x,
                            current_pos_.pose.position.y,
                            current_pos_.pose.position.z);
        else
            publishVelocity(0.0, 0.0, 0.0, 0.0);
        return;
    }

    if (!armed_) {
        state_=State::Hold; hold_valid_=false; captureHold(); resetCmd(); publishHold(); return;
    }
    const bool rc_fresh = rcFresh();
    if (rc_fresh && swa_ != Swa::Down)
        swa_left_down_ = true;

    switch (state_)
    {
    case State::WaitOffboard:
        enterOffboard();
        break;

    case State::Hold:
        if (!rc_fresh)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "RC data stale: holding, SWA ignored");
            captureHold();
            publishHold();
            break;
        }

        if (swa_ == Swa::Mid)
        {
            if (!poseFresh())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "SWA mid but pose stale: holding");
                publishHold();
            }
            else if (landedStateBlocksTracking())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "SWA mid but landed_state=%d (on ground/landing): holding",
                                  landed_state_);
                publishHold();
            }
            else if (current_pos_.pose.position.z < min_tracking_height_m_)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "SWA mid but z=%.2f < min_tracking_height=%.2f: holding, take off first",
                                  current_pos_.pose.position.z, min_tracking_height_m_);
                publishHold();
            }
            else
            {
                state_ = State::Track;
                lost_hold_valid_ = false;
                target_was_valid_ = false;
                resetCmd();
                RCLCPP_INFO(node_->get_logger(), "SWA mid: tracking enabled (z=%.2f)",
                         current_pos_.pose.position.z);
                publishHold();
            }
        }
        else if (swa_ == Swa::Down && swa_left_down_)
        {
            enterLanding();
        }
        else
        {
            publishHold();
            if (swa_ == Swa::Up)
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "SWA up in OFFBOARD: protection hold "
                                       "(no takeoff, no altitude change)");
        }
        break;

    case State::Track:
        if (!rc_fresh)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "RC data stale: braking and holding");
            captureHold();
            resetCmd();
            publishHold();
            break;
        }

        if (landedStateBlocksTracking())
        {
            captureHold();
            state_ = State::Hold;
            resetCmd();
            publishHold();
            RCLCPP_WARN(node_->get_logger(), "landed_state=%d while tracking: stopping tracking and holding", landed_state_);
            break;
        }

        if (swa_ == Swa::Up)
        {
            captureHold();
            state_ = State::Hold;
            resetCmd();
            publishHold();
            RCLCPP_WARN(node_->get_logger(), "SWA up: protection hold");
        }
        else if (swa_ == Swa::Down && swa_left_down_)
        {
            enterLanding();
        }
        else if (targetFresh())
        {
            last_target_visible_time_ = steadyNow();
            target_was_valid_ = true;
            lost_hold_valid_ = false;
            publishTrackCommand(dt);
        }
        else
        {
            const bool in_grace = target_was_valid_ &&
                                  (last_target_visible_time_.nanoseconds()!=0) &&
                                  (steadyNow() - last_target_visible_time_).seconds() <
                                      target_loss_grace_s_;
            if (in_grace)
            {
                cmd_vx_ = slew(cmd_vx_, 0.0, max_xy_accel_, dt);
                cmd_vz_ = slew(cmd_vz_, 0.0, max_z_accel_, dt);
                cmd_yaw_rate_ = slew(cmd_yaw_rate_, 0.0, max_yaw_accel_, dt);
                cmd_vy_ = 0.0;
                publishVelocity(cmd_vx_, 0.0, cmd_vz_, cmd_yaw_rate_);
            }
            else
            {
                if (target_was_valid_ || !lost_hold_valid_)
                    captureLostHold();
                target_was_valid_ = false;
                publishLostHold();
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No primary target: holding position");
            }
        }
        break;

    case State::Land:
        publishLandCommand(dt);
        break;
    }
}

void Safina_Track::enterOffboard()
{
    state_ = State::Hold;
    hold_valid_ = false;
    captureHold();
    lost_hold_valid_ = false;
    target_was_valid_ = false;
    swa_left_down_ = rcFresh() && (swa_ != Swa::Down);
    resetCmd();

    RCLCPP_INFO(node_->get_logger(), "OFFBOARD entered: holding at (%.2f, %.2f, %.2f), SWA=%s",
             hold_x_, hold_y_, hold_z_, swaName(swa_));
}

void Safina_Track::leaveOffboard()
{
    state_ = State::WaitOffboard;
    hold_valid_ = false;
    lost_hold_valid_ = false;
    target_was_valid_ = false;
    swa_left_down_ = false;
    resetCmd();

    RCLCPP_INFO(node_->get_logger(), "Left OFFBOARD: SWA actions disabled, pilot has control");
}

void Safina_Track::captureHold()
{
    if (!poseFresh())
        return;

    hold_x_ = current_pos_.pose.position.x;
    hold_y_ = current_pos_.pose.position.y;
    hold_z_ = current_pos_.pose.position.z;
    hold_valid_ = true;
}

void Safina_Track::captureLostHold()
{
    if (!poseFresh())
        return;

    lost_hold_x_ = current_pos_.pose.position.x;
    lost_hold_y_ = current_pos_.pose.position.y;
    lost_hold_z_ = current_pos_.pose.position.z;
    lost_hold_valid_ = true;
}

void Safina_Track::enterLanding()
{
    state_ = State::Land;
    land_start_time_ = steadyNow();
    auto_land_requested_ = false;
    last_auto_land_try_ = rclcpp::Time(0,0,RCL_STEADY_TIME);
    resetCmd();

    if (poseFresh())
    {
        land_x_ = current_pos_.pose.position.x;
        land_y_ = current_pos_.pose.position.y;
        land_start_z_ = current_pos_.pose.position.z;
    }
    else if (hold_valid_)
    {
        land_x_ = hold_x_;
        land_y_ = hold_y_;
        land_start_z_ = hold_z_;
    }
    else
    {
        land_x_ = 0.0;
        land_y_ = 0.0;
        land_start_z_ = 0.0;
        RCLCPP_WARN(node_->get_logger(), "Landing requested without a valid pose; XY hold unavailable");
    }
    land_target_z_ = land_start_z_;

    RCLCPP_INFO(node_->get_logger(), "Landing in place at (%.2f, %.2f), z=%.2f", land_x_, land_y_, land_start_z_);
}

void Safina_Track::publishHold()
{
    if (!hold_valid_)
        captureHold();

    if (hold_valid_ && poseFresh())
        publishPosition(hold_x_, hold_y_, hold_z_);
    else
        publishVelocity(0.0, 0.0, 0.0, 0.0);
}

void Safina_Track::publishLostHold()
{
    if (!lost_hold_valid_)
        captureLostHold();

    if (lost_hold_valid_ && poseFresh())
        publishPosition(lost_hold_x_, lost_hold_y_, lost_hold_z_);
    else
        publishVelocity(0.0, 0.0, 0.0, 0.0);
}

void Safina_Track::publishTrackCommand(double dt)
{
    if (!poseFresh())
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Pose stale: braking");
        cmd_vx_ = slew(cmd_vx_, 0.0, max_xy_accel_, dt);
        cmd_vz_ = slew(cmd_vz_, 0.0, max_z_accel_, dt);
        cmd_yaw_rate_ = slew(cmd_yaw_rate_, 0.0, max_yaw_accel_, dt);
        cmd_vy_ = 0.0;
        publishVelocity(cmd_vx_, 0.0, cmd_vz_, cmd_yaw_rate_);
        return;
    }

    if (!hold_valid_)
        captureHold();

    const double current_z = current_pos_.pose.position.z;
    const double target_z = (hold_valid_ ? hold_z_ : current_z) + altitude_offset_m_;

    double vz = altitude_kp_ * (target_z - current_z);
    if (velocity_valid_ && sourceFresh(velocity_stamp_,pose_timeout_s_) &&
        (steadyNow()-last_velocity_time_).seconds()<=pose_timeout_s_ && velocity_frame_==pose_frame_)
        vz -= altitude_kd_ * current_vz_;
    vz = std::max(-max_descent_speed_, std::min(max_climb_speed_, vz));

    double vx = 0.0;
    if (use_metric_distance_ && distance_valid_ && measured_distance_m_ > 0.0)
    {
        const double err = measured_distance_m_ - target_distance_m_;
        if (std::fabs(err) <= distance_deadband_m_)
            vx = 0.0;
        else
            vx = distance_kp_ * err;
        vx = std::max(-max_backward_speed_, std::min(max_forward_speed_, vx));
    }
    else
    {
        // Fallback: original pixel-radial heuristic.
        vx = tracking_vlxy_ * target_x_offset_;
        vx = std::max(-max_backward_speed_, std::min(max_forward_speed_, vx));
    }

    double yaw_rate = tracking_ryaw_ * target_y_offset_;
    yaw_rate = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, yaw_rate));

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "tracking distance=%.2f m target=%.2f m source=%d valid=%d vx=%.2f",
        measured_distance_m_, target_distance_m_, distance_source_,
        distance_valid_ ? 1 : 0, vx);

    cmd_vx_ = slew(cmd_vx_, vx, max_xy_accel_, dt);
    cmd_vz_ = slew(cmd_vz_, vz, max_z_accel_, dt);
    cmd_yaw_rate_ = slew(cmd_yaw_rate_, yaw_rate, max_yaw_accel_, dt);
    cmd_vy_ = 0.0;

    publishVelocity(cmd_vx_, 0.0, cmd_vz_, cmd_yaw_rate_);
}

void Safina_Track::publishLandCommand(double dt)
{
    if (!poseFresh() || !rcFresh()) { resetCmd(); publishVelocity(0,0,0,0); return; }
    // Landing remains latched like the deployed controller; a pilot mode exit cancels it.
    land_target_z_-=landing_descent_speed_*dt;
    land_target_z_=std::max(0.03,std::min(land_target_z_,current_pos_.pose.position.z-0.05));
    publishPosition(land_x_,land_y_,land_target_z_);
    const auto now=steadyNow();
    if (!auto_land_requested_ && (current_pos_.pose.position.z<=landing_switch_height_ || (now-land_start_time_).seconds()>=landing_timeout_s_)) {
        if (last_auto_land_try_.nanoseconds()==0 || (now-last_auto_land_try_).seconds()>=1.0) {
            last_auto_land_try_=now; requestAutoLand();
        }
    }
}

void Safina_Track::publishPosition(double x, double y, double z)
{
    mavros_msgs::msg::PositionTarget cmd;
    cmd.header.stamp = node_->now();
    // mavros interprets FRAME_LOCAL_NED as a local ENU setpoint and converts
    // it to NED for PX4. Keep using this convention (same as land_test).
    cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::FORCE |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    cmd.position.x = x;
    cmd.position.y = y;
    cmd.position.z = z;
    cmd.yaw = 0.0;
    cmd.yaw_rate = 0.0;
    pub_setraw_->publish(cmd);
}

void Safina_Track::publishVelocity(double vx, double vy, double vz, double yaw_rate)
{
    mavros_msgs::msg::PositionTarget cmd;
    cmd.header.stamp = node_->now();
    // mavros treats FRAME_BODY_NED as body FLU and converts it to FRD for
    // PX4, so +x is forward, +y is left, +z is up, and +yaw_rate is CCW.
    cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
    cmd.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::FORCE |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    cmd.velocity.x = vx;
    cmd.velocity.y = vy;
    cmd.velocity.z = vz;
    cmd.yaw = 0.0;
    cmd.yaw_rate = yaw_rate;
    pub_setraw_->publish(cmd);
}

void Safina_Track::resetCmd()
{
    cmd_vx_ = 0.0;
    cmd_vy_ = 0.0;
    cmd_vz_ = 0.0;
    cmd_yaw_rate_ = 0.0;
}

double Safina_Track::slew(double current, double target, double max_rate, double dt)
{
    if (max_rate <= 0.0 || dt <= 0.0)
        return target;

    const double max_delta = max_rate * dt;
    if (target > current + max_delta)
        return current + max_delta;
    if (target < current - max_delta)
        return current - max_delta;
    return target;
}

const char *Safina_Track::swaName(Swa swa)
{
    switch (swa)
    {
    case Swa::Up:
        return "up(protection)";
    case Swa::Mid:
        return "mid(follow)";
    case Swa::Down:
        return "down(land/hold)";
    }
    return "unknown";
}

const char *Safina_Track::stateName(State state)
{
    switch (state)
    {
    case State::WaitOffboard:
        return "WAIT_OFFBOARD";
    case State::Hold:
        return "HOLD";
    case State::Track:
        return "TRACK";
    case State::Land:
        return "LAND";
    }
    return "unknown";
}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv); auto node=std::make_shared<rclcpp::Node>("track");
    int status=0;
    try { Safina_Track tracker(node); rclcpp::spin(node); }
    catch(const std::exception &e) { RCLCPP_ERROR(node->get_logger(),"%s",e.what()); status=1; }
    rclcpp::shutdown(); return status;
}
