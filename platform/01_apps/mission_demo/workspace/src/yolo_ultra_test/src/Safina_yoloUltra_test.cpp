#include "yoloUltra_test/Safina_yoloUltra_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "process/postprocess.h"

Safina_YoloUltra::Safina_YoloUltra(const rclcpp::Node::SharedPtr &node_yoloUltra)
    : node_(node_yoloUltra)
{
    loadParams();
    initializeSubscribers();
    initializePublishers();

    const nn_error_e load_ret = yoloUltra.LoadModel(yoloUltra_model_file.c_str());
    if (load_ret != NN_SUCCESS)
        throw std::runtime_error("Failed to load RKNN model: " + yoloUltra_model_file);

    // Apply detection thresholds read from the YAML file.
    yolo::setObjectThreshold(static_cast<float>(object_threshold_));
    yolo::setNmsThreshold(static_cast<float>(nms_threshold_));

    // Camera intrinsics: K = [fx 0 cx; 0 fy cy; 0 0 1]. These are not used by
    // the current pixel-offset tracking path, but they are correct for any
    // future metric-distance work.
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        camera_fx, 0.0, camera_cx,
        0.0, camera_fy, camera_cy,
        0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
        -0.033186350017786026, 0.03535980358719826, -0.0002135216782335192,
        -0.0001933806634042412, -0.012397058308124542);

    PersonTracker::Params tracker_params;
    tracker_params.new_track_confidence = new_track_confidence_;
    tracker_params.match_confidence = match_confidence_;
    tracker_params.assoc_iou_threshold = assoc_iou_threshold_;
    tracker_params.assoc_center_ratio = assoc_center_ratio_;
    tracker_params.assoc_size_tol = assoc_size_tol_;
    tracker_params.track_loss_timeout_s = track_loss_timeout_s_;
    tracker_params.max_tracks = max_tracks_;
    tracker_params.primary_iou_min = primary_iou_min_;
    tracker_params.primary_center_ratio = primary_center_ratio_;
    tracker_params.primary_size_tol = primary_size_tol_;
    tracker_params.primary_predict_s = primary_predict_s_;
    tracker_params.max_predict_speed_ratio = max_predict_speed_ratio_;
    tracker_ = PersonTracker(tracker_params);

    RCLCPP_INFO(node_->get_logger(), "yoloUltra_test ready: model=%s image=%dx%d",
             yoloUltra_model_file.c_str(), img_cols, img_rows);
    RCLCPP_INFO(node_->get_logger(), "detection: object_threshold=%.2f nms=%.2f class_id=%d",
             object_threshold_, nms_threshold_, class_id_);
    RCLCPP_INFO(node_->get_logger(), "tracker: new_track_conf=%.2f match_conf=%.2f iou=%.2f center_ratio=%.2f loss_timeout=%.1fs reacquire=%s",
             new_track_confidence_, match_confidence_, assoc_iou_threshold_,
             assoc_center_ratio_, track_loss_timeout_s_, reacquire_mode_.c_str());
    RCLCPP_INFO(node_->get_logger(), "primary gate: center_ratio=%.2f size_tol=%.2f iou=%.2f predict=%.2fs reacquire_radius=%.2f",
             primary_center_ratio_, primary_size_tol_, primary_iou_min_,
             primary_predict_s_, reacquire_radius_ratio_);
    if (!require_offboard_)
        RCLCPP_WARN(node_->get_logger(), "BENCH MODE: tracker/require_offboard=false. IDs are assigned in "
                 "POSCTL + SWA mid for ground display. This node has no flight-control output.");
    RCLCPP_INFO(node_->get_logger(), "distance: depth_enabled=%d person_height=%.2f m filter_frames=%d",
             depth_enabled_ ? 1 : 0, person_height_m_, distance_filter_frames_);
}

Safina_YoloUltra::~Safina_YoloUltra() {}

void Safina_YoloUltra::loadParams()
{
    yoloUltra_model_file = node_->declare_parameter<std::string>("model_file", "/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install/yolo_ultra_test/share/yolo_ultra_test/weights/yolov11_s_det_int.rknn");

    img_rows = node_->declare_parameter<int>("camera_rows", 720);
    img_cols = node_->declare_parameter<int>("camera_cols", 1280);

    double fx = camera_fx, fy = camera_fy, cx = camera_cx, cy = camera_cy;
    fx = node_->declare_parameter<double>("camera_fx", fx);
    fy = node_->declare_parameter<double>("camera_fy", fy);
    cx = node_->declare_parameter<double>("camera_cx", cx);
    cy = node_->declare_parameter<double>("camera_cy", cy);
    camera_fx = static_cast<float>(fx);
    camera_fy = static_cast<float>(fy);
    camera_cx = static_cast<float>(cx);
    camera_cy = static_cast<float>(cy);

    object_threshold_ = node_->declare_parameter<double>("detection.object_threshold", 0.20);
    nms_threshold_ = node_->declare_parameter<double>("detection.nms_threshold", 0.25);
    draw_confidence_ = node_->declare_parameter<double>("detection.draw_confidence", 0.50);
    class_id_ = node_->declare_parameter<int>("detection.class_id", 0);

    new_track_confidence_ = node_->declare_parameter<double>("tracker.new_track_confidence", 0.50);
    match_confidence_ = node_->declare_parameter<double>("tracker.match_confidence", 0.20);
    assoc_iou_threshold_ = node_->declare_parameter<double>("tracker.assoc_iou_threshold", 0.30);
    assoc_center_ratio_ = node_->declare_parameter<double>("tracker.assoc_center_ratio", 0.08);
    assoc_size_tol_ = node_->declare_parameter<double>("tracker.assoc_size_tol", 0.60);
    track_loss_timeout_s_ = node_->declare_parameter<double>("tracker.track_loss_timeout_s", 1.50);
    reacquire_mode_ = node_->declare_parameter<std::string>("tracker.reacquire_mode", "new_track");
    max_tracks_ = node_->declare_parameter<int>("tracker.max_tracks", 20);
    require_offboard_ = node_->declare_parameter<bool>("tracker.require_offboard", true);
    primary_iou_min_ = node_->declare_parameter<double>("tracker.primary_iou_min", 0.15);
    primary_center_ratio_ = node_->declare_parameter<double>("tracker.primary_center_ratio", 0.10);
    primary_size_tol_ = node_->declare_parameter<double>("tracker.primary_size_tol", 0.35);
    primary_predict_s_ = node_->declare_parameter<double>("tracker.primary_predict_s", 0.20);
    max_predict_speed_ratio_ = node_->declare_parameter<double>("tracker.max_predict_speed_ratio", 0.50);
    reacquire_radius_ratio_ = node_->declare_parameter<double>("tracker.reacquire_radius_ratio", 0.25);
    reacquire_size_min_ratio_ = node_->declare_parameter<double>("tracker.reacquire_size_min_ratio", 0.50);
    reacquire_size_max_ratio_ = node_->declare_parameter<double>("tracker.reacquire_size_max_ratio", 2.00);
    reacquire_fallback_s_ = node_->declare_parameter<double>("tracker.reacquire_fallback_s", 1.50);

    standoff_deadband_px_ = node_->declare_parameter<double>("offset.standoff_deadband_px", 70.0);
    standoff_full_scale_px_ = node_->declare_parameter<double>("offset.standoff_full_scale_px", 150.0);

    depth_enabled_ = node_->declare_parameter<bool>("depth.enabled", true);
    depth_fx_ = node_->declare_parameter<double>("depth.fx", depth_fx_);
    depth_fy_ = node_->declare_parameter<double>("depth.fy", depth_fy_);
    depth_cx_ = node_->declare_parameter<double>("depth.cx", depth_cx_);
    depth_cy_ = node_->declare_parameter<double>("depth.cy", depth_cy_);
    depth_max_age_s_ = node_->declare_parameter<double>("depth.max_age_s", 0.30);
    depth_min_range_m_ = node_->declare_parameter<double>("depth.min_range_m", 0.30);
    depth_max_range_m_ = node_->declare_parameter<double>("depth.max_range_m", 8.0);
    depth_roi_x_min_ = node_->declare_parameter<double>("depth.roi_x_min", 0.30);
    depth_roi_x_max_ = node_->declare_parameter<double>("depth.roi_x_max", 0.70);
    depth_roi_y_min_ = node_->declare_parameter<double>("depth.roi_y_min", 0.25);
    depth_roi_y_max_ = node_->declare_parameter<double>("depth.roi_y_max", 0.65);
    depth_grid_ = node_->declare_parameter<int>("depth.grid", 7);
    depth_min_valid_ratio_ = node_->declare_parameter<double>("depth.min_valid_ratio", 0.30);
    depth_outlier_m_ = node_->declare_parameter<double>("depth.outlier_m", 0.50);

    person_height_m_ = node_->declare_parameter<double>("distance.person_height_m", 1.70);
    distance_filter_frames_ = node_->declare_parameter<int>("distance.filter_frames", 5);
    min_bbox_height_px_ = node_->declare_parameter<double>("distance.min_bbox_height_px", 80.0);

    image_max_age_s_ = node_->declare_parameter("input.image_max_age_s", 0.5);
    input_max_age_s_ = node_->declare_parameter("input.control_max_age_s", 1.0);
    if (!std::isfinite(image_max_age_s_) || image_max_age_s_<=0 || !std::isfinite(input_max_age_s_) || input_max_age_s_<=0)
        throw std::invalid_argument("Invalid input freshness parameters");
    std::vector<double> rotation, translation;
    if ((rotation = node_->declare_parameter<std::vector<double>>("depth.extrinsic_rotation", std::vector<double>{})).size() == 9)
        depth_R_ = rotation;
    else
    {
        depth_enabled_ = false;
        RCLCPP_WARN(node_->get_logger(), "depth fusion disabled: depth/extrinsic_rotation must be a 9-element list");
    }
    if ((translation = node_->declare_parameter<std::vector<double>>("depth.extrinsic_translation", std::vector<double>{})).size() == 3)
        depth_t_ = translation;
    else
    {
        depth_enabled_ = false;
        RCLCPP_WARN(node_->get_logger(), "depth fusion disabled: depth/extrinsic_translation must be a 3-element list");
    }

    swa_up_max_pwm_ = node_->declare_parameter<int>("swa.up_max_pwm", 1211);
    swa_mid_max_pwm_ = node_->declare_parameter<int>("swa.mid_max_pwm", 1757);

    // ---- validation ----
    if (object_threshold_ < 0.0 || object_threshold_ > 1.0)
        object_threshold_ = 0.20;
    if (nms_threshold_ < 0.0 || nms_threshold_ > 1.0)
        nms_threshold_ = 0.25;
    if (draw_confidence_ < 0.0 || draw_confidence_ > 1.0)
        draw_confidence_ = 0.50;
    if (new_track_confidence_ < match_confidence_)
        new_track_confidence_ = match_confidence_;
    if (standoff_full_scale_px_ < 1.0)
        standoff_full_scale_px_ = 1.0;
    if (track_loss_timeout_s_ <= 0.0)
        track_loss_timeout_s_ = 1.50;
    if (assoc_size_tol_ < 0.0)
        assoc_size_tol_ = 0.0;
    if (assoc_size_tol_ > 1.0)
        assoc_size_tol_ = 1.0;
    if (primary_iou_min_ < 0.0)
        primary_iou_min_ = 0.0;
    if (primary_iou_min_ > 1.0)
        primary_iou_min_ = 1.0;
    if (primary_center_ratio_ <= 0.0)
        primary_center_ratio_ = 0.10;
    if (primary_size_tol_ < 0.0)
        primary_size_tol_ = 0.0;
    if (primary_size_tol_ > 1.0)
        primary_size_tol_ = 1.0;
    if (primary_predict_s_ < 0.0)
        primary_predict_s_ = 0.0;
    if (primary_predict_s_ > 2.0)
        primary_predict_s_ = 2.0;
    if (max_predict_speed_ratio_ <= 0.0)
        max_predict_speed_ratio_ = 0.50;
    if (reacquire_radius_ratio_ <= 0.0)
        reacquire_radius_ratio_ = 0.25;
    if (reacquire_size_min_ratio_ <= 0.0)
        reacquire_size_min_ratio_ = 0.50;
    if (reacquire_size_max_ratio_ < reacquire_size_min_ratio_)
        reacquire_size_max_ratio_ = 2.00;
    if (reacquire_fallback_s_ < 0.0)
        reacquire_fallback_s_ = 0.0;
    if (max_tracks_ < 1)
        max_tracks_ = 1;
    if (reacquire_mode_ != "new_track" && reacquire_mode_ != "any_visible")
    {
        RCLCPP_WARN(node_->get_logger(), "Unknown tracker/reacquire_mode '%s', using 'new_track'", reacquire_mode_.c_str());
        reacquire_mode_ = "new_track";
    }
    if (swa_up_max_pwm_ >= swa_mid_max_pwm_)
    {
        RCLCPP_WARN(node_->get_logger(), "SWA thresholds invalid (up_max=%d mid_max=%d), using 1211/1757",
                 swa_up_max_pwm_, swa_mid_max_pwm_);
        swa_up_max_pwm_ = 1211;
        swa_mid_max_pwm_ = 1757;
    }

    if (depth_grid_ < 3)
        depth_grid_ = 3;
    if (depth_grid_ > 15)
        depth_grid_ = 15;
    if (depth_grid_ % 2 == 0)
        depth_grid_ += 1;
    if (depth_max_age_s_ <= 0.0)
        depth_max_age_s_ = 0.30;
    if (depth_min_range_m_ < 0.10)
        depth_min_range_m_ = 0.10;
    if (depth_max_range_m_ <= depth_min_range_m_)
        depth_max_range_m_ = depth_min_range_m_ + 1.0;
    if (depth_roi_x_min_ < 0.0)
        depth_roi_x_min_ = 0.0;
    if (depth_roi_x_max_ > 1.0)
        depth_roi_x_max_ = 1.0;
    if (depth_roi_x_min_ >= depth_roi_x_max_)
    {
        depth_roi_x_min_ = 0.30;
        depth_roi_x_max_ = 0.70;
    }
    if (depth_roi_y_min_ < 0.0)
        depth_roi_y_min_ = 0.0;
    if (depth_roi_y_max_ > 1.0)
        depth_roi_y_max_ = 1.0;
    if (depth_roi_y_min_ >= depth_roi_y_max_)
    {
        depth_roi_y_min_ = 0.25;
        depth_roi_y_max_ = 0.65;
    }
    if (depth_min_valid_ratio_ < 0.0)
        depth_min_valid_ratio_ = 0.0;
    if (depth_min_valid_ratio_ > 1.0)
        depth_min_valid_ratio_ = 1.0;
    if (depth_outlier_m_ <= 0.0)
        depth_outlier_m_ = 0.50;
    if (person_height_m_ < 0.5)
        person_height_m_ = 1.70;
    if (distance_filter_frames_ < 1)
        distance_filter_frames_ = 5;
    if (min_bbox_height_px_ < 10.0)
        min_bbox_height_px_ = 80.0;
    if (!std::isfinite(camera_fx) || !std::isfinite(camera_fy) || camera_fx<=0 || camera_fy<=0 ||
        !std::isfinite(camera_cx) || !std::isfinite(camera_cy) || img_cols<=0 || img_rows<=0)
        throw std::invalid_argument("Invalid color calibration");
    bool extrinsics_valid=depth_R_.size()==9 && depth_t_.size()==3;
    for (double v:depth_R_) extrinsics_valid=extrinsics_valid && std::isfinite(v);
    for (double v:depth_t_) extrinsics_valid=extrinsics_valid && std::isfinite(v);
    if (extrinsics_valid) {
        cv::Mat R(3,3,CV_64F,depth_R_.data());
        extrinsics_valid=cv::norm(R.t()*R-cv::Mat::eye(3,3,CV_64F))<0.01 && std::abs(cv::determinant(R)-1.0)<0.01;
    }
    if (!extrinsics_valid) depth_enabled_=false;
    if (!std::isfinite(depth_fx_) || !std::isfinite(depth_fy_) ||
        !std::isfinite(depth_cx_) || !std::isfinite(depth_cy_) || depth_fx_ <= 0.0 || depth_fy_ <= 0.0)
    {
        depth_enabled_ = false;
        RCLCPP_WARN(node_->get_logger(), "depth fusion disabled: invalid depth intrinsics");
    }
}

void Safina_YoloUltra::initializeSubscribers()
{
    // Reliable is required for the large Orbbec frames on this platform.
    sub_img = node_->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::QoS(1),
        std::bind(&Safina_YoloUltra::readStream, this, std::placeholders::_1));
    sub_depth = node_->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw", rclcpp::QoS(3),
        std::bind(&Safina_YoloUltra::readDepth, this, std::placeholders::_1));
    sub_Rcin = node_->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", rclcpp::SensorDataQoS(),
        std::bind(&Safina_YoloUltra::readRcin, this, std::placeholders::_1));
    sub_state = node_->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(),
        std::bind(&Safina_YoloUltra::state_cb, this, std::placeholders::_1));
    color_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", rclcpp::QoS(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
            if (m->width == static_cast<unsigned>(img_cols) && m->height == static_cast<unsigned>(img_rows) &&
                std::isfinite(m->k[0]) && std::isfinite(m->k[4]) && m->k[0]>0 && m->k[4]>0) {
                camera_fx=m->k[0]; camera_fy=m->k[4]; camera_cx=m->k[2]; camera_cy=m->k[5];
            }
        });
    depth_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::QoS(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
            if (std::isfinite(m->k[0]) && std::isfinite(m->k[4]) && m->k[0]>0 && m->k[4]>0) {
                depth_fx_=m->k[0]; depth_fy_=m->k[4]; depth_cx_=m->k[2]; depth_cy_=m->k[5];
            }
        });
    freshness_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Safina_YoloUltra::watchdog, this));
}

void Safina_YoloUltra::initializePublishers()
{
    pub_yoloUltra_imgresult = node_->create_publisher<sensor_msgs::msg::Image>("/yoloUltra_imgresult", 1);
    pub_track = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/Safina_track", 1);
    pub_target_id = node_->create_publisher<std_msgs::msg::Int32>("/Safina_target_id", 1);
    stamped_pub_ = node_->create_publisher<yolo_ultra_test::msg::TrackedTarget>("/Safina_track_stamped", 1);
}

bool Safina_YoloUltra::fresh(const rclcpp::Time &stamp, const rclcpp::Time &receipt, double limit) const
{
    const double age = (node_->now()-stamp).seconds();
    return stamp.nanoseconds()>0 && receipt.nanoseconds()>0 && age>=-.05 && age<=limit &&
        (rclcpp::Clock(RCL_STEADY_TIME).now()-receipt).seconds()<=limit;
}

void Safina_YoloUltra::watchdog()
{
    updateSessionGate();
    if (!fresh(color_stamp_, color_receipt_, image_max_age_s_)) {
        if (tracking_session_active) endSession();
        publishTrackState(-1, 0, 0, -1);
    }
}

void Safina_YoloUltra::readRcin(const mavros_msgs::msg::RCIn::ConstSharedPtr &msg)
{
    rc_stamp_ = msg->header.stamp;
    rc_receipt_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    if (msg->channels.size() <= 8) { rc_state=-1; updateSessionGate(); return; }

    const int pwm = msg->channels[8];
    int new_state;
    if (pwm <= swa_up_max_pwm_)
        new_state = 0;       // up   -> protection / no tracking session
    else if (pwm <= swa_mid_max_pwm_)
        new_state = 1;       // mid  -> follow
    else
        new_state = 2;       // down -> land

    if (new_state != rc_state)
    {
        rc_state = new_state;
        updateSessionGate();
    }
}

void Safina_YoloUltra::state_cb(const mavros_msgs::msg::State::ConstSharedPtr &msg)
{
    state_stamp_ = msg->header.stamp;
    state_receipt_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    fc_connected_ = msg->connected;
    const bool new_offboard = (msg->mode == "OFFBOARD");
    if (new_offboard != offboard_mode)
    {
        const bool entering_offboard = new_offboard && !offboard_mode;
        offboard_mode = new_offboard;

        if (!require_offboard_ && entering_offboard && tracking_session_active)
        {
            // Bench mode was displaying IDs in POSCTL. Restart the ID session
            // when the real flight starts, so the flight target selection is
            // identical to the normal OFFBOARD + SWA mid workflow.
            endSession();
            session_latched = (rc_state == 1);
            RCLCPP_INFO(node_->get_logger(), "Bench mode: entering OFFBOARD, restarting ID session for flight");
        }
        updateSessionGate();
    }
}

void Safina_YoloUltra::readDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        last_depth_ = cv_bridge::toCvShare(msg, "16UC1")->image;
        last_depth_msg_ = msg;
        last_depth_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "depth image conversion failed: %s", e.what());
    }
}

void Safina_YoloUltra::updateSessionGate()
{
    // Flight mode: IDs only while PX4 OFFBOARD + SWA mid.
    // Bench mode (require_offboard=false): IDs while SWA mid regardless of
    // PX4 mode, for ground display only. This node never publishes setpoints.
    const bool blocked = (rc_state < 0) || (rc_state == 2) ||
        !fresh(color_stamp_, color_receipt_, image_max_age_s_) ||
        !fresh(rc_stamp_, rc_receipt_, input_max_age_s_) ||
        !fresh(state_stamp_, state_receipt_, 2.0) || !fc_connected_ ||
        (require_offboard_ && !offboard_mode);
    if (blocked)
    {
        session_latched = false;
        if (tracking_session_active)
            endSession();
        return;
    }

    if (rc_state == 1)
        session_latched = true;

    if (session_latched && !tracking_session_active)
        startSession();
    else if (!session_latched && tracking_session_active)
        endSession();
}

void Safina_YoloUltra::startSession()
{
    tracker_.reset();
    primary_id = -1;
    waiting_for_new_primary = false;
    primary_last_seen_s = 0.0;
    primary_lost_time_s = 0.0;
    primary_last_center_x = -1.0;
    primary_last_center_y = -1.0;
    primary_last_height = 0.0;
    primary_last_velocity_x = 0.0;
    primary_last_velocity_y = 0.0;
    tracking_session_active = true;
    x_offset = 0.0f;
    y_offset = 0.0f;
    target_state = -1;
    distance_history_.clear();
    distance_history_id_ = -1;
    distance_m_ = 0.0f;
    distance_source_ = 0;
    distance_valid_ = false;
    RCLCPP_INFO(node_->get_logger(), "Person tracking session START (OFFBOARD + SWA mid), IDs start at 1");
}

void Safina_YoloUltra::endSession()
{
    tracker_.reset();
    primary_id = -1;
    waiting_for_new_primary = false;
    primary_last_seen_s = 0.0;
    primary_lost_time_s = 0.0;
    primary_last_center_x = -1.0;
    primary_last_center_y = -1.0;
    primary_last_height = 0.0;
    primary_last_velocity_x = 0.0;
    primary_last_velocity_y = 0.0;
    tracking_session_active = false;
    x_offset = 0.0f;
    y_offset = 0.0f;
    target_state = -1;
    distance_history_.clear();
    distance_history_id_ = -1;
    distance_m_ = 0.0f;
    distance_source_ = 0;
    distance_valid_ = false;
    RCLCPP_INFO(node_->get_logger(), "Person tracking session END");
}

void Safina_YoloUltra::readStream(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    const rclcpp::Time stamp(img_msg->header.stamp);
    const auto receipt = rclcpp::Clock(RCL_STEADY_TIME).now();
    if (!fresh(stamp, receipt, image_max_age_s_) || img_msg->header.frame_id.empty() || stamp<=last_color_stamp_) {
        publishTrackState(-1, 0, 0, -1); return;
    }
    color_stamp_=stamp; color_receipt_=receipt; color_frame_=img_msg->header.frame_id;
    last_color_stamp_=stamp;
    if (static_cast<int>(img_msg->width)!=img_cols || static_cast<int>(img_msg->height)!=img_rows) {
        publishTrackState(-1, 0, 0, -1);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Image dimensions do not match calibration parameters");
        return;
    }
    try {
        cv::Mat img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        std::vector<Detection> detections;
        const auto ret=yoloUltra.Run(img, detections);
        if (ret!=NN_SUCCESS || !fresh(color_stamp_, color_receipt_, image_max_age_s_)) {
            publishTrackState(-1,0,0,-1); return;
        }
        updateSessionGate();
        drawDetections(img, detections);
        if (tracking_session_active) processTracking(img, detections);
        else publishTrackState(-1,0,0,-1);
        pub_yoloUltra_imgresult->publish(*cv_bridge::CvImage(img_msg->header,"bgr8",img).toImageMsg());
    } catch (const cv_bridge::Exception &e) {
        publishTrackState(-1,0,0,-1);
        RCLCPP_WARN(node_->get_logger(), "Color image conversion failed: %s",e.what());
    }
}

void Safina_YoloUltra::processTracking(cv::Mat &img, const std::vector<Detection> &detections)
{
    std::vector<PersonDetection> person_detections;
    person_detections.reserve(detections.size());
    for (const auto &det : detections)
    {
        if (det.class_id != class_id_)
            continue;
        if (det.confidence < static_cast<float>(match_confidence_))
            continue;
        PersonDetection pd;
        pd.box = clampBox(det.box, img_cols, img_rows);
        pd.confidence = det.confidence;
        person_detections.push_back(pd);
    }

    const double now = color_stamp_.seconds();
    tracker_.update(person_detections, now, img_cols, img_rows);

    // ---- primary target selection ----
    // Detect a removed primary first, so a person who appears in the same
    // frame as the timeout can be adopted immediately.
    if (primary_id != -1 && tracker_.find(primary_id) == nullptr)
    {
        RCLCPP_WARN(node_->get_logger(), "Primary target ID=%d lost; holding and waiting for a new person", primary_id);
        primary_lost_time_s = primary_last_seen_s;
        primary_id = -1;
        waiting_for_new_primary = true;
    }

    if (primary_id == -1)
    {
        const PersonTrack *best = nullptr;
        if (!waiting_for_new_primary)
        {
            // First person of this session: the lowest ID is the first person
            // that appeared (IDs are assigned in appearance order).
            for (const auto &track : tracker_.tracks())
            {
                if (!track.matched_this_frame)
                    continue;
                if (best == nullptr || track.id < best->id)
                    best = &track;
            }
        }
        else
        {
            // The old primary left. By default only a new person near the
            // predicted last primary position AND with a similar box size can
            // become the new primary, so a passer-by elsewhere is ignored.
            const double diag = std::hypot(static_cast<double>(img_cols),
                                           static_cast<double>(img_rows));
            const double elapsed = std::max(0.0, now - primary_last_seen_s);
            const double horizon = std::min(elapsed, 1.5);
            const double predicted_x = primary_last_center_x +
                                       primary_last_velocity_x * horizon;
            const double predicted_y = primary_last_center_y +
                                       primary_last_velocity_y * horizon;

            for (const auto &track : tracker_.tracks())
            {
                bool eligible = false;
                if (reacquire_mode_ == "any_visible")
                {
                    eligible = track.matched_this_frame;
                }
                else
                {
                    const bool size_ok =
                        primary_last_height > 0.0 &&
                        track.box.height >= reacquire_size_min_ratio_ * primary_last_height &&
                        track.box.height <= reacquire_size_max_ratio_ * primary_last_height;
                    const bool near =
                        primary_last_center_x >= 0.0 && size_ok &&
                        std::hypot(track.center.x - predicted_x,
                                   track.center.y - predicted_y) <=
                            reacquire_radius_ratio_ * diag;
                    eligible = track.matched_this_frame &&
                               track.first_seen > primary_lost_time_s && near;
                }
                if (!eligible)
                    continue;
                if (best == nullptr || track.id < best->id)
                    best = &track;
            }

            // Fallback: if no near/size candidate appears after the loss
            // timeout plus a grace period, do not wait forever while a person
            // is visible. Pick the visible track closest to the predicted last
            // position and size (or the most confident one if no position was
            // recorded).
            if (best == nullptr && reacquire_fallback_s_ > 0.0 &&
                (now - primary_lost_time_s) >=
                    track_loss_timeout_s_ + reacquire_fallback_s_)
            {
                double best_score = std::numeric_limits<double>::infinity();
                for (const auto &track : tracker_.tracks())
                {
                    if (!track.matched_this_frame)
                        continue;

                    double score = 0.0;
                    if (primary_last_center_x >= 0.0)
                    {
                        const double dist = std::hypot(track.center.x - predicted_x,
                                                       track.center.y - predicted_y);
                        double size_penalty = 0.0;
                        if (primary_last_height > 0.0 && track.box.height > 0)
                            size_penalty = std::fabs(std::log(
                                static_cast<double>(track.box.height) / primary_last_height));
                        score = dist + 0.5 * diag * size_penalty;
                    }
                    else
                    {
                        score = -track.confidence;
                    }

                    if (score < best_score)
                    {
                        best_score = score;
                        best = &track;
                    }
                }

                if (best != nullptr)
                    RCLCPP_INFO(node_->get_logger(), "Reacquire fallback: adopting visible ID=%d after %.1fs",
                             best->id, now - primary_lost_time_s);
            }

            if (best == nullptr)
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Waiting for the original person near the last position/size");
        }

        if (best != nullptr)
        {
            primary_id = best->id;
            waiting_for_new_primary = false;
            RCLCPP_INFO(node_->get_logger(), "Primary target acquired: ID=%d", primary_id);
        }
    }

    tracker_.setPrimary(primary_id);

    // ---- publish offsets for the primary target ----
    bool visible = false;
    cv::Rect target_box;
    if (primary_id != -1)
    {
        const PersonTrack *track = tracker_.find(primary_id);
        if (track != nullptr && track->matched_this_frame)
        {
            visible = true;
            target_box = track->box;
        }
    }

    if (visible)
    {
        primary_last_seen_s = now;
        primary_last_center_x = target_box.x + target_box.width * 0.5;
        primary_last_center_y = target_box.y + target_box.height * 0.5;
        primary_last_height = target_box.height;
        const PersonTrack *primary_track = tracker_.find(primary_id);
        if (primary_track != nullptr)
        {
            primary_last_velocity_x = primary_track->velocity.x;
            primary_last_velocity_y = primary_track->velocity.y;
        }
        computeOffsets(target_box);
        double dist = 0.0;
        int source = 0;
        const bool dist_ok = computeDistance(target_box, dist, source);
        distance_m_ = dist_ok ? static_cast<float>(dist) : 0.0f;
        distance_source_ = source;
        distance_valid_ = dist_ok;
        target_state = 0;
        publishTrackState(0, x_offset, y_offset, primary_id,
                          distance_m_, distance_source_, distance_valid_);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "primary ID=%d x_off=%.2f y_off=%.2f distance=%.2f m source=%s",
            primary_id, x_offset, y_offset, distance_m_,
            distance_source_ == 2 ? "depth" : (distance_source_ == 1 ? "bbox" : "none"));
    }
    else
    {
        target_state = -1;
        distance_m_ = 0.0f;
        distance_source_ = 0;
        distance_valid_ = false;
        publishTrackState(-1, 0.0f, 0.0f, primary_id, 0.0f, 0, false);
    }

    drawTracks(img);
}

void Safina_YoloUltra::computeOffsets(const cv::Rect &box)
{
    const float center_x = box.x + box.width * 0.5f;
    // Distance from the image bottom to the feet. This is the same heuristic
    // the original demo used; it is not a metric distance.
    const float bottom = static_cast<float>(img_rows - (box.y + box.height));
    const float y_px = static_cast<float>(img_cols) * 0.5f - center_x;
    const float radial = std::sqrt(y_px * y_px + bottom * bottom);

    float x_norm = (radial - standoff_deadband_px_) / standoff_full_scale_px_;
    x_norm = std::max(-1.0f, std::min(1.0f, x_norm));

    x_offset = x_norm;
    y_offset = y_px / (static_cast<float>(img_cols) * 0.5f);
}

bool Safina_YoloUltra::bboxDistance(const cv::Rect &box, double &distance_m)
{
    const cv::Rect b = clampBox(box, img_cols, img_rows);
    if (b.height < min_bbox_height_px_)
        return false;
    // Reject boxes cut off by the image border: the height would be wrong.
    if (b.y <= 4 || b.y + b.height >= img_rows - 4)
        return false;

    const double d = static_cast<double>(camera_fy) * person_height_m_ /
                     static_cast<double>(b.height);
    if (d < depth_min_range_m_ || d > depth_max_range_m_)
        return false;
    distance_m = d;
    return true;
}

bool Safina_YoloUltra::depthDistance(const cv::Rect &box, double &distance_m)
{
    if (!depth_enabled_ || last_depth_.empty() || !last_depth_msg_)
        return false;
    const rclcpp::Time depth_stamp(last_depth_msg_->header.stamp);
    if (!fresh(depth_stamp, last_depth_time_, depth_max_age_s_) ||
        std::abs((color_stamp_-depth_stamp).seconds())>depth_max_age_s_)
        return false;
    if (last_depth_.type() != CV_16UC1 || last_depth_.cols < 2 || last_depth_.rows < 2)
        return false;
    if (depth_R_.size() != 9 || depth_t_.size() != 3)
        return false;

    const cv::Rect b = clampBox(box, img_cols, img_rows);
    if (b.width < 2 || b.height < 2)
        return false;

    // Sample the torso region, not the whole box, to avoid background pixels.
    const double x0 = b.x + depth_roi_x_min_ * b.width;
    const double x1 = b.x + depth_roi_x_max_ * b.width;
    const double y0 = b.y + depth_roi_y_min_ * b.height;
    const double y1 = b.y + depth_roi_y_max_ * b.height;

    const int n = depth_grid_;
    std::vector<double> samples;
    samples.reserve(n * n);

    for (int iy = 0; iy < n; ++iy)
    {
        const double v = (n == 1) ? (y0 + y1) * 0.5
                                  : y0 + (y1 - y0) * static_cast<double>(iy) / (n - 1);
        for (int ix = 0; ix < n; ++ix)
        {
            const double u = (n == 1) ? (x0 + x1) * 0.5
                                      : x0 + (x1 - x0) * static_cast<double>(ix) / (n - 1);

            // Extrinsics map depth to color: Pc = R * Pd + t (meters).
            // Iterate the inverse projection to include the stereo baseline.
            const double dx = (u - camera_cx) / camera_fx;
            const double dy = (v - camera_cy) / camera_fy;
            double zc = std::max(1.0, depth_min_range_m_);
            bool valid = true;
            double projected_u=0, projected_v=0;
            for (int iteration=0; iteration<5; ++iteration) {
                const double pc[3]={dx*zc-depth_t_[0],dy*zc-depth_t_[1],zc-depth_t_[2]};
                double pd[3]={0,0,0};
                for (int j=0;j<3;++j)
                    for (int k=0;k<3;++k) pd[j]+=depth_R_[k*3+j]*pc[k];
                if (pd[2]<=0) { valid=false; break; }
                const int ud=static_cast<int>(std::lround(depth_fx_*pd[0]/pd[2]+depth_cx_));
                const int vd=static_cast<int>(std::lround(depth_fy_*pd[1]/pd[2]+depth_cy_));
                if (ud<0 || vd<0 || ud>=last_depth_.cols || vd>=last_depth_.rows) { valid=false; break; }
                const double zd=last_depth_.at<uint16_t>(vd,ud)*0.001;
                if (zd<depth_min_range_m_ || zd>depth_max_range_m_) { valid=false; break; }
                const double xd=(ud-depth_cx_)/depth_fx_*zd;
                const double yd=(vd-depth_cy_)/depth_fy_*zd;
                const double xc=depth_R_[0]*xd+depth_R_[1]*yd+depth_R_[2]*zd+depth_t_[0];
                const double yc=depth_R_[3]*xd+depth_R_[4]*yd+depth_R_[5]*zd+depth_t_[1];
                zc=depth_R_[6]*xd+depth_R_[7]*yd+depth_R_[8]*zd+depth_t_[2];
                if (zc<=0) { valid=false; break; }
                projected_u=camera_fx*xc/zc+camera_cx;
                projected_v=camera_fy*yc/zc+camera_cy;
            }
            if (valid && std::hypot(projected_u-u,projected_v-v)<=3.0 &&
                zc>=depth_min_range_m_ && zc<=depth_max_range_m_) samples.push_back(zc);
        }
    }

    const size_t min_samples = static_cast<size_t>(
        std::max(1.0, depth_min_valid_ratio_ * n * n));
    if (samples.size() < min_samples)
        return false;

    std::sort(samples.begin(), samples.end());
    const double median = samples[samples.size() / 2];

    // Reject depth pixels that are far from the median (background/edges).
    std::vector<double> kept;
    kept.reserve(samples.size());
    for (double z : samples)
        if (std::fabs(z - median) <= depth_outlier_m_)
            kept.push_back(z);
    if (kept.size() * 2 < samples.size())
        kept.swap(samples);

    std::sort(kept.begin(), kept.end());
    const double result = kept[kept.size() / 2];
    if (result < depth_min_range_m_ || result > depth_max_range_m_)
        return false;

    distance_m = result;
    return true;
}

bool Safina_YoloUltra::computeDistance(const cv::Rect &box, double &distance_m, int &source)
{
    if (primary_id != distance_history_id_)
    {
        distance_history_.clear();
        distance_history_id_ = primary_id;
    }

    double raw = 0.0;
    if (depthDistance(box, raw))
        source = 2;  // depth
    else if (bboxDistance(box, raw))
        source = 1;  // bbox height fallback
    else
    {
        distance_m = 0.0;
        source = 0;
        return false;
    }

    if (source != distance_history_source_) distance_history_.clear();
    distance_history_source_=source;
    distance_history_.push_back(raw);
    while (static_cast<int>(distance_history_.size()) > distance_filter_frames_)
        distance_history_.pop_front();

    std::vector<double> values(distance_history_.begin(), distance_history_.end());
    std::sort(values.begin(), values.end());
    distance_m = values[values.size() / 2];
    return true;
}

void Safina_YoloUltra::drawDetections(cv::Mat &img, const std::vector<Detection> &detections)
{
    for (const auto &det : detections)
    {
        if (det.class_id != class_id_)
            continue;
        if (det.confidence < static_cast<float>(draw_confidence_))
            continue;

        cv::rectangle(img, det.box, cv::Scalar(0, 255, 0), 1);
        char text[64];
        std::snprintf(text, sizeof(text), "person %.2f", det.confidence);
        cv::putText(img, text, cv::Point(det.box.x, std::max(15, det.box.y - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
}

void Safina_YoloUltra::drawTracks(cv::Mat &img)
{
    for (const auto &track : tracker_.tracks())
    {
        if (!track.matched_this_frame)
            continue;

        const bool is_primary = (track.id == primary_id);
        const cv::Scalar color = is_primary ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 128, 0);
        const int thickness = is_primary ? 3 : 1;

        cv::rectangle(img, track.box, color, thickness);
        char text[64];
        std::snprintf(text, sizeof(text), "ID:%d %.2f", track.id, track.confidence);
        cv::putText(img, text, cv::Point(track.box.x, std::max(15, track.box.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);
    }
}

void Safina_YoloUltra::publishTrackState(int state, float x, float y, int id,
                                         float distance_m, int distance_source,
                                         bool distance_valid)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(static_cast<float>(state));
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(static_cast<float>(id));
    msg.data.push_back(distance_m);
    msg.data.push_back(static_cast<float>(distance_source));
    msg.data.push_back(distance_valid ? 1.0f : 0.0f);
    pub_track->publish(msg);

    std_msgs::msg::Int32 id_msg;
    id_msg.data = id;
    pub_target_id->publish(id_msg);
    yolo_ultra_test::msg::TrackedTarget stamped;
    stamped.header.stamp = color_stamp_;
    stamped.header.frame_id = color_frame_;
    stamped.state=state; stamped.offset_x=x; stamped.offset_y=y; stamped.target_id=id;
    stamped.distance_m=distance_m; stamped.distance_source=distance_source;
    stamped.distance_valid=distance_valid; stamped.session_active=tracking_session_active;
    stamped_pub_->publish(stamped);
}

cv::Rect Safina_YoloUltra::clampBox(const cv::Rect &box, int width, int height)
{
    const int x1 = std::max(0, std::min(box.x, width - 1));
    const int y1 = std::max(0, std::min(box.y, height - 1));
    const int x2 = std::max(x1 + 1, std::min(box.x + box.width, width));
    const int y2 = std::max(y1 + 1, std::min(box.y + box.height, height));
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("Safina_yoloUltra_test");
    int result=0;
    try { Safina_YoloUltra detector(node); rclcpp::spin(node); }
    catch (const std::exception &e) { RCLCPP_ERROR(node->get_logger(), "%s",e.what()); result=1; }
    rclcpp::shutdown(); return result;
}
