#ifndef SAFINA_ARUCO_TEST_H
#define SAFINA_ARUCO_TEST_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <utility>
#include <cmath>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>

class Safina_Aruco : public rclcpp::Node
{
public:
    Safina_Aruco();
    ~Safina_Aruco();


    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;


    cv::Ptr<cv::aruco::Dictionary> dictionary_5;
    cv::Ptr<cv::aruco::Dictionary> dictionary_ori;
    // One parameter set per family: the small ORIGINAL marker needs the wide
    // adaptive window, the large 5X5 marker does not and is much cheaper with it.
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_orig_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_five_;


    double aruco_roll=0;
    double aruco_pitch=0;
    double aruco_yaw=0;


private:
    static double steadySeconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static bool stampIsZero(const builtin_interfaces::msg::Time& stamp) {
        return stamp.sec == 0 && stamp.nanosec == 0;
    }

    std::string img_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_aruco_imgresult;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_aruco_poseresult;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_target_id_;
    bool original_enabled_=true,five_enabled_=true,prefer_original_=true,require_resolution_=true;
    int original_id_=0,five_id_=231,calibrated_width_=1920,calibrated_height_=1080;
    double original_size_=0.035,five_size_=0.252,original_offset_=0,five_offset_=0;
    int last_family_=-1,pending_family_=-1,pending_count_=0;
    cv::Rect tracking_roi_;
    double last_seen_=-1,last_full_search_=-1,last_native_search_=-1,full_search_scale_=0.75;
    double full_search_period_=0.6,full_native_period_=1.0;
    bool roi_enabled_=true;
    int roi_margin_=120;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
    int preview_width_=960,opencv_threads_=2;
    double preview_hz_=10,last_preview_=-1;
    // Colour annotation for the human viewer comes from a separate, downscaled
    // topic; detection itself always runs on mono8 and never converts to BGR.
    std::string color_topic_="/rtsp_cam/image_color";
    bool preview_color_=true;
    sensor_msgs::msg::Image::ConstSharedPtr color_msg_;
    double color_at_=-1;
    // The annotated preview is published from its own thread so a slow full
    // search cannot throttle the visible stream to the detection rate.
    struct PreviewSnapshot {
        sensor_msgs::msg::Image::ConstSharedPtr source;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        int selected=-1;
        std_msgs::msg::Header header;
    };
    std::thread preview_thread_;
    std::mutex preview_mutex_;
    std::mutex color_mutex_;
    std::condition_variable preview_cv_;
    bool preview_stopping_=false,preview_has_data_=false;
    PreviewSnapshot preview_snapshot_;
    cv::Mat last_color_canvas_;
    std::atomic<double> last_preview_ms_{0.0};
    int original_adaptive_max_=53,five_adaptive_max_=23;
    cv::Ptr<cv::CLAHE> clahe_;
    double window_start_=-1,processed_fps_=0;
    int window_frames_=0;


    geometry_msgs::msg::PoseStamped aruco_pos;

    void initializeSubscribers();
    void initializePublishers();
    inline double calculateRotationAngle(const std::vector<cv::Point2f>& corners);
    void readStream(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void readColor(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg);
    void previewLoop();

};

#endif  //SAFINA_ARUCO_TEST_H
