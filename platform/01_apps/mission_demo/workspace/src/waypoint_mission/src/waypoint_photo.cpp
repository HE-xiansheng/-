// Save the latest decoded gimbal-camera frame when the mission node asks.
//
// It subscribes to the image topic published by rtsp_test/rtsp_hw_pub_ultra
// (/rtsp_cam/image_raw, 1920x1080, hardware decoded on RK3588). Trigger payload
// is a filename base without extension; the JPEG is written to base + ".jpg".
// A result string is published on /waypoint_mission/photo_done:
//   "OK <path>" or "FAIL <reason>".
//
// This node never publishes flight setpoints, services, or gimbal commands.

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>

class PhotoNode {
public:
    explicit PhotoNode(rclcpp::Node::SharedPtr node) : node_(node) {
        image_topic_ = node_->declare_parameter<std::string>("image_topic", "/rtsp_cam/image_color");
        trigger_topic_ = node_->declare_parameter<std::string>("trigger_topic", "/waypoint_mission/photo_trigger");
        done_topic_ = node_->declare_parameter<std::string>("done_topic", "/waypoint_mission/photo_done");
        freshness_ = node_->declare_parameter<double>("frame_timeout", 0.5);
        if (!std::isfinite(freshness_) || freshness_ <= 0) throw std::invalid_argument("frame_timeout must be positive");
        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(image_topic_, rclcpp::QoS(2), [this](sensor_msgs::msg::Image::ConstSharedPtr m) { image_cb(m); });
        trigger_sub_ = node_->create_subscription<std_msgs::msg::String>(trigger_topic_, 10, [this](std_msgs::msg::String::ConstSharedPtr m) { trigger_cb(m); });
        done_pub_ = node_->create_publisher<std_msgs::msg::String>(done_topic_, 10);
    }
    static double steady_now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = msg;
        latest_at_ = steady_now();
        ++frames_;
    }

    void trigger_cb(const std_msgs::msg::String::ConstSharedPtr& msg) {
        const std::string base = msg->data;
        sensor_msgs::msg::Image::ConstSharedPtr img;
        double at;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            img = latest_;
            at = latest_at_;
        }
        std_msgs::msg::String out;
        if (!img) {
            out.data = "FAIL " + base + " no_image_topic_or_no_frame";
            done_pub_->publish(out);
            RCLCPP_WARN(node_->get_logger(), "photo trigger rejected: no frame yet");
            return;
        }
        const double age = steady_now() - at;
        const double source = double(img->header.stamp.sec) + double(img->header.stamp.nanosec)*1e-9;
        const double source_age = node_->now().seconds() - source;
        if (img->header.stamp.sec < 0 || img->header.stamp.nanosec >= 1000000000u || age > freshness_ || source <= 0 || source_age < -0.1 || source_age > freshness_) {
            out.data = "FAIL " + base + " stale_or_future_frame";
            done_pub_->publish(out); return;
        }

        const std::filesystem::path path(base + ".jpg");
        std::error_code ec;
        if (!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path(), ec);

        bool ok = false;
        try {
            cv_bridge::CvImageConstPtr cv = cv_bridge::toCvShare(img, "bgr8");
            std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            ok = cv::imwrite(path.string(), cv->image, params);
        } catch (const std::exception& e) {
            out.data = std::string("FAIL ") + base + " encode " + e.what();
            done_pub_->publish(out);
            RCLCPP_ERROR(node_->get_logger(), "photo encode failed: %s", e.what());
            return;
        }

        if (ok) {
            out.data = "OK " + path.string();
            RCLCPP_INFO(node_->get_logger(), "photo saved: %s (frame age %.2fs, %ux%u)", path.string().c_str(), age,
                     img->width, img->height);
        } else {
            out.data = "FAIL " + base + " write " + path.string();
            RCLCPP_ERROR(node_->get_logger(), "photo write failed: %s", path.string().c_str());
        }
        done_pub_->publish(out);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_;
    double freshness_ = 0.5;
    std::string image_topic_, trigger_topic_, done_topic_;
    int jpeg_quality_ = 95;

    std::mutex mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
    double latest_at_ = 0;
    std::uint64_t frames_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("waypoint_photo");
    PhotoNode photo(node);
    rclcpp::spin(node);
    rclcpp::shutdown(); return 0;
}
