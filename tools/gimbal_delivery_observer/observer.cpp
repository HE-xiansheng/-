// 只记录接收元数据和稀疏像素摘要，不写入图像或发布控制消息。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <fstream>
#include <memory>
#include <vector>
#include <thread>
#include <filesystem>
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("gimbal_delivery_observer");
    std::ofstream out(node->declare_parameter<std::string>("output","/tmp/gimbal-live-ros2-check/native-headers.csv"));
    if (!out) return 2;
    out<<"qos,key,stamp_ns,receipt_ns,age_ns,width,height,step,bytes,encoding,frame,hash\n";
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions;
    for (const bool reliable : {true,false}) {
    for (const auto& key: {std::string("raw"),std::string("color")}) {
        subscriptions.push_back(node->create_subscription<sensor_msgs::msg::Image>("/rtsp_cam/image_"+key,reliable ? rclcpp::QoS(30).reliable() : rclcpp::QoS(30).best_effort(),
          [&,key,reliable](sensor_msgs::msg::Image::ConstSharedPtr message) {
            auto stamp=rclcpp::Time(message->header.stamp).nanoseconds();
            auto receipt=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            uint64_t hash=1469598103934665603ull;
            for(size_t i=0;i<message->data.size();i+=4096) hash=(hash^message->data[i])*1099511628211ull;
            out<<(reliable ? "reliable" : "best_effort")<<','<<key<<','<<stamp<<','<<receipt<<','<<(node->now().nanoseconds()-stamp)<<','<<message->width<<','<<message->height<<','<<message->step<<','<<message->data.size()<<','<<message->encoding<<','<<message->header.frame_id<<','<<hash<<'\n';
          }));
    }
    }
    const auto stop_file=node->declare_parameter<std::string>("stop_file", "");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && (stop_file.empty() || !std::filesystem::exists(stop_file))) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    executor.remove_node(node);
    subscriptions.clear();
    out.close();rclcpp::shutdown();return 0;
}
