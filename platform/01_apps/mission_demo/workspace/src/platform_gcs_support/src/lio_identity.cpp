// Read-only per-message publisher identity for the Humble Python task manager.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <iomanip>
#include <sstream>
#include <sys/prctl.h>
#include <unistd.h>
#include <csignal>
int main(int argc, char** argv) {
    const auto parent=getppid();
    if (parent==1 || prctl(PR_SET_PDEATHSIG,SIGTERM)!=0 || getppid()!=parent) return 2;
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("gcs_lio_identity");
    const auto topic=node->declare_parameter<std::string>("identity_topic","/gcs/lio_identity");
    auto pub=node->create_publisher<std_msgs::msg::String>(topic,10);
    auto sub=node->create_subscription<nav_msgs::msg::Odometry>("/factory_fastlio/odometry",rclcpp::SensorDataQoS(),
        [pub](nav_msgs::msg::Odometry::ConstSharedPtr msg,const rclcpp::MessageInfo& info) {
            if (msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000u) return;
            const auto& gid=info.get_rmw_message_info().publisher_gid;
            std::ostringstream hex;hex<<std::hex<<std::setfill('0');
            for (auto byte:gid.data) hex<<std::setw(2)<<unsigned(byte);
            const auto ns=int64_t(msg->header.stamp.sec)*1000000000ll+msg->header.stamp.nanosec;
            const auto& p=msg->pose.pose.position;
            const auto& q=msg->pose.pose.orientation;
            std::ostringstream data;data<<std::setprecision(17)
                <<"{\"stamp_ns\":"<<ns<<",\"gid\":\""<<hex.str()<<"\",\"frame\":"<<std::quoted(msg->header.frame_id)
                <<",\"p\":["<<p.x<<','<<p.y<<','<<p.z<<"],\"q\":["<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"]}";
            std_msgs::msg::String out;out.data=data.str();
            pub->publish(out);
        });
    rclcpp::spin(node);rclcpp::shutdown();return 0;
}
