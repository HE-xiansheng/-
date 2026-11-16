#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

// 延续旧机部署的100帧位置/RPY算术平均及分量减零点，不擅自改变坐标语义。
// 定时器只限频；每个有效测量源戳最多输出一次，停流后不重发或重新盖戳。
class VisionToMavros : public rclcpp::Node {
public:
    VisionToMavros():Node("factory_vision2mavros") {
        const auto topic=declare_parameter<std::string>("vision_pose_topic","/factory_fastlio/odometry");
        const double rate=declare_parameter<double>("publish_rate",30.0);
        max_age_=declare_parameter<double>("max_source_age",.2);
        future_=declare_parameter<double>("future_tolerance",.02);
        offset_=Eigen::Vector3d(declare_parameter<double>("body_lidar_x",0.),
                               declare_parameter<double>("body_lidar_y",0.),
                               declare_parameter<double>("body_lidar_z",0.));
        // 历史启动入口传此参数，但旧部署实际按固定100帧初始化，不使用滑窗。
        declare_parameter<int>("window_size",50);
        if(!std::isfinite(rate)||rate<=0||rate>1000||!std::isfinite(max_age_)||max_age_<=0||
           !std::isfinite(future_)||future_<0||!offset_.allFinite())
            throw std::invalid_argument("Invalid pose relay parameters");
        pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose",10);
        sub_=create_subscription<nav_msgs::msg::Odometry>(topic,rclcpp::QoS(1).best_effort(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){receive(*msg);});
        timer_=create_wall_timer(std::chrono::duration<double>(1./rate),[this]{publish();});
        RCLCPP_INFO(get_logger(),"Pose-only relay: 100 unique fresh samples for initialization, source stamps preserved");
    }
private:
    bool fresh(const rclcpp::Time &stamp)const {
        const double age=(now()-stamp).seconds();
        return stamp.nanoseconds()>0&&std::isfinite(age)&&age>=-future_&&age<=max_age_;
    }
    void receive(const nav_msgs::msg::Odometry &m) {
        const rclcpp::Time stamp(m.header.stamp);
        const auto &p=m.pose.pose.position;const auto &q=m.pose.pose.orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if(!fresh(stamp)||stamp<=source_||m.header.frame_id!="camera_init"||m.child_frame_id!="body"||
           !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(norm)||norm<.81||norm>1.21)return;
        const tf2::Quaternion quat(q.x,q.y,q.z,q.w);
        const tf2::Matrix3x3 rotation(quat);
        const tf2::Vector3 lever=rotation*tf2::Vector3(offset_.x(),offset_.y(),offset_.z());
        position_=Eigen::Vector3d(p.x-lever.x(),p.y-lever.y(),p.z-lever.z());
        rotation.getRPY(rpy_.x(),rpy_.y(),rpy_.z());
        source_=stamp;received_=std::chrono::steady_clock::now();
        if(count_<100){
            origin_+=position_;rpy_origin_+=rpy_;
            if(++count_==100){origin_/=100.;rpy_origin_/=100.;}
        }
    }
    void publish() {
        if(count_<100||source_<=published_||!fresh(source_)||
           std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>max_age_)return;
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp=source_;m.header.frame_id="map";
        const Eigen::Vector3d position=position_-origin_,rpy=rpy_-rpy_origin_;
        m.pose.position.x=position.x();m.pose.position.y=position.y();m.pose.position.z=position.z();
        tf2::Quaternion q;q.setRPY(rpy.x(),rpy.y(),rpy.z());
        m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();m.pose.orientation.w=q.w();
        pub_->publish(m);published_=source_;
    }
    double max_age_,future_;int count_=0;
    Eigen::Vector3d offset_,position_{0,0,0},rpy_{0,0,0},origin_{0,0,0},rpy_origin_{0,0,0};
    rclcpp::Time source_{0,0,RCL_ROS_TIME},published_{0,0,RCL_ROS_TIME};
    std::chrono::steady_clock::time_point received_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
    rclcpp::init(argc,argv);int code=0;
    try{rclcpp::spin(std::make_shared<VisionToMavros>());}
    catch(const std::exception &e){std::cerr<<"Pose relay failed: "<<e.what()<<std::endl;code=1;}
    if(rclcpp::ok())rclcpp::shutdown();return code;
}
