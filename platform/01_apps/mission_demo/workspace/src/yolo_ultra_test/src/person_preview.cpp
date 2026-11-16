#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "yolo_det_task/yoloUltra_custom.h"
#include "process/postprocess.h"
#include <cmath>
#include <cstdio>
#include <iostream>

// 地面站独立人物预览：复用现有RKNN模型和后处理，只发布保留源时间戳的标注图像。
// 不订阅飞控/遥控状态，不创建跟踪目标、飞控设定点或控制服务。
class PersonPreview : public rclcpp::Node {
public:
    PersonPreview():Node("gcs_person_preview") {
        const auto model=declare_parameter<std::string>("model_file", "/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install/yolo_ultra_test/share/yolo_ultra_test/weights/yolov11_s_det_int.rknn");
        max_age_=declare_parameter<double>("image_max_age_s",.5);
        const double threshold=declare_parameter<double>("object_threshold",.2);
        const double nms=declare_parameter<double>("nms_threshold",.25);
        draw_=declare_parameter<double>("draw_confidence",.5);
        if(!std::isfinite(max_age_)||max_age_<=0||max_age_>2||!std::isfinite(threshold)||threshold<=0||threshold>1||!std::isfinite(nms)||nms<=0||nms>1||!std::isfinite(draw_)||draw_<0||draw_>1)
            throw std::invalid_argument("Invalid preview parameters");
        cv::setNumThreads(1);
        if(engine_.LoadModel(model.c_str())!=NN_SUCCESS)throw std::runtime_error("Unable to load preview RKNN model: "+model);
        yolo::setObjectThreshold(threshold);yolo::setNmsThreshold(nms);
        pub_=create_publisher<sensor_msgs::msg::Image>("/manual_person_follow/image",rclcpp::QoS(1));
        sub_=create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",rclcpp::QoS(1),[this](sensor_msgs::msg::Image::ConstSharedPtr m){receive(m);});
        RCLCPP_INFO(get_logger(),"Person preview ready: image only, RKNN model=%s",model.c_str());
    }
private:
    bool fresh(const rclcpp::Time &stamp)const {const double age=(now()-stamp).seconds();return stamp.nanoseconds()>0&&age>=-.05&&age<=max_age_;}
    void receive(const sensor_msgs::msg::Image::ConstSharedPtr &m){
        const rclcpp::Time stamp(m->header.stamp);
        if(!fresh(stamp)||stamp<=last_||m->header.frame_id.empty()||!m->width||!m->height)return;
        last_=stamp;
        try{
            cv::Mat image=cv_bridge::toCvCopy(m,"bgr8")->image;
            std::vector<Detection> detections;
            if(engine_.Run(image,detections)!=NN_SUCCESS||!fresh(stamp))return;
            const cv::Rect bounds(0,0,image.cols,image.rows);
            for(const auto &d:detections){
                if(d.class_id!=0||!std::isfinite(d.confidence)||d.confidence<draw_)continue;
                const cv::Rect box=d.box&bounds;if(box.empty())continue;
                cv::rectangle(image,box,cv::Scalar(0,255,0),2);
                char label[48];std::snprintf(label,sizeof(label),"person %.2f",d.confidence);
                cv::putText(image,label,cv::Point(box.x,std::max(15,box.y-5)),cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(0,255,0),1,cv::LINE_AA);
            }
            if(fresh(stamp))pub_->publish(*cv_bridge::CvImage(m->header,"bgr8",image).toImageMsg());
        }catch(const cv_bridge::Exception &error){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Preview image invalid: %s",error.what());}
    }
    YoloUltraCustom engine_;
    double max_age_,draw_;
    rclcpp::Time last_{0,0,RCL_ROS_TIME};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
int main(int argc,char **argv){
    rclcpp::init(argc,argv);int code=0;
    try{rclcpp::spin(std::make_shared<PersonPreview>());}catch(const std::exception &error){std::cerr<<"Person preview failed: "<<error.what()<<std::endl;code=1;}
    if(rclcpp::ok())rclcpp::shutdown();return code;
}
