#include <ros/ros.h> // 引入 ROS C++ 接口，提供节点初始化、日志和频率控制功能。
int main(int argc, char **argv) // 程序入口，接收命令行参数，包括 roslaunch 传入的参数。
{ // main 函数开始。
    ros::init(argc, argv, "hello_cpp"); // 初始化 ROS 节点，默认节点名为 hello_cpp。
    ros::NodeHandle node; // 创建节点句柄，启动并维持本进程的 ROS 通信环境。
    ros::Rate rate(1.0); // 设置循环频率为 1 Hz，即正常情况下每秒执行一次。
    while (ros::ok()) // ROS 未关闭时持续循环；按 Ctrl+C 后通常退出循环。
    { // 循环体开始。
        ROS_INFO("[C++] hello world"); // 输出 INFO 日志，用 [C++] 标识消息来源。
        rate.sleep(); // 等待本轮剩余时间，避免循环无间隔地占用 CPU。
    } // 循环体结束。
    return 0; // 返回 0，表示程序正常结束。
} // main 函数结束，节点句柄随之释放。
