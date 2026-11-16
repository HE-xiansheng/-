#!/usr/bin/env python3
# 上一行指定 Python 3 解释器；这是特殊声明，不要在它的末尾追加中文注释。
import rospy  # 引入 ROS Python 接口，提供节点初始化、日志和频率控制功能。
def main():  # 定义节点的主要运行逻辑。
    rospy.init_node("hello_python")  # 初始化 ROS 节点，默认节点名为 hello_python。
    rate = rospy.Rate(1.0)  # 设置循环频率为 1 Hz，即正常情况下每秒执行一次。
    while not rospy.is_shutdown():  # ROS 未关闭时持续循环。
        rospy.loginfo("[Python] hello world")  # 输出 INFO 日志，用 [Python] 标识消息来源。
        rate.sleep()  # 等待本轮剩余时间，避免循环无间隔地占用 CPU。
if __name__ == "__main__":  # 仅在直接运行本脚本时执行以下内容。
    try:  # 捕获等待过程中因 ROS 关闭而产生的中断异常。
        main()  # 调用主函数，开始运行 ROS 节点。
    except rospy.ROSInterruptException:  # 处理 Ctrl+C 或 ROS 关闭引发的等待中断。
        pass  # 无须额外处理，让脚本正常结束。
