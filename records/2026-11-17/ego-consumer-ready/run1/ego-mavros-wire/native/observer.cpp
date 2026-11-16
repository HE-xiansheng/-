// 诊断用独立观察器：相同进程/执行器中比较RELIABLE与SensorDataQoS。
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/mavlink.hpp>
#include <rmw/rmw.h>
#include <iostream>
#include <chrono>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_native_qos_observer");
  using Target = mavros_msgs::msg::PositionTarget;
  auto record = [](const char *kind, const Target &m) {
    auto ns = int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;
    std::cout << "{\"kind\":\"" << kind << "\",\"stamp_ns\":" << ns << "}" << std::endl;
  };
  auto best = node->create_subscription<Target>("/mavros/setpoint_raw/local", rclcpp::SensorDataQoS(),
      [&](Target::ConstSharedPtr m) { record("best_effort", *m); });
  auto reliable = node->create_subscription<Target>("/mavros/setpoint_raw/local", rclcpp::QoS(5),
      [&](Target::ConstSharedPtr m) { record("reliable", *m); });
  auto sink = node->create_subscription<mavros_msgs::msg::Mavlink>("/uas1/mavlink_sink",
      rclcpp::QoS(1000).best_effort().durability_volatile(),
      [](mavros_msgs::msg::Mavlink::ConstSharedPtr m) {
        if (m->msgid==84 && !m->payload64.empty()) {
          std::cout << "{\"kind\":\"sink\",\"time_boot_ms\":"
                    << uint32_t(m->payload64[0] & 0xffffffffULL) << "}" << std::endl;
        }
      });
  std::cout << "{\"kind\":\"ready\",\"rmw\":\"" << rmw_get_implementation_identifier() << "\"}" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
}
