#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Core>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <cmath>

// Diagnostic data only; never participates in planner decisions.
namespace flight_diag {
inline std::string quote(const std::string& x) {
  std::string s="\"";
  for (unsigned char c:x) {
    if(c=='"' || c=='\\') { s+='\\'; s+=c; }
    else if(c=='\n') s+="\\n";
    else if(c>=32) s+=c;
  }
  return s+'"';
}
inline std::string number(double x) {
  if (!std::isfinite(x)) return "null";
  std::ostringstream s; s << std::setprecision(17) << x; return s.str();
}
template<class T> std::string matrix(const Eigen::MatrixBase<T>& x) {
  std::ostringstream s; s << "[";
  for (int r=0;r<x.rows();++r) {
    if(r) s << ",";
    s << "[";
    for(int c=0;c<x.cols();++c) { if(c) s << ","; s << number(x(r,c)); }
    s << "]";
  }
  s << "]"; return s.str();
}
inline bool& enabled() { static bool value=true; return value; }
inline unsigned long& plan() { static unsigned long p=0; return p; }
inline rclcpp::Node::SharedPtr& owner() { static rclcpp::Node::SharedPtr n; return n; }
inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher() {
  static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p; return p;
}
inline void bind(const rclcpp::Node::SharedPtr& n) {
  owner()=n;
  publisher()=n->create_publisher<std_msgs::msg::String>("/ego_planner/diagnostics/events", 1000);
}
inline void event(const std::string& type, const std::string& fields="") {
  if (!enabled() || !owner()) return;
  static unsigned long seq=0;
  static const std::string session=std::to_string(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
  std_msgs::msg::String m;
  m.data="{\"schema\":1,\"session\":\""+session+"\",\"seq\":"+std::to_string(++seq)+
    ",\"plan_id\":"+std::to_string(plan())+",\"event\":"+quote(type)+",\"ros_time\":"+
    number(owner()->now().seconds())+",\"wall_time\":"+
    number(rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds())+fields+"}";
  publisher()->publish(m);
}
}
