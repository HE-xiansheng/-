#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
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
inline void event(const std::string& type, const std::string& fields="") {
  if (!enabled()) return;
  static ros::NodeHandle nh;
  static ros::Publisher pub=nh.advertise<std_msgs::String>("/ego_planner/diagnostics/events",1000);
  static unsigned long seq=0;
  static const std::string session=std::to_string(ros::WallTime::now().toNSec());
  static std::ofstream file([]() { const char* p=std::getenv("EGO_PLANNER_RECORD_DIR");
    return p ? std::string(p)+"/optimizer.jsonl" : std::string(); }(), std::ios::app);
  std_msgs::String m;
  m.data="{\"schema\":1,\"session\":\""+session+"\",\"seq\":"+std::to_string(++seq)+
    ",\"plan_id\":"+std::to_string(plan())+",\"event\":\""+type+"\",\"ros_time\":"+
    number(ros::Time::now().toSec())+",\"wall_time\":"+number(ros::WallTime::now().toSec())+fields+"}";
  pub.publish(m);
  if(file.is_open()) { file << m.data << '\n'; file.flush();
    if(!file.good()) ROS_ERROR_THROTTLE(5,"Diagnostic journal write failed"); }
}
}
