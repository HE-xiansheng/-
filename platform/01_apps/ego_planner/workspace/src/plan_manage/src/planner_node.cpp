#include <plan_manage/replan_fsm.h>
#include <traj_utils/diagnostics.hpp>
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");
  int result = 0;
  try {
    ego_planner::EGOReplanFSM fsm;
    fsm.init(node);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what()); result = 1;
  }
  flight_diag::publisher().reset(); flight_diag::owner().reset();
  rclcpp::shutdown(); return result;
}
