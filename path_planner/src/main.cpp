#include "path_planner/path_planner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planner::PathPlanner>());
  rclcpp::shutdown();
  return 0;
}
