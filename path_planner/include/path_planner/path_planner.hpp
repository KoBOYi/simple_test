#ifndef PATH_PLANNER__PATH_PLANNER_HPP_
#define PATH_PLANNER__PATH_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_planner/srv/get_distance.hpp" 

namespace path_planner {

class PathPlanner : public rclcpp::Node {
 public:
  PathPlanner();  

 private:
  double k_resolution_ = 0.01;

  void StartPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void EndPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void ComputeAndPublishPath();
  void GetDistanceSrvCallback(
      const std::shared_ptr<srv::GetDistance::Request> request,
      std::shared_ptr<srv::GetDistance::Response> response);  
  geometry_msgs::msg::PoseStamped MakePose(const geometry_msgs::msg::Point& point,
                                          const std_msgs::msg::Header& header);

  geometry_msgs::msg::Point start_point_;
  geometry_msgs::msg::Point end_point_;
  bool has_start_ = false;
  bool has_end_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr end_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<srv::GetDistance>::SharedPtr distance_srv_; 
};

}  // namespace path_planner

#endif  // PATH_PLANNER__PATH_PLANNER_HPP_
