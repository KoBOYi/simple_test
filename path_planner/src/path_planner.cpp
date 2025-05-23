#include "path_planner/path_planner.hpp"

namespace path_planner {

PathPlanner::PathPlanner() : Node("path_planner_node") {
  start_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/start_point", 10,
      std::bind(&PathPlanner::StartPointCallback, this, std::placeholders::_1));

  end_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/end_point", 10,
      std::bind(&PathPlanner::EndPointCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  distance_srv_ = this->create_service<srv::GetDistance>(
      "/distance",
      std::bind(&PathPlanner::GetDistanceSrvCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Path Planner Node has been started.");
}

void PathPlanner::StartPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  start_point_ = *msg;
  has_start_ = true;
  if (has_end_) {
    ComputeAndPublishPath();
  }
}

void PathPlanner::EndPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  end_point_ = *msg;
  has_end_ = true;
  if (has_start_) {
    ComputeAndPublishPath();
  }
}

// Normally. start point and end point should be set synchronously. dunno why this problem ask 
// me to separate them.


void PathPlanner::ComputeAndPublishPath() {
// TODO: maybe i'll add some time checker to make sure that start point and end point are not too old   
  
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "map";

  double dx = end_point_.x - start_point_.x;
  double dy = end_point_.y - start_point_.y;
  double dz = end_point_.z - start_point_.z;

  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (distance < k_resolution_) {
    path.poses.push_back(MakePose(start_point_, path.header));
    path.poses.push_back(MakePose(end_point_, path.header));
    path_pub_->publish(path);
    return;
  } 


  int num_points = static_cast<int>(std::floor(distance / k_resolution_)) + 1;

  for (int i = 0; i <= num_points; ++i) {
    double ratio = static_cast<double>(i) / num_points;

    geometry_msgs::msg::Point point;
    point.x = start_point_.x + dx * ratio;
    point.y = start_point_.y + dy * ratio;
    point.z = start_point_.z + dz * ratio;

    path.poses.push_back(MakePose(point, path.header));
  }
  path_pub_->publish(path);
}


void PathPlanner::GetDistanceSrvCallback(
    const std::shared_ptr<srv::GetDistance::Request> /*request*/,
    std::shared_ptr<srv::GetDistance::Response> response) {
  const double dx = end_point_.x - start_point_.x;
  const double dy = end_point_.y - start_point_.y;
  const double dz = end_point_.z - start_point_.z;
  response->distance = std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::msg::PoseStamped PathPlanner::MakePose(const geometry_msgs::msg::Point& point,
                                         const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position = point;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace path_planner
