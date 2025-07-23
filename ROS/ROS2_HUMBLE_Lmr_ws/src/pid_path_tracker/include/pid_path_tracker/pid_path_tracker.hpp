#ifndef PID_PATH_TRACKER__PID_PATH_TRACKER_HPP_
#define PID_PATH_TRACKER__PID_PATH_TRACKER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "control_toolbox/pid.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pid_path_tracker
{

class PIDPathTracker : public nav2_core::Controller
{
public:
  PIDPathTracker() = default;
  ~PIDPathTracker() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void fixGlobalPlanFrameId(const std::string & default_frame_id);
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
 
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PIDPathTracker")};
  rclcpp::Clock::SharedPtr clock_;

  
  std::unique_ptr<control_toolbox::Pid> pid_;

  
  nav_msgs::msg::Path global_plan_;

  
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_subscriber_;
  
  
  double lookahead_dist_;
  double desired_linear_vel_;
  double max_angular_vel_;
  double transform_tolerance_;
  rclcpp::Time last_time_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

 
  double speed_limit_ = 100.0;
};

}  // namespace pid_path_tracker

#endif  // PID_PATH_TRACKER__PID_PATH_TRACKER_HPP_