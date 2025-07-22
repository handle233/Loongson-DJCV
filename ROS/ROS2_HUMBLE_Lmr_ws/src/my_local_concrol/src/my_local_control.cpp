#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace my_local_control
{

class MyLocalController : public nav2_core::Controller
{
public:
  MyLocalController() = default;
  ~MyLocalController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>  tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = std::move(name);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    RCLCPP_INFO(node_->get_logger(), "[%s] Configured", name_.c_str());
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_link";

    auto target = current_pose.pose;

    double dx = target.position.x - current_pose.pose.position.x;
    double dy = target.position.y - current_pose.pose.position.y;
    double distance = std::hypot(dx, dy);

    double yaw = tf2::getYaw(current_pose.pose.orientation);
    double angle_to_target = std::atan2(dy, dx);
    double angle_error = angle_to_target - yaw;

    cmd_vel.twist.linear.x = std::min(0.5, distance);
    cmd_vel.twist.angular.z = std::clamp(angle_error, -1.0, 1.0);

    return cmd_vel;
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    global_plan_ = path;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    (void)speed_limit;
    (void)percentage;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace my_local_control

PLUGINLIB_EXPORT_CLASS(my_local_control::MyLocalController, nav2_core::Controller)
