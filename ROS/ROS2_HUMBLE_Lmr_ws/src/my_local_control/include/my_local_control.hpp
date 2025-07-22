#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

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
  ;

  void cleanup() override ;
  void activate() override ;
  void deactivate() override ;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override
  ;

  void setPlan(const nav_msgs::msg::Path & path) override
  ;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  ;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace my_local_control

