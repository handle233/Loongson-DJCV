#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"
#include "my_local_control.hpp"

namespace my_local_control
{

void MyLocalController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>  tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = std::move(name);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    RCLCPP_INFO(node_->get_logger(), "[%s] Configured", name_.c_str());
  }

  void MyLocalController::cleanup() {}
  void MyLocalController::activate() {}
  void MyLocalController::deactivate() {}

  geometry_msgs::msg::TwistStamped MyLocalController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_link";

    auto target = global_plan_.poses[0].pose;
using namespace std;
    cout<<"num of poses "<<global_plan_.poses.size()<<" "<<endl;
    cout<<"my pos "<<current_pose.pose.position.x<<" "<<current_pose.pose.position.y
    <<" "<<current_pose.pose.orientation.z<<endl;

    for(auto &i : global_plan_.poses){
      cout<<"x "<<i.pose.position.x<<" y "<<i.pose.position.y;
      cout<<" z "<<i.pose.orientation.z<<endl;
    }


    double dx = target.position.x - current_pose.pose.position.x;
    double dy = target.position.y - current_pose.pose.position.y;
    double distance = std::hypot(dx, dy);

    double yaw = tf2::getYaw(current_pose.pose.orientation);
    double angle_to_target = std::atan2(dy, dx);
    double angle_error = angle_to_target - yaw;

    cmd_vel.twist.linear.x = 0 ;//std::max(0.11, distance);
    cmd_vel.twist.angular.z = 0;//std::clamp(angle_error, -1.0, 1.0)*0.1;

    return cmd_vel;
  }

  

  void MyLocalController::setPlan(const nav_msgs::msg::Path & path)
  {
    global_plan_ = path;
  }

  void MyLocalController::setSpeedLimit(const double & speed_limit, const bool & percentage)
  {
    (void)speed_limit;
    (void)percentage;
  }

}  // namespace my_local_control

PLUGINLIB_EXPORT_CLASS(my_local_control::MyLocalController, nav2_core::Controller)
