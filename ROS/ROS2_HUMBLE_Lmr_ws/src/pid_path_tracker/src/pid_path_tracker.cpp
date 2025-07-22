#include "pid_path_tracker/pid_path_tracker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pid_path_tracker
{

void PIDPathTracker::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  parent_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.2));
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  double kp, ki, kd;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid.kp", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".pid.kp", kp);
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid.ki", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".pid.ki", ki);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid.kd", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".pid.kd", kd);

  pid_ = std::make_unique<control_toolbox::Pid>();
  pid_->initPid(kp, ki, kd, 0.0, 0.0); 

  lookahead_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    plugin_name_ + "/lookahead_point", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/local_path", 1);
    
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  
  auto on_parameter_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      for (auto & changed_parameter : event->changed_parameters) {
        const auto & name = changed_parameter.name;
        const auto & value = changed_parameter.value;

        if (name == plugin_name_ + ".lookahead_dist") {
          lookahead_dist_ = value.double_value;
        } else if (name == plugin_name_ + ".desired_linear_vel") {
          desired_linear_vel_ = value.double_value;
        } else if (name == plugin_name_ + ".pid.kp" || name == plugin_name_ + ".pid.ki" || name == plugin_name_ + ".pid.kd") {
            auto gains = pid_->getGains();
            double current_kp = gains.p_gain_;
            double current_ki = gains.i_gain_;
            double current_kd = gains.d_gain_;

            if (name == plugin_name_ + ".pid.kp") current_kp = value.double_value;
            if (name == plugin_name_ + ".pid.ki") current_ki = value.double_value;
            if (name == plugin_name_ + ".pid.kd") current_kd = value.double_value;

            pid_->setGains(control_toolbox::Pid::Gains{current_kp, current_ki, current_kd, 0.0, 0.0});
        }
      }
    };
  
  param_subscriber_ = param_client_->on_parameter_event(on_parameter_event_callback);
}

void PIDPathTracker::activate()
{
  RCLCPP_INFO(logger_, "Activating PIDPathTracker plugin.");
  lookahead_pub_->on_activate();
  local_plan_pub_->on_activate();
}

void PIDPathTracker::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PIDPathTracker plugin.");
  lookahead_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
}

void PIDPathTracker::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PIDPathTracker plugin.");
  lookahead_pub_.reset();
  local_plan_pub_.reset();
  pid_.reset();
}

void PIDPathTracker::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    RCLCPP_WARN(logger_, "Percentage-based speed limit is not fully supported, interpreting as absolute value.");
    speed_limit_ = speed_limit;
  } else {
    speed_limit_ = speed_limit;
  }
}

void PIDPathTracker::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped PIDPathTracker::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel->header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Received empty plan, stopping the robot.");
    return *cmd_vel;
  }

  nav_msgs::msg::Path transformed_plan;
  try {
    for (const auto & global_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf_->transform(
        global_pose, transformed_pose, "map",
        tf2::durationFromSec(transform_tolerance_));
      transformed_plan.poses.push_back(transformed_pose);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform the global plan to the robot's frame: %s", ex.what());
    return *cmd_vel; 
  }
  local_plan_pub_->publish(transformed_plan);

  geometry_msgs::msg::PoseStamped target_waypoint;
  bool target_found = false;
  for (const auto & pose_local : transformed_plan.poses) {
    double dist = std::hypot(pose_local.pose.position.x, pose_local.pose.position.y);
    if (dist > lookahead_dist_) {
      target_waypoint = pose_local;
      target_found = true;
      break;
    }
  }
  if (!target_found) {
    target_waypoint = transformed_plan.poses.back();
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = costmap_ros_->getBaseFrameID();
  marker.header.stamp = clock_->now();
  marker.ns = "lookahead_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = target_waypoint.pose;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  lookahead_pub_->publish(marker);

  double lateral_error = target_waypoint.pose.position.y;
  rclcpp::Time now = clock_->now();
  rclcpp::Duration duration = now - last_time_;
  double dt_ = duration.seconds();
  if (dt_ < 1e-6) {
    dt_ = 1e-3;
  }
  last_time_ = now;
  double angular_velocity = pid_->computeCommand(lateral_error, dt_);

  angular_velocity = std::clamp(angular_velocity, -max_angular_vel_, max_angular_vel_);

  double linear_velocity = std::min(desired_linear_vel_, speed_limit_);

  double sim_time = 1.0;
  int num_steps = 10;
  double dt = sim_time / num_steps;
  double current_x = 0.0, current_y = 0.0, current_theta = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    current_x += linear_velocity * cos(current_theta) * dt;
    current_y += linear_velocity * sin(current_theta) * dt;
    current_theta += angular_velocity * dt;

    unsigned int mx, my;
    if (!costmap_ros_->getCostmap()->worldToMap(current_x, current_y, mx, my)) {
      RCLCPP_WARN(logger_, "Projected path is out of costmap bounds. Stopping.");
      return *cmd_vel; 
    }

    unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      RCLCPP_WARN(logger_, "Collision ahead! Stopping the robot.");
      return *cmd_vel; 
    }
  }

  cmd_vel->twist.linear.x = linear_velocity;
  cmd_vel->twist.angular.z = angular_velocity;

  return *cmd_vel;
}

}  // namespace pid_path_tracker

PLUGINLIB_EXPORT_CLASS(pid_path_tracker::PIDPathTracker, nav2_core::Controller)
