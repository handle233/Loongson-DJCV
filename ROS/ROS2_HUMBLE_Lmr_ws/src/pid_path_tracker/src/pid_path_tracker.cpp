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

  speed_limit_ = 2.0;  // 默认不限制

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
  last_time_ = clock_->now();
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

void PIDPathTracker::fixGlobalPlanFrameId(const std::string & default_frame_id)
{
  for (auto & pose : global_plan_.poses) {
    if (pose.header.frame_id.empty()) {
      // RCLCPP_WARN(logger_, "Empty frame_id detected in global_plan, setting to '%s'", default_frame_id.c_str());
      pose.header.frame_id = default_frame_id;
    }
  }
}

geometry_msgs::msg::TwistStamped PIDPathTracker::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  fixGlobalPlanFrameId("map");
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel->header.stamp = clock_->now();

  // 如果没有路径，则停止
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Received empty plan, stopping the robot.");
    return *cmd_vel;
  }

  // 步骤 1: 变换全局路径至机器人局部坐标系 (base_link)
  nav_msgs::msg::Path transformed_plan;
  try {
    for (const auto & global_pose : global_plan_.poses) {
      // if (global_plan_.poses.empty() || global_plan_.poses[0].header.frame_id.empty()) {
      //   RCLCPP_ERROR(logger_, "Global plan is empty or has no frame_id!");
      //     return *cmd_vel;
      //   }
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf_->transform(
        global_pose, transformed_pose, costmap_ros_->getBaseFrameID(),
        tf2::durationFromSec(transform_tolerance_));
        
      transformed_plan.poses.push_back(transformed_pose);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform the global plan to the robot's frame: %s", ex.what());
    return *cmd_vel; // 返回零速指令
  }
  local_plan_pub_->publish(transformed_plan);

  // 步骤 2: 搜索下一个目标点 (lookahead point)
  geometry_msgs::msg::PoseStamped target_waypoint;
  bool target_found = false;
  double angle_error = 0;

  for (const auto & pose_local : transformed_plan.poses) {

    //std::cout<<"point: "<<pose_local.pose.position.x<<" "<<pose_local.pose.position.y<<std::endl;

    double dist = std::hypot(pose_local.pose.position.x, pose_local.pose.position.y);

    // double yaw = tf2::getYaw(pose_local.pose.orientation);
    double angle_to_target = std::atan2(pose_local.pose.position.x, -pose_local.pose.position.y);
    angle_error = angle_to_target - 3.1415926/2;

    if(abs(angle_error)>3.1415926*2./3.){
      //std::cout<<"target behind the car"<<std::endl;
      continue;
    }

    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));
    printf("pose_local.pose.position.x = %.2f\n",pose_local.pose.position.x);
    printf("pose_local.pose.position.y = %.2f\n",pose_local.pose.position.y);
    printf("dist = %.2f\n",dist);
    printf("angle_to_target = %.2f\n",angle_to_target);
    printf("angle_error = %.2f\n",angle_error);
    if (dist > lookahead_dist_) {
      target_waypoint = pose_local;
      target_found = true;
      break;
    }
  }
  if (!target_found) {
    std::cout<<"target mismatch"<<std::endl;
    return *cmd_vel; // 返回零速指令
    //target_waypoint = transformed_plan.poses.back();
  }

  // 可视化 lookahead point
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = costmap_ros_->getBaseFrameID();
  marker.header.stamp = clock_->now();
  marker.ns = "lookahead_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = target_waypoint.pose;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  lookahead_pub_->publish(marker);

  // 步骤 3: 计算控制值
  // double lateral_error = target_waypoint.pose.position.y;
  // rclcpp::Time now = clock_->now();
  // rclcpp::Duration duration = now - last_time_;
  // double dt_ = duration.seconds();
  // if (dt_ < 1e-6) {
  //    dt_ = 1e-3;
  // }
  // last_time_ = now;

  // double angular_velocity = pid_->computeCommand(lateral_error, dt_);
  double pid_kp = 0.8;
  double angular_velocity = std::clamp(pid_kp * angle_error, -max_angular_vel_, max_angular_vel_);
  printf("angular_velocity =%.2f\n",angular_velocity);
  // 限制角速度
  // angular_velocity = std::clamp(angular_velocity, -max_angular_vel_, max_angular_vel_);

  // 步骤 4: 安全性 - 碰撞检测
  double linear_velocity = std::min(desired_linear_vel_, speed_limit_);
  
  // // 简单的前向碰撞检测：模拟未来 1.0 秒的轨迹
  // double sim_time = 0.5;
  // int num_steps = 10;
  // double dt = sim_time / num_steps;
  // double current_x = 0.0, current_y = 0.0, current_theta = 0.0;

  // for (int i = 0; i < num_steps; ++i) {
  //   current_x += linear_velocity * cos(current_theta) * dt;
  //   current_y += linear_velocity * sin(current_theta) * dt;
  //   current_theta += angular_velocity * dt;

  //   unsigned int mx, my;
  //   if (!costmap_ros_->getCostmap()->worldToMap(current_x, current_y, mx, my)) {
  //     RCLCPP_WARN(logger_, "Projected path is out of costmap bounds. Stopping.");
  //     // return *cmd_vel; // 零速
  //   }

  //   unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
  //   if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
  //     RCLCPP_WARN(logger_, "Collision ahead! Stopping the robot.");
  //     // return *cmd_vel; // 零速
  //   }
  // }

  // 步骤 5: 发布指令
  cmd_vel->twist.linear.x = linear_velocity;
  cmd_vel->twist.angular.z = angular_velocity;
  RCLCPP_INFO(logger_, "Velocity: linear=%.2f, angular=%.2f", cmd_vel->twist.linear.x, cmd_vel->twist.angular.z);

  return *cmd_vel;
}


}  // namespace pid_path_tracker

PLUGINLIB_EXPORT_CLASS(pid_path_tracker::PIDPathTracker, nav2_core::Controller)
