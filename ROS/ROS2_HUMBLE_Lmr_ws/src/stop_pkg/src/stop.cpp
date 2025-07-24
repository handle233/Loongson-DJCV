#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <memory>
#include <cstdlib>

class Nav2StatusMonitor : public rclcpp::Node {
public:
  Nav2StatusMonitor() : Node("nav2_status_monitor") {
    using std::placeholders::_1;
    status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10,
      std::bind(&Nav2StatusMonitor::status_callback, this, _1));
  }

private:
  void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    if (msg->status_list.empty()) {
      RCLCPP_INFO(this->get_logger(), "��ǰû�л�Ծ��Ŀ��");
      return;
    }

    // ��ȡ���һ��Ŀ���״̬
    const auto & latest_status = msg->status_list.back();

    switch (latest_status.status) {
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING:{
        RCLCPP_INFO(this->get_logger(), "导航中...");
        break;
      }
        
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:{
        RCLCPP_INFO(this->get_logger(), "导航成功！");
        int ret = system("pkill -f lidar_pkg"); // system("pkill -f xxx")
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Node successfully shut down");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to shut down node");
        }
        break;
      }
        
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:{
        RCLCPP_WARN(this->get_logger(), "导航失败！");
        break;
      }
        
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:{
        RCLCPP_WARN(this->get_logger(), "导航被取消！");
        break;
      }
        
      default:
        RCLCPP_INFO(this->get_logger(), "当前状态码: %d", latest_status.status);
        break;
    }
  }

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2StatusMonitor>());
  rclcpp::shutdown();
  return 0;
}