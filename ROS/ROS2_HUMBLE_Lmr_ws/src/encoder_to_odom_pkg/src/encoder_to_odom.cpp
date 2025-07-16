#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

#include "net/NetComm.h"

#define PORT 7774

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher"), x_(0.0), y_(0.0), theta_(0.0)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&OdomPublisher::imuCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 初始化 TCP 监听
        if (net_.serv(PORT)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start TCP server");
            return;
        }

        net_.dispach(1); // 开启线程

        // 每 10ms 更新一次
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OdomPublisher::updateOdom, this));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, theta_);
    }

//     void update_car_state(CarState *state, int16_t leftEncoderCount, int16_t rightEncoderCount, float gyro_z) 
// {
//     double delta_time_ = SAMP_DT;
//     double delta_right = rightEncoderCount * delta_time_;
//     double delta_left  = leftEncoderCount  * delta_time_;

//     double delta_dis = (delta_right + delta_left) * ENC_TO_DIS;
//     // double v_dis = delta_dis / delta_time_;

//     double delta_theta = gyro_z;//(delta_right - delta_left) * ENC_TO_ANGLE;
//     // double v_theta = delta_theta / delta_time_;
//     double delta_x, delta_y;


//     // 小于0.02弧度认为没有转向
//     if (std::fabs(delta_theta) < 0.02)
//     {
//         delta_x = delta_dis;
//         delta_y = 0.0;
//     }
//     else
//     {
//         delta_x = delta_dis * (sin (delta_theta) / delta_theta);
//         delta_y = delta_dis * ( (1 - cos (delta_theta) ) / delta_theta);
//     }

//     state->x += (cos (state->theta) * delta_x - sin (state->theta) * delta_y);
//     state->y += (sin (state->theta) * delta_x + cos (state->theta) * delta_y);
//     state->theta += delta_theta;

//     state->linear_x = (cos (state->theta) * delta_x - sin (state->theta) * delta_y) / SAMP_DT;
//     state->linear_y = (sin (state->theta) * delta_x + cos (state->theta) * delta_y) / SAMP_DT;

// }

    void updateOdom()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 从网络接收 delta_s（double类型，单位米）
        int ret = net_.recv(&delta_s_, sizeof(delta_s_));
        if (ret != sizeof(delta_s_)) {
            RCLCPP_WARN(this->get_logger(), "Incomplete distance data received");
            return;
        }

        x_ += delta_s_ * cos(theta_);
        y_ += delta_s_ * sin(theta_);

        auto now = this->get_clock()->now();

        // 发布 odometry 消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_msg.twist.twist.linear.x = delta_s_ / 0.01;  // v = s / t (t = 0.01s)
        odom_msg.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom_msg);

        // 发布 TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_tf);

        // 重置
        delta_s_ = 0.0;
    }

    // 成员变量
    NetComm net_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    double x_, y_, theta_;
    double roll_, pitch_;
    double delta_s_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
