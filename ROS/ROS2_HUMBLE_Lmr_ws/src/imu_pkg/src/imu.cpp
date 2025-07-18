#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <queue>
#define LINUX
#include "net/NetComm.h"

#define TCP_PORT 7776
#define TCP_IP "192.168.3.12"

using namespace std::chrono_literals;

class IMUDriver : public rclcpp::Node {
NetComm net;
public:
  IMUDriver() : Node("imu_driver_node"), key_(0), run_flag_(true),first_time_(true) {
  last_time_=this->now();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 50);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    net.serv(TCP_PORT);
    net.dispach(20);
    recv_thread_ = std::thread(&IMUDriver::recv_loop, this);
  }
  
  ~IMUDriver() override {
    run_flag_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    net.shutdown();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::thread recv_thread_;
  std::atomic<bool> run_flag_;
  uint8_t key_ = 0;
  // uint8_t buff_[11] = {0};
  float acceleration_[3]{};
  float angular_velocity_[3]{};
  float angle_degree_[3]{};

  typedef struct {
        uint8_t imu_data[11];
        double encoder_data;
    }data_pack;
  data_pack data;
  uint8_t buf[sizeof(data_pack)];
  

  double x_, y_, theta_,theta_integral,wz;
  // double delta_x, delta_y,linear_x,linear_y;
  double roll_, pitch_;
  // double delta_s_ = 0.0;
  bool first_time_;
  rclcpp::Time last_time_;
  const double d = 5.8;
  const double h = 8.5;


  void recv_loop() {
    uint8_t temp_buf[128];
    while (run_flag_) {
      int len = net.recv(temp_buf, sizeof(temp_buf));
      for (int i = 0; i < len; ++i) {
        parse_byte(temp_buf[i]);
      }
    }
  }

  void parse_byte(uint8_t byte) {
    buf[key_++] = byte;

    if (buf[0] != 0x55) {
      key_ = 0;
      return;
    }

    // if (key_ < 11) return;

    // static uint8_t double_buf[64] = {0};
    // double_buf[key_-11] = byte;
    // key_++;

    // printf("%X ",byte);

    if (key_ < sizeof(data_pack)) return;


    memcpy(&data,(void*)buf,sizeof(data_pack));
    //printf("%f\n",data.encoder_data);

    if (verify_checksum(data.imu_data)) {
      parse_frame(data.imu_data);
    }

    key_ = 0;
  }

  bool verify_checksum(const uint8_t *data) {
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i) {
      sum += data[i];
    }
    return (sum & 0xFF) == data[10];
  }

  void parse_frame(const uint8_t *data) {
    int16_t raw[3];
    for (int i = 0; i < 3; ++i) {
      raw[i] = (int16_t)((data[2 + i * 2 + 1] << 8) | data[2 + i * 2]);
    }

    switch (data[1]) {
      case 0x51:
        for (int i = 0; i < 3; ++i) {
          acceleration_[i] = raw[i] / 32768.0f * 16.0f * 9.8f;
        }
        break;
      case 0x52:
        for (int i = 0; i < 3; ++i) {
          angular_velocity_[i] = raw[i] / 32768.0f * 2000.0f * M_PI / 180.0f;
        }
        break;
      case 0x53:
        for (int i = 0; i < 3; ++i) {
          angle_degree_[i] = raw[i] / 32768.0f * 180.0f;
        }
        publish_imu();
        break;
    }
        updateOdom();
  }

  void publish_imu() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = acceleration_[0];
    msg.linear_acceleration.y = acceleration_[1];
    msg.linear_acceleration.z = acceleration_[2];

    msg.angular_velocity.x = angular_velocity_[0];
    msg.angular_velocity.y = angular_velocity_[1];
    msg.angular_velocity.z = angular_velocity_[2];
    wz=angular_velocity_[2];

    auto q = euler_to_quaternion(angle_degree_[0], angle_degree_[1], angle_degree_[2]);
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];

    msg.orientation_covariance[0] = 0.02;
    msg.orientation_covariance[4] = 0.02;
    msg.orientation_covariance[8] = 0.02;

    msg.angular_velocity_covariance[0] = 0.001;
    msg.angular_velocity_covariance[4] = 0.001;
    msg.angular_velocity_covariance[8] = 0.001;

    msg.linear_acceleration_covariance[0] = 0.04;
    msg.linear_acceleration_covariance[4] = 0.04;
    msg.linear_acceleration_covariance[8] = 0.04;

    imu_pub_->publish(msg);
  }

  std::vector<double> euler_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg) {
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
  }

  void updateOdom()
    {
        // std::lock_guard<std::mutex> lock(data_mutex_);

        // 从网络接收 delta_s（double类型，单位米）
        // int ret = net_.recv(&delta_s_, sizeof(delta_s_));
        // if (ret != sizeof(delta_s_)) {
        //     RCLCPP_WARN(this->get_logger(), "Incomplete distance data received");
        //     return;
        // }
        auto current_time = this->now();

        if (first_time_) {
            last_time_ = current_time;
            first_time_ = false;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        //std::cout<<"dt:"<<dt<<std::endl;
        last_time_ = current_time;
        double theta_10ms=wz*dt; 

            static double total_dist = 0;
            total_dist += data.encoder_data;
            //printf("%f\n",total_dist);

        if(theta_10ms > 0)
        {
            // delta_x = delta_s_ * (sin (theta_10ms) / theta_10ms);
            // delta_y = delta_s_ * ( (1 - cos (theta_10ms) ) / theta_10ms);
            theta_integral += theta_10ms;


            x_ +=data.encoder_data *h/(h*cos(theta_10ms)-d*sin(theta_10ms)) * cos(theta_integral);
            y_ +=data.encoder_data *h/(h*cos(theta_10ms)-d*sin(theta_10ms)) * sin(theta_integral);
        }
            else if(theta_10ms < 0)
        {
            theta_integral += theta_10ms;
            x_ +=data.encoder_data *h/(h*cos(theta_10ms)+d*sin(theta_10ms)) * cos(theta_integral);
            y_ +=data.encoder_data *h/(h*cos(theta_10ms)+d*sin(theta_10ms)) * sin(theta_integral);
        }
            else if(theta_10ms == 0)
        {
            theta_integral += theta_10ms;
            x_ +=data.encoder_data  * cos(theta_integral);
            y_ +=data.encoder_data  * sin(theta_integral);
        }

        // x_ += (cos (theta_integral) * delta_x - sin (theta_integral) * delta_y);
        // y_ += (sin (theta_integral) * delta_x + cos (theta_integral) * delta_y);
        // theta_integral += theta_10ms;
        
        // linear_x = (cos (theta_integral) * delta_x - sin (theta_integral) * delta_y) / dt;
        // linear_y = (sin (theta_integral) * delta_x + cos (theta_integral) * delta_y) / dt;

        auto now = this->get_clock()->now();

        // 发布 odometry 消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q1;
        q1.setRPY(0, 0, theta_integral);
        odom_msg.pose.pose.orientation = tf2::toMsg(q1);

        odom_msg.twist.twist.linear.x = data.encoder_data / dt;  // v = s / t (t = 0.01s)
        // odom_msg.twist.twist.angular.z = 0.0;

        // odom_msg.twist.twist.linear.x  = linear_x;
        // odom_msg.twist.twist.linear.y  = linear_y;
        odom_msg.twist.twist.angular.z = theta_integral;

        odom_pub_->publish(odom_msg);

        // 发布 TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = tf2::toMsg(q1);


        tf_broadcaster_->sendTransform(odom_tf);
    }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUDriver>());
  rclcpp::shutdown();
  return 0;
}
