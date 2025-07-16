#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
  IMUDriver() : Node("imu_driver_node"), key_(0), run_flag_(true) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 50);
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
  std::thread recv_thread_;
  std::atomic<bool> run_flag_;
  uint8_t key_ = 0;
  uint8_t buff_[11] = {0};
  float acceleration_[3]{};
  float angular_velocity_[3]{};
  float angle_degree_[3]{};

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
    buff_[key_++] = byte;

    if (buff_[0] != 0x55) {
      key_ = 0;
      return;
    }

    if (key_ < 11) return;

    if (verify_checksum(buff_)) {
      parse_frame(buff_);
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUDriver>());
  rclcpp::shutdown();
  return 0;
}
