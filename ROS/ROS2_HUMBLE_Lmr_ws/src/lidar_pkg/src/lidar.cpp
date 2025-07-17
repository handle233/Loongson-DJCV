#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ldlidar_driver.h"
#include "ros2_api.h"
#include <memory>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class LdLidarNode : public rclcpp::Node {
public:
    LdLidarNode() : Node("ldlidar_published") {
        declare_parameters();
        get_parameters();

        ldlidarnode_ = std::make_unique<ldlidar::LDLidarDriver>();

        RCLCPP_INFO(this->get_logger(), "LDLiDAR SDK Version: %s", ldlidarnode_->GetLidarSdkVersionNumber().c_str());
        RCLCPP_INFO(this->get_logger(), "product_name: %s, topic_name: %s, frame_id: %s", 
                    product_name_.c_str(), topic_name_.c_str(), setting_.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "server_ip: %s, server_port: %s", server_ip_.c_str(), server_port_.c_str());

        ldlidarnode_->RegisterGetTimestampFunctional(std::bind(&LdLidarNode::GetSystemTimeStamp, this));
        ldlidarnode_->EnableFilterAlgorithnmProcess(true);

        type_name_ = (product_name_ == "LDLiDAR_LD19") ? ldlidar::LDType::LD_19 : ldlidar::LDType::LD_19;

        if (!ldlidarnode_->Start(type_name_, server_ip_.c_str(), server_port_.c_str(), ldlidar::COMM_TCP_SERVER_MODE)) {
            RCLCPP_ERROR(this->get_logger(), "LDLiDAR start failed");
            rclcpp::shutdown();
            return;
        }

        if (!ldlidarnode_->WaitLidarCommConnect(10000)) {
            RCLCPP_ERROR(this->get_logger(), "LDLiDAR communication abnormal");
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&LdLidarNode::timer_callback, this));
    }

    ~LdLidarNode() {
        ldlidarnode_->Stop();
    }

private:
    void declare_parameters() {
        this->declare_parameter<std::string>("product_name", "LDLiDAR_LD19");
        this->declare_parameter<std::string>("topic_name", "scan");
        this->declare_parameter<std::string>("frame_id", "laser_frame");
        this->declare_parameter<std::string>("server_ip", "192.168.3.12");
        this->declare_parameter<std::string>("server_port", "7502");
        this->declare_parameter<bool>("laser_scan_dir", false);
        this->declare_parameter<bool>("enable_angle_crop_func", false);
        this->declare_parameter<double>("angle_crop_min", 0.0);
        this->declare_parameter<double>("angle_crop_max", 0.0);
    }

    void get_parameters() {
        this->get_parameter("product_name", product_name_);
        this->get_parameter("topic_name", topic_name_);
        this->get_parameter("frame_id", setting_.frame_id);
        this->get_parameter("server_ip", server_ip_);
        this->get_parameter("server_port", server_port_);
        this->get_parameter("laser_scan_dir", setting_.laser_scan_dir);
        this->get_parameter("enable_angle_crop_func", setting_.enable_angle_crop_func);
        this->get_parameter("angle_crop_min", setting_.angle_crop_min);
        this->get_parameter("angle_crop_max", setting_.angle_crop_max);
    }

    void timer_callback() {
        ldlidar::Points2D laser_scan_points;
        double lidar_scan_freq;

        switch (ldlidarnode_->GetLaserScanData(laser_scan_points, 5000)) {
            case ldlidar::LidarStatus::NORMAL:
                ldlidarnode_->GetLidarScanFreq(lidar_scan_freq);
                ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting_);
                break;
            case ldlidar::LidarStatus::DATA_TIME_OUT:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "LDLiDAR data timeout.");
                break;
            default:
                break;
        }
    }

    void ToLaserscanMessagePublish(ldlidar::Points2D &src, double lidar_spin_freq, LaserScanSetting &setting) {
    constexpr int fixed_beam_size = 500;  // slam_toolbox 要求固定数量
    sensor_msgs::msg::LaserScan output;
    auto now = this->now();
    static auto last = now;
    double scan_time = (now - last).seconds();
    last = now;

    float angle_min = 0.0;
    float angle_max = 2.0f * static_cast<float>(M_PI);
    float range_min = 0.02f;
    float range_max = 25.0f;
    float angle_increment = (angle_max - angle_min) / (fixed_beam_size - 1);

    output.header.stamp = now;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    output.scan_time = scan_time > 0.01 ? scan_time : (1.0 / lidar_spin_freq);
    output.time_increment = (fixed_beam_size > 1) ? (output.scan_time / (fixed_beam_size - 1)) : 0.0f;

    output.ranges.assign(fixed_beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(fixed_beam_size, std::numeric_limits<float>::quiet_NaN());

    for (const auto &point : src) {
        float range = point.distance / 1000.f;
        float intensity = point.intensity;
        float dir_angle = point.angle;

        if (point.distance == 0 && point.intensity == 0) {
            range = intensity = std::numeric_limits<float>::quiet_NaN();
        }

        if (setting.enable_angle_crop_func &&
            dir_angle >= setting.angle_crop_min &&
            dir_angle <= setting.angle_crop_max) {
            range = intensity = std::numeric_limits<float>::quiet_NaN();
        }

        float angle = ANGLE_TO_RADIAN(dir_angle);
        int index = std::round((angle - angle_min) / angle_increment);
        index = std::clamp(index, 0, fixed_beam_size - 1);

        if (setting.laser_scan_dir) {
            index = fixed_beam_size - index - 1;
        }

        if (std::isnan(output.ranges[index]) || range < output.ranges[index]) {
            output.ranges[index] = range;
        }
        output.intensities[index] = intensity;
    }

    publisher_->publish(output);
}


    uint64_t GetSystemTimeStamp() {
        auto tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
        return static_cast<uint64_t>(tp.time_since_epoch().count());
    }

    std::string product_name_;
    std::string topic_name_;
    std::string server_ip_;
    std::string server_port_;
    ldlidar::LDType type_name_;
    LaserScanSetting setting_;

    std::unique_ptr<ldlidar::LDLidarDriver> ldlidarnode_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LdLidarNode>());
    rclcpp::shutdown();
    return 0;
}
