#include "ros2_api.h"
#include "ldlidar_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class LdLidarNode : public rclcpp::Node
{
public:
    LdLidarNode() : Node("ldlidar_published")
    {
        // 声明ROS 2参数
        declare_parameters();
        // 获取ROS 2参数
        get_parameters();

        ldlidarnode_ = new ldlidar::LDLidarDriver();

        RCLCPP_INFO(this->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode_->GetLidarSdkVersionNumber().c_str());
        RCLCPP_INFO(this->get_logger(), "<product_name>: %s", product_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "<topic_name>: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "<frame_id>: %s", setting_.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "<port_name>: %s", port_name_.c_str());
        // RCLCPP_INFO(this->get_logger(), "<port_baudrate>: %d", serial_port_baudrate_);

        RCLCPP_INFO(this->get_logger(), "<server_ip>: %s", server_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "<server_port>: %s", server_port_.c_str());
        
        RCLCPP_INFO(this->get_logger(), "<laser_scan_dir>: %s", (setting_.laser_scan_dir ? "Counterclockwise" : "Clockwise"));
        RCLCPP_INFO(this->get_logger(), "<enable_angle_crop_func>: %s", (setting_.enable_angle_crop_func ? "true" : "false"));
        RCLCPP_INFO(this->get_logger(), "<angle_crop_min>: %f", setting_.angle_crop_min);
        RCLCPP_INFO(this->get_logger(), "<angle_crop_max>: %f", setting_.angle_crop_max);

       if (product_name_ == "LDLiDAR_LD19")
        {
            type_name_ = ldlidar::LDType::LD_19;
        }

        ldlidarnode_->RegisterGetTimestampFunctional(std::bind(&LdLidarNode::GetSystemTimeStamp, this));
        ldlidarnode_->EnableFilterAlgorithnmProcess(true);

        // if (ldlidarnode_->Start (type_name_, port_name_, serial_port_baudrate_, ldlidar::COMM_TCP_SERVER_MODE) )
        // {
        //     RCLCPP_INFO (this->get_logger(), "ldlidar node start is success");
        // }
        // else
        // {
        //     RCLCPP_ERROR (this->get_logger(), "ldlidar node start is fail");
        //     rclcpp::shutdown();
        //     return;
        // }


        if (ldlidarnode_->Start (type_name_, server_ip_.c_str(), server_port_.c_str(), ldlidar::COMM_TCP_SERVER_MODE) )
        {
            RCLCPP_INFO (this->get_logger(), "ldlidar node start is success");
        }
        else
        {
            RCLCPP_ERROR (this->get_logger(), "ldlidar node start is fail");
            rclcpp::shutdown();
            return;
        }


        if (ldlidarnode_->WaitLidarCommConnect(10000))
        {
            RCLCPP_INFO(this->get_logger(), "ldlidar communication is normal.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "ldlidar communication is abnormal.");
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&LdLidarNode::timer_callback, this));

        // 启动TCP连接处理线程
        // tcp_thread_ = std::thread(&LdLidarNode::init_tcp_connection, this);
    }

    ~LdLidarNode()
    {
        ldlidarnode_->Stop();
        delete ldlidarnode_;
        ldlidarnode_ = nullptr;
    }

private:
    void declare_parameters()
    {
        this->declare_parameter<std::string>("product_name", product_name_);
        this->declare_parameter<std::string>("topic_name", topic_name_);
        this->declare_parameter<std::string>("frame_id", setting_.frame_id);


        this->declare_parameter<std::string>("server_ip", server_ip_);
        this->declare_parameter<std::string>("server_port", server_port_);

        this->declare_parameter<int>("port_baudrate", serial_port_baudrate_);
        this->declare_parameter<bool>("laser_scan_dir", setting_.laser_scan_dir);
        this->declare_parameter<bool>("enable_angle_crop_func", setting_.enable_angle_crop_func);
        this->declare_parameter<double>("angle_crop_min", setting_.angle_crop_min);
        this->declare_parameter<double>("angle_crop_max", setting_.angle_crop_max);
    }

    void get_parameters()
    {
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

    void timer_callback()
    {
        ldlidar::Points2D laser_scan_points;
        double lidar_scan_freq;

        switch (ldlidarnode_->GetLaserScanData(laser_scan_points, 5000))
        {
        case ldlidar::LidarStatus::NORMAL:
            ldlidarnode_->GetLidarScanFreq(lidar_scan_freq);
            ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting_);
            break;

        case ldlidar::LidarStatus::DATA_TIME_OUT:
            RCLCPP_ERROR(this->get_logger(), "get ldlidar data is time out, please check your lidar device.");
            break;

        case ldlidar::LidarStatus::DATA_WAIT:
            break;

        default:
            break;
        }
    }

    void ToLaserscanMessagePublish(ldlidar::Points2D &src, double lidar_spin_freq, LaserScanSetting &setting)
    {
        float angle_min, angle_max, range_min, range_max, angle_increment;
        double scan_time;
        rclcpp::Time start_scan_time = this->now();
        static rclcpp::Time end_scan_time;
        static bool first_scan = true;

        scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

        if (first_scan)
        {
            first_scan = false;
            end_scan_time = start_scan_time;
            return;
        }

        // Adjust the parameters according to the demand
        angle_min = 0;
        angle_max = (2 * M_PI);
        range_min = 0.02;
        range_max = 25;
        int beam_size = static_cast<int>(src.size());
        angle_increment = (angle_max - angle_min) / (float)(beam_size - 1);

        // Calculate the number of scanning points
        if (lidar_spin_freq > 0)
        {
            sensor_msgs::msg::LaserScan output;
            output.header.stamp = start_scan_time;
            output.header.frame_id = setting.frame_id;
            output.angle_min = angle_min;
            output.angle_max = angle_max;
            output.range_min = range_min;
            output.range_max = range_max;
            output.angle_increment = angle_increment;

            if (beam_size <= 1)
            {
                output.time_increment = 0;
            }
            else
            {
                output.time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
            }

            output.scan_time = scan_time;
            // First fill all the data with Nan
            output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
            output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

            for (auto point : src)
            {
                float range = point.distance / 1000.f; // distance unit transform to meters
                float intensity = point.intensity;     // laser receive intensity
                float dir_angle = point.angle;

                if ((point.distance == 0) && (point.intensity == 0)) // filter is handled to  0, Nan will be assigned variable.
                {
                    range = std::numeric_limits<float>::quiet_NaN();
                    intensity = std::numeric_limits<float>::quiet_NaN();
                }

                if (setting.enable_angle_crop_func) // Angle crop setting, Mask data within the set angle range
                {
                    if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max))
                    {
                        range = std::numeric_limits<float>::quiet_NaN();
                        intensity = std::numeric_limits<float>::quiet_NaN();
                    }
                }

                float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
                int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));

                if (index < beam_size)
                {
                    if (index < 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f",
                                     index, beam_size, angle, angle_min, angle_increment);
                    }

                    if (setting.laser_scan_dir)
                    {
                        int index_anticlockwise = beam_size - index - 1;

                        // If the current content is Nan, it is assigned directly
                        if (std::isnan(output.ranges[index_anticlockwise]))
                        {
                            output.ranges[index_anticlockwise] = range;
                        }
                        else // Otherwise, only when the distance is less than the current
                        {
                            //   value, it can be re assigned
                            if (range < output.ranges[index_anticlockwise])
                            {
                                output.ranges[index_anticlockwise] = range;
                            }
                        }

                        output.intensities[index_anticlockwise] = intensity;
                    }
                    else
                    {
                        // If the current content is Nan, it is assigned directly
                        if (std::isnan(output.ranges[index]))
                        {
                            output.ranges[index] = range;
                        }
                        else // Otherwise, only when the distance is less than the current
                        {
                            //   value, it can be re assigned
                            if (range < output.ranges[index])
                            {
                                output.ranges[index] = range;
                            }
                        }

                        output.intensities[index] = intensity;
                    }
                }
            }

            publisher_->publish(output);
            end_scan_time = start_scan_time;
        }
    }

    uint64_t GetSystemTimeStamp()
    {
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
            std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
        auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
        return ((uint64_t)tmp.count());
    }

    // // 初始化TCP连接
    // void init_tcp_connection()
    // {

    //     #define PORT 8888
    //     // 创建套接字
    //     socket_ = socket(AF_INET, SOCK_STREAM, 0);
    //     if (socket_ == -1)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
    //         return;
    //     }

    //     struct sockaddr_in server_addr;
    //     memset(&server_addr, 0, sizeof(server_addr));
    //     server_addr.sin_family = AF_INET;
    //     server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //     server_addr.sin_port = htons(PORT);

    //     // 绑定套接字到指定地址和端口
    //     if (bind(socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
    //         close(socket_);
    //         return;
    //     }

    //     // 监听连接请求
    //     if (listen(socket_, 1) == -1)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
    //         close(socket_);
    //         return;
    //     }

    //     RCLCPP_INFO(this->get_logger(), "Server is listening on port %d...", PORT);

    //     while (rclcpp::ok())
    //     {
    //         struct sockaddr_in client_addr;
    //         socklen_t client_addr_size = sizeof(client_addr);
    //         // 接受客户端连接
    //         int client_socket = accept(socket_, (struct sockaddr *)&client_addr, &client_addr_size);
    //         if (client_socket == -1)
    //         {
    //             RCLCPP_ERROR(this->get_logger(), "Failed to accept client connection");
    //             continue;
    //         }192.168.2.42

    //         RCLCPP_INFO(this->get_logger(), "Client connected");
    //         // 接收数据
    //         receive_data(client_socket);
    //         close(client_socket);
    //     }
    // }

    // 接收数据
    // void receive_data(int client_socket)
    // {
    //     #define BUF_SIZE  64
    //     uint8_t buf[BUF_SIZE] = {0};
    //     while (rclcpp::ok())
    //     {
    //         ssize_t recv_len = recv(client_socket, buf, BUF_SIZE, 0);
    //         if (recv_len <= 0)
    //         {
    //             RCLCPP_INFO(this->get_logger(), "Client disconnected");
    //             break;
    //         }
    //         // RCLCPP_INFO(this->get_logger(), "recv_len = %d", recv_len);
    //         ldlidarnode_->comm_pkg_->CommReadCallback((const char *)buf, recv_len);

    //     }
    // }

    std::string product_name_;
    std::string topic_name_;
    std::string port_name_;

    std::string server_ip_;
    std::string server_port_;

    int serial_port_baudrate_;
    ldlidar::LDType type_name_;
    LaserScanSetting setting_;
    ldlidar::LDLidarDriver *ldlidarnode_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int socket_;
    std::thread tcp_thread_; // 添加线程成员变量
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LdLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}