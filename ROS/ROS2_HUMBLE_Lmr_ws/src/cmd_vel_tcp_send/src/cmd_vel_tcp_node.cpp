#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

// ==================== 宏定义区 ====================
#define PORT 8890                     // TCP端口号
#define PI 3.1415926f                // 圆周率
#define AKM_TURN_R_MINI 0.325f       // 最小转弯半径
#define AKM_ACLE_BASE 0.155f         // 轴距
#define AKM_WHEEL_BASE 0.120f        // 轮距
#define func_limit(x, y) ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))

// ==================== 控制结构体 ====================
typedef struct __attribute__((packed)) {
    int16_t left_speed;    // 左轮速度
    int16_t right_speed;   // 右轮速度
    int16_t servo_duty;    // 舵机占空比
    int16_t servo_angle;   // 舵机角度
    int16_t speed;         // 车速
    bool forward;          // 前进 or 后退
} car_control_typedef;


// ==================== 节点类定义 ====================
class CmdVelTCPServer : public rclcpp::Node {
public:
FILE* log = nullptr; // 日志文件指针
    CmdVelTCPServer() : Node("cmd_vel_tcp_server") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelTCPServer::cmd_vel_callback, this, std::placeholders::_1));
        init_tcp_connection();
        log = fopen("/tmp/cmd_vel_tcp_server.log", "at+");
        fprintf(log, "linear,angular,");
        fprintf(log, "akm_angle,rad,");
        fprintf(log, "servo_duty,left_speed,right_speed\n");
        fflush(log);
    }

    ~CmdVelTCPServer() {
        if (new_socket_ != -1) close(new_socket_);
        if (server_fd_ != -1) close(server_fd_);
        fclose(log);
    }

private:
    // ==================== TCP连接初始化 ====================
    void init_tcp_connection() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        server_address_.sin_family = AF_INET;
        server_address_.sin_addr.s_addr = INADDR_ANY;
        server_address_.sin_port = htons(PORT);

        if (bind(server_fd_, (struct sockaddr *)&server_address_, sizeof(server_address_)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(server_fd_);
            return;
        }

        if (listen(server_fd_, 1) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
            close(server_fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP server is listening on port %d", PORT);

        int addrlen = sizeof(client_address_);
        new_socket_ = accept(server_fd_, (struct sockaddr *)&client_address_, (socklen_t*)&addrlen);
        if (new_socket_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to accept client connection");
            close(server_fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Client connected");
    }

    // ==================== 前轮转角计算函数 ====================
    float AX_AKM_WToAngle(float vx, float vw) {
        // if (vw == 0 || vx == 0) return 0.0f;
        if (vw == 0 ) return 0.0f;
        else if( vw != 0 && vx == 0) return atan(AKM_ACLE_BASE / AKM_TURN_R_MINI); // 当vx为0时，vw不为0，返回最小转弯角度
        
        else if( vw != 0 && vx != 0)
        {
        float radius = vx / vw;
        // 限制最小转弯半径
        if (radius > 0 && radius < AKM_TURN_R_MINI) radius = AKM_TURN_R_MINI;
        if (radius < 0 && radius > -AKM_TURN_R_MINI) radius = -AKM_TURN_R_MINI;

        return atan(AKM_ACLE_BASE / radius); // 弧度制
        }
    }

    double calculate_beta(double theta) {
        double denominator = 0.155 - 0.06 * tan(theta);
        if (fabs(denominator) < 1e-10) {
            fprintf(stderr, "分母接近0，计算错误\n");
            return -1;
        }
        return atan(tan(theta) / denominator);
    }

    // ==================== /cmd_vel 回调函数 ====================
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (new_socket_ == -1) return;

        car_control_typedef car_control;

        float akm_angle = AX_AKM_WToAngle(msg->linear.x, msg->angular.z);
        float radius = AKM_ACLE_BASE / tan(akm_angle);
        float akm_angle_deg = akm_angle * 180.0f / PI;
        float akm_angle_deg_n =3.5 * akm_angle_deg;

        // ==================== 左右轮速度计算 ====================
        float temp_left_speed = msg->linear.x;
        float temp_right_speed = msg->linear.x;

        // if (akm_angle != 0.0f) {
        //     temp_left_speed  = msg->linear.x * (radius - 0.5f * AKM_WHEEL_BASE) / radius;
        //     temp_right_speed = msg->linear.x * (radius + 0.5f * AKM_WHEEL_BASE) / radius;
        // }

        //car_control.left_speed = func_limit(temp_left_speed * 100, 100);
        //car_control.right_speed = func_limit(temp_right_speed * 100, 100);
        
        if(msg->linear.x > 0)
        {
          car_control.left_speed = 38 * temp_left_speed + 7;
          car_control.right_speed = 38 * temp_right_speed + 7;
        }
        else if(msg->linear.x < 0)
        {
          car_control.left_speed = 38 * temp_left_speed - 7;
          car_control.right_speed = 38 * temp_right_speed - 7;
        }
        else
        {
          car_control.left_speed = 0;
          car_control.right_speed = 0;
        }
        car_control.left_speed = func_limit(car_control.left_speed , 100);
        car_control.right_speed = func_limit(car_control.right_speed , 100);
        // ==================== 舵机占空比映射 ====================
        car_control.servo_duty = 90 + akm_angle_deg_n;

        // ==================== LCYX的扩展：servo_angle、speed、forward ====================
        float temp_speed = fabs(msg->linear.x) * 60;
        if (temp_speed > 30) temp_speed = 30;

        if (msg->linear.x == 0.0 && msg->angular.z == 0.0) {
            car_control.servo_angle = 90;
            car_control.speed = 0;
            car_control.forward = 1;
        } else if (msg->linear.x != 0.0 && msg->angular.z == 0.0) {
            car_control.servo_angle = 90;
            car_control.speed = temp_speed;
            car_control.forward = msg->linear.x > 0;
        } else if (msg->linear.x != 0.0 && msg->angular.z != 0.0) {
            float temp_servo_angle = 90 + (msg->angular.z > 0 ? fabs(msg->angular.z) : -fabs(msg->angular.z)) * 45;
            if (temp_servo_angle > 180) temp_servo_angle = 180;
            if (temp_servo_angle < 0) temp_servo_angle = 0;
            car_control.servo_angle = temp_servo_angle;
            car_control.speed = temp_speed;
            car_control.forward = msg->linear.x > 0;
        } else {
            float temp_servo_angle = 90 + (msg->angular.z > 0 ? fabs(msg->angular.z) : -fabs(msg->angular.z));
            if (temp_servo_angle > 180) temp_servo_angle = 180;
            if (temp_servo_angle < 0) temp_servo_angle = 0;
            car_control.servo_angle = temp_servo_angle;
            car_control.speed = 0;
            car_control.forward = 0;
        }

        // ==================== 调试信息输出 ====================
        RCLCPP_INFO(this->get_logger(), "linear: x=%f, angular: z=%f", msg->linear.x, msg->angular.z);
        RCLCPP_INFO(this->get_logger(), "akm_angle: %f rad, %f deg", akm_angle, akm_angle_deg);
        RCLCPP_INFO(this->get_logger(), "servo_duty: %d, left_speed: %d, right_speed: %d",
                    car_control.servo_duty, car_control.left_speed, car_control.right_speed);

        fprintf(log, "%f,%f,", msg->linear.x, msg->angular.z);
        fprintf(log, "%f,%f,", akm_angle, akm_angle_deg);
        fprintf(log, "%d,%d,%d\n",
                car_control.servo_duty, car_control.left_speed, car_control.right_speed);
        fflush(log);
        // ==================== 发送控制数据 ====================
        uint8_t *buff = (uint8_t *)&car_control;
        ssize_t send_len = send(new_socket_, buff, sizeof(car_control_typedef), 0);

        if (send_len <= 0) {
            close(new_socket_);
            new_socket_ = -1;
            RCLCPP_INFO(this->get_logger(), "Client disconnected");
            init_tcp_connection();
        }
    }

    // ==================== 成员变量 ====================
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    int server_fd_ = -1;
    int new_socket_ = -1;
    struct sockaddr_in server_address_;
    struct sockaddr_in client_address_;
};

// ==================== 主函数 ====================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelTCPServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

