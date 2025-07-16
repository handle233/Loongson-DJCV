
#include "zf_common_headfile.h"

#include "class_thread_wrapper.h"
#include "class_posix_pit.h"

// #include "get_encoder.h"
// #include "get_radar.h"
// #include "get_imu.h"
#include "tcp_send_ladar.h"
// #include "tcp_send_sensor.h"
#include "tcp_send_imu.h"
#include "tcp_send_encoder.h"
#include "tcp_recv_control.h"

#include "get_config.h"


#include "LCYX.hpp"
// #include "NetComm.h"
TimerFdWrapper *timer;

//LCYX
ServoController Servo;
MotorController Motor;
tcp_send_imu imu(USB_PATH);  // 创建 imu 对象
tcp_send_encoder encoder; // 创建编码器对象
//LCYX

// 定义线程回调函数，用于处理定时器到期事件
void timer_callback(void) 
{
    //LCYX
    if (g_car_control.servo_duty < 0 )
    {
        g_car_control.servo_duty = 0; // 限制舵机角度最小值为0
    }
    else if (g_car_control.servo_duty > 180)
    {
        g_car_control.servo_duty = 180; // 限制舵机角
        // 最大值为180
    }
    Servo.setAngle(g_car_control.servo_duty);  // 设置舵机角度
    //LCYX
    if (g_car_control.left_speed > 0 && g_car_control.right_speed > 0) {
        Motor.setLeftWheel(g_car_control.left_speed, true);
        Motor.setRightWheel(g_car_control.right_speed, true); // 设置电机前进，速度为g_car_control.left_speed
    } else if (g_car_control.left_speed < 0 && g_car_control.right_speed < 0) {
        Motor.setLeftWheel(-g_car_control.left_speed, false);
        Motor.setRightWheel(-g_car_control.right_speed, false); // 设置电机后
    } else if (g_car_control.left_speed == 0 ) {
        Motor.stop(); // 停止电机
    } else {
        printf("错误的速度值，无法设置电机速度！\n");
    }
    // Servo.setAngle(g_car_control.servo_angle);  // 设置舵机角度
    // if (g_car_control.forward) {
    //     Motor.move(g_car_control.speed, true); // 前进
    // } else {
    //     Motor.move(g_car_control.speed, false); // 后退
    // }
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}


void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 处理程序退出！！！
    // 这里需要关闭电机，关闭电调等。
    timer->stop();
    system_delay_ms(10);

    
    close_tcp_send_ladar();
    close_tcp_recv_control();
    imu.stop();  // 停止IMU数据发送线程
    encoder.stop(); // 停止编码器数据发送线程

    //LCYX
    Motor.stop();  // 停止电机
    Servo.setAngle(90);  // 设置舵机到中间位置
    //LCYX
}




int main(int, char**) 
{

    // 注册清理函数
     atexit(cleanup);

    // 注册SIGINT信号的处理函数
     signal(SIGINT, sigint_handler);

    // 创建周期定时器 周期10ms
    timer = new TimerFdWrapper(10, timer_callback);
    timer->start();

    // 创建并运行线程
    // 创建新线程

    // 串口接收雷达数据，通过TCP发送到ubuntu中。
    ThreadWrapper thd_1(tcp_send_ladar_thd_entry);
    // TCP接收ROS2的数据，控制车模运动和转向
    ThreadWrapper thd_2(tcp_recv_control_thd_entry);
    imu.start();  // 启动IMU数据发送线程
    encoder.start(); // 启动编码器数据发送线程
    while(1)
    {
        system_delay_ms(10);  // 延时10ms   
    }


}
