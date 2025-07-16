#ifndef _LCYX_HPP_
#define _LCYX_HPP_

//////////////// C++标准库 //////////////////////////////
#include <iostream>
#include <string>

//////////////// C标准库 ////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>    // 共享内存
#include <sys/sem.h>    // 信号量集
#include <fcntl.h>
#include <map>

//////////////// 龙邱库文件 /////////////////////////////
#include "LQ_PWM.hpp"
#include "LQ_GPIO.hpp"
#include "LQ_GTIM_PWM.hpp"
#include "LQ_HW_GPIO.hpp"
#include "LQ_MAP_ADDR.hpp"
#include "LQ_PWM_ENCODER.hpp"

bool check_gpio_available(int gpio_num);

class ServoController {
private:
    GtimPwm* servopwm;        // 改为 GtimPwm 类型
    int current_angle;
    uint32_t duty;
    
    // PWM参数常量
    static const uint32_t SERVO_PERIOD = 500000;    // 5ms周期
    // static const uint32_t MIN_DUTY = 135000;        // 最小占空比
    // static const uint32_t MID_DUTY = 145000;        // 中间占空比
    // static const uint32_t MAX_DUTY = 155000;        // 最大占空比
    static const uint32_t MIN_DUTY = 137000;        // 最小占空比
    static const uint32_t MID_DUTY = 147000;        // 中间占空比
    static const uint32_t MAX_DUTY = 157000;        // 最大占空比
    static const uint8_t SERVO_GPIO = 88;            // GPIO88
    static const uint8_t SERVO_CHANNEL = 2;          // GTIM通道2

public:
    ServoController();
    ~ServoController();
    bool setAngle(int angle);
    int getCurrentAngle() const { return current_angle; }
    uint32_t getCurrentDuty() const { return duty; }
};

class MotorController {
private:
    SetPWM* left_motor;    // 左轮PWM控制
    SetPWM* right_motor;   // 右轮PWM控制
    SetGPIO* left_gpio;    // 左轮方向控制
    SetGPIO* right_gpio;   // 右轮方向控制
    
    // PWM参数常量
    static const uint32_t PWM_PERIOD = 5000000;
    static const uint32_t MAX_SPEED = 2750000;   // 最高速度
    static const uint32_t MIN_SPEED = 5000000;   // 最低速度
    
    // GPIO引脚定义
    static const uint8_t LEFT_GPIO = 74;   // 左轮方向控制引脚
    static const uint8_t RIGHT_GPIO = 75;  // 右轮方向控制引脚

    // 将基本控制函数移到 private
    bool setSpeed(int left_speed, int right_speed);  // 设置两轮速度
    bool setDirection(bool forward);                 // 设置运动方向
    bool setLeftWheel(int speed, bool forward);     // 控制左轮
    bool setRightWheel(int speed, bool forward);    // 控制右轮

    void moveForward(int speed);   // 前进
    void moveBackward(int speed);  // 后退

public:
    MotorController();
    ~MotorController();
    
    // 直线运动控制
    void move(int speed, bool forward);  // 移动：speed为速度(0-100)，forward为true表示前进，false表示后退
    void stop();                         // 停止
};


#endif // _LCYX_HPP_