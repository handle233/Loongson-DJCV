#include "LCYX.hpp"
#include <bits/stdint-uintn.h>
#include <iostream>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <string>
#include <fstream>
#include <sys/ioctl.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <utility>
#include <cstring>
#include <cstdlib>  // for system command


/*LCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYX
 * @函数名称：check_gpio_available()
 * @功能说明：检查指定GPIO引脚的可用性并重置其状态，包括释放GPIO、PWM、I2C、SPI等功能
 * @参数说明：gpio_num - 要检查的GPIO引脚编号
 * @函数返回：bool - true表示GPIO可用且已重置成功，false表示重置失败或GPIO不可用
 * @调用方法：if(check_gpio_available(66)) { // GPIO可用 }
 * @备注说明：此函数会尝试释放指定GPIO引脚上的所有外设功能，并将其重置为普通GPIO模式
 LMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMR*/

bool check_gpio_available(int gpio_num) {
    // PWM映射关系
    const std::map<int, std::string> pwm_map = {
        {64, "pwmchip0"},  // SPI2_CLK  gpio64
        {65, "pwmchip1"},  // SPI2_MISO gpio65
        {66, "pwmchip2"},  // SPI2_MOSI gpio66
        {67, "pwmchip3"}   // SPI2_CS   gpio67
    };

    // 首先检查是否为PWM引脚
    auto pwm_it = pwm_map.find(gpio_num);
    if (pwm_it != pwm_map.end()) {
        // 如果是PWM引脚，尝试释放PWM
        std::string pwm_path = "/sys/class/pwm/" + pwm_it->second;
        std::string pwm_chan_path = pwm_path + "/pwm0";
        
        if (access(pwm_chan_path.c_str(), F_OK) == 0) {
            // PWM通道已经导出，需要先禁用
            SetPWM pwm;
            pwm.Mold = pwm_it->second;
            pwm.Channel = "0";
            
            // 禁用PWM
            pwm.Disable();
            
            // 取消导出PWM通道
            if (pwm.UnexportPWM() != 0) {
                std::cerr << "无法取消导出PWM通道" << std::endl;
                return false;
            }
            
            usleep(100); // 等待取消导出完成
        }
    }

    // 检查GPIO是否已经导出
    std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio_num);
    if (access(gpio_path.c_str(), F_OK) == 0) {
        // GPIO已导出，尝试取消导出
        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd < 0) {
            std::cerr << "无法打开GPIO unexport文件" << std::endl;
            return false;
        }
        std::string str = std::to_string(gpio_num);
        write(fd, str.c_str(), str.size());
        close(fd);
        usleep(100); // 等待取消导出完成
    }

    // 检查引脚复用功能
    std::string pinmux_path = "/sys/class/pinctrl/pinctrl-0/pins/pin" + 
                             std::to_string(gpio_num) + "/function";
    if (access(pinmux_path.c_str(), F_OK) == 0) {
        std::ofstream pinmux(pinmux_path);
        if (pinmux.is_open()) {
            pinmux << "gpio" << std::endl; // 设置为GPIO功能
            pinmux.close();
        }
    }

    // 最后验证GPIO是否可用
    return true;
}

/*LCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYX
 * @类名称：ServoController
 * @功能说明：舵机控制类，实现舵机角度控制
 * @使用说明：
 *   - 角度范围：0-180度
 *   - PWM周期：5000000ns (5ms)
 *   - 占空比范围：1350000-1550000
 LMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMR*/

ServoController::ServoController() : current_angle(90), duty(MID_DUTY) {
    // 创建GTIM PWM对象
    servopwm = new GtimPwm(SERVO_GPIO, 
                          SERVO_CHANNEL, 
                          LS_GTIM_INVERSED, 
                          SERVO_PERIOD - 1, 
                          duty);
    
    if (servopwm) {
        servopwm->Enable();  // Enable()函数无返回值，直接调用
        std::cout << "舵机已初始化到中间位置(90度)" << std::endl;
    } else {
        std::cerr << "舵机初始化失败" << std::endl;
    }
}

ServoController::~ServoController() {
    servopwm->SetDutyCycle(MID_DUTY);
    sleep(1);
    if (servopwm) {
        servopwm->Disable();
        delete servopwm;
        servopwm = nullptr;
    }
}

bool ServoController::setAngle(int angle) {
    // 参数检查
    if (angle < 0 || angle > 180) {
        std::cerr << "角度必须在0到180之间" << std::endl;
        return false;
    }

    // 计算新的占空比
    duty = MIN_DUTY + (uint32_t)(angle * (MAX_DUTY - MIN_DUTY) / 180);
    
    // 更新PWM占空比
    if (servopwm) {
        servopwm->SetDutyCycle(duty);
        current_angle = angle;
        //std::cout << "舵机已设置到" << angle << "度，占空比：" << duty << std::endl;
        return true;
    }
    std::cerr << "设置舵机角度失败" << std::endl;
    return false;
}

/*LCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYXLCYX
 * @类名称：MotorController
 * @功能说明：电机控制类，实现电机的速度和方向控制
 * @使用说明：
 *   - 速度范围：0-100（百分比）
 *   - PWM周期：5000000ns (5ms)
 *   - PWM值范围：200000-600000
 LMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMRLMR*/

MotorController::MotorController() {
    // 初始化PWM
    left_motor = new SetPWM(PWM2, PWM_PERIOD, MIN_SPEED, PWM_NORMAL);
    right_motor = new SetPWM(PWM1, PWM_PERIOD, MIN_SPEED, PWM_NORMAL);
    
    // 初始化GPIO
    left_gpio = new SetGPIO(LEFT_GPIO, OUT);
    right_gpio = new SetGPIO(RIGHT_GPIO, OUT);
    
    // 使能PWM输出
    left_motor->Enable();
    right_motor->Enable();
    
    // 默认设置为前进方向
    left_gpio->SetGpioValue(0);
    right_gpio->SetGpioValue(0);
}

MotorController::~MotorController() {
    // 停止电机
    stop();
    
    // 释放资源
    if (left_motor) {
        left_motor->Disable();
        delete left_motor;
    }
    if (right_motor) {
        right_motor->Disable();
        delete right_motor;
    }
    if (left_gpio) delete left_gpio;
    if (right_gpio) delete right_gpio;
}

bool MotorController::setSpeed(int left_speed, int right_speed) {
    // 参数检查
    if (left_speed < 0 || left_speed > 100 || 
        right_speed < 0 || right_speed > 100) {
        //std::cerr << "速度值必须在0-100之间" << std::endl;
        return false;
    }
    
    // 将0-100的速度值映射到PWM值范围
    uint32_t left_pwm = MAX_SPEED + (uint32_t)((100 - left_speed) * 
                       (MIN_SPEED - MAX_SPEED) / 100);
    uint32_t right_pwm = MAX_SPEED + (uint32_t)((100 - right_speed) * 
                        (MIN_SPEED - MAX_SPEED) / 100);
    
    // 设置PWM值
    left_motor->SetDutyCycle(left_pwm);
    right_motor->SetDutyCycle(right_pwm);
    
    //std::cout << "左轮速度: " << left_speed << "%, PWM值: " << left_pwm << std::endl;
    //std::cout << "右轮速度: " << right_speed << "%, PWM值: " << right_pwm << std::endl;
    
    return true;
}

bool MotorController::setDirection(bool forward) {
    uint8_t gpio_value = forward ? 0 : 1;
    left_gpio->SetGpioValue(gpio_value);
    right_gpio->SetGpioValue(gpio_value);
    //std::cout << "设置方向: " << (forward ? "前进" : "后退") << std::endl;
    return true;
}

bool MotorController::setLeftWheel(int speed, bool forward) {
    // 参数检查
    if (speed < 0 || speed > 100) {
        //std::cerr << "速度值必须在0-100之间" << std::endl;
        return false;
    }
    
    // 设置方向
    left_gpio->SetGpioValue(forward ? 0 : 1);
    
    // 计算PWM值
    uint32_t pwm_value = MAX_SPEED + (uint32_t)((100 - speed) * 
                        (MIN_SPEED - MAX_SPEED) / 100);
    
    // 设置PWM
    left_motor->SetDutyCycle(pwm_value);
    
    //std::cout << "左轮: " << (forward ? "前进" : "后退") 
    //          << ", 速度: " << speed << "%" << std::endl;
    return true;
}

bool MotorController::setRightWheel(int speed, bool forward) {
    if (speed < 0 || speed > 100) {
        //std::cerr << "速度值必须在0-100之间" << std::endl;
        return false;
    }
    
    right_gpio->SetGpioValue(forward ? 0 : 1);
    
    uint32_t pwm_value = MAX_SPEED + (uint32_t)((100 - speed) * 
                        (MIN_SPEED - MAX_SPEED) / 100);
    
    right_motor->SetDutyCycle(pwm_value);
    
    //std::cout << "右轮: " << (forward ? "前进" : "后退") 
    //          << ", 速度: " << speed << "%" << std::endl;
    return true;
}

void MotorController::moveForward(int speed) {
    setLeftWheel(speed, true);
    setRightWheel(speed, true);
    std::cout << "车辆前进，速度: " << speed << "%" << std::endl;
}

void MotorController::moveBackward(int speed) {
    setLeftWheel(speed, false);
    setRightWheel(speed, false);
    std::cout << "车辆后退，速度: " << speed << "%" << std::endl;
}

void MotorController::stop() {
    left_motor->SetDutyCycle(MIN_SPEED);
    right_motor->SetDutyCycle(MIN_SPEED);
    std::cout << "车辆停止" << std::endl;
}

void MotorController::move(int speed, bool forward) {
    if (speed < 0 || speed > 100) {
        std::cerr << "速度值必须在0-100之间" << std::endl;
        return;
    }
    
    setLeftWheel(speed, forward);
    setRightWheel(speed, forward);
    std::cout << "车辆" << (forward ? "前进" : "后退") << "，速度: " << speed << "%" << std::endl;
}

