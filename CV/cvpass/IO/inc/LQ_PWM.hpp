/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#pragma once

#include <iostream>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

using namespace std;

#define PWM0 "pwmchip0"  // SPI2_CLK  gpio64   使用通道0
#define PWM1 "pwmchip1"  // SPI2_MISO gpio65   使用通道0
#define PWM2 "pwmchip2"  // SPI2_MOSI gpio66   使用通道0
#define PWM3 "pwmchip3"  // SPI2_CS   gpio67   使用通道0

#define PWM_NORMAL   "normal"   // 正相
#define PWM_INVERSED "inversed" // 反相

// pwm设置类
class SetPWM {
public:
    SetPWM();   // 构造函数
    SetPWM(const string &mold, int cycle, int duty_cycle, const string &polarity);//输入的值是ns
    int ExportPWM(const string &mold);       // 导出通道
    int UnexportPWM();                       // 关闭通道
    int SetPeriod(int cycle);                // 设置周期
    int SetDutyCycle(int duty_cycle);        // 设置占空比
    int SetPolarity(const string &polarity); // 设置极性
    int Enable();                            // 使能
    int Disable();                           // 失能
    ~SetPWM();  // 析沟函数
public:
    int    fd;          // 文件描述符
    string Mold;        // 设备文件名
    string Channel;     // 通道
    int    Cycle;       // 周期
    int    Duty_Cycle;  // 占空比
    string Polarity;    // 极性
};
