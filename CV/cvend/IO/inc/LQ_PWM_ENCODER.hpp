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

#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "LQ_HW_GPIO.hpp"
#include "LQ_MAP_ADDR.hpp"

#define NUM_ENCODER_LINE 512 // 编码器线数

/***********************************寄存器地址**************************************/
#define LS_PWM_BASE_ADDR            0x1611B000  // pwm0基地址
#define LS_PWM_OFFSET               0x10        // pwm控制器偏移量
#define LS_PWM_LOW_BUFFER_OFFSET    0x4         // 低脉冲缓冲寄存器
#define LS_PWM_FULL_BUFFER_OFFSET   0x8         // 脉冲周期缓冲寄存器
#define LS_PWM_CTRL_OFFSET          0xC         // 控制寄存器

/********************************控制寄存器各位域***********************************/
#define LS_PWM_CTRL_EN      BIT(0)  // 计数器使能   （1--计数；0--停止）
#define LS_PWM_CTRL_INTE    BIT(5)  // 中断使能     （1--使能；0--失能）
#define LS_PWM_CTRL_RST     BIT(7)  // 计数器重置   （1--重置；0--正常工作）
#define LS_PWM_CTRL_CAPTE   BIT(8)  // 测量脉冲使能 （1--测量脉冲；0--脉冲输出）

// 编码器模式
class LS_PwmEncoder
{
public:
    LS_PwmEncoder(int channel, int gpio); // 构造函数
    void Init(void);                      // 编码器功能初始化
    float Update(void);                   // 更新
    void ResetCounter(void);              // 重置
    void CloseResCounter(void);           // 关闭重置
    ~LS_PwmEncoder(void);                 // 析构函数
public:
    HWGpio DirGPIO;
private:
    uint32_t PWM_ADDR;
    void *Low_buffer;  // 低脉冲缓冲寄存器
    void *Full_buffer; // 脉冲周期缓冲寄存器
    void *CTRL;        // 控制寄存器
};
