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

#include <error.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "LQ_MAP_ADDR.hpp"

using namespace std;

#define GPIO_Mode_In  1
#define GPIO_Mode_Out 0
/***********************************寄存器地址**************************************/
#define LS_GPIO_BASE_ADDR  0x16104000  // GPIO 基地址

#define LS_GPIO_OEN_OFFSET(n) (0x00 + (n) / 8 * 0x01)  // 方向寄存器：0 --> 输出；1 --> 输入
#define LS_GPIO_O_OFFSET(n)   (0x10 + (n) / 8 * 0x01)  // 输出寄存器
#define LS_GPIO_I_OFFSET(n)   (0x20 + (n) / 8 * 0x01)  // 输入寄存器
/************************************gpio类****************************************/
class HWGpio
{
public:
    HWGpio();                           // 空构造函数
    HWGpio(uint8_t gpio, uint8_t mode); // 设置引脚和模式的构造函数
    void GpioDirection(uint8_t mode);   // 设置输入输出模式
    void SetGpioValue(uint8_t value);   // 设置 GPIO 值
    bool GetGpioValue();                // 获取 GPIO 值
    ~HWGpio();                          // 析构函数
public:
    int Gpio;   // GPIO 引脚号
private:
    void *GPIO_ONE;     // 方向寄存器映射地址
    void *GPIO_O;       // 输出寄存器映射地址
    void *GPIO_I;       // 输入寄存器映射地址
    void *GPIO_REUSE;   // 复用寄存器映射地址
};
