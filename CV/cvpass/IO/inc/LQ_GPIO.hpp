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

using namespace std;

#define IN  "in"
#define OUT "out"

class SetGPIO
{
public:
    SetGPIO();                                // 构造函数
    SetGPIO(uint8_t gpio, const string &dir); // 带参构造函数
    int GpioExport(uint8_t gpio);             // 导出 gpio
    int GpioUnexport();                       // 取消导出 gpio
    int GpioDirection(const string &dir);     // 设置 gpio 工作模式
    int SetGpioValue(uint8_t val);            // 设置 gpio 电平值
    int GetGpioValue();                       // 获取 gpio 电平值
    ~SetGPIO();                               // 析构函数
public:
    int fd;       // 文件描述符
    uint8_t Gpio; // gpio口
};
