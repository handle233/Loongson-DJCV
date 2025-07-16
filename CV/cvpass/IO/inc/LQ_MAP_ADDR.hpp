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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define LS_GPIO_REUSE_ADDR 0x16000490   // GPIO复用配置寄存器基地址
#define PAGE_SIZE          0x10000      // 页大小

#define LS_GPIO_REUSE_OFS  0x04   // 复用配置寄存器偏移量

/************************************宏函数****************************************/
#define BIT(n)               (1 << (n))
#define LS_readl(addr)       (*(volatile uint32_t *)(addr))         // 读寄存器值
#define LS_writel(addr, val) (*(volatile uint32_t *)(addr) = (val)) // 写寄存器值

// 将物理地址映射为虚拟地址
void *AddressMap(uint32_t PA, size_t size);

// 复用 GPIO
void GpioReuse(uint8_t gpio, int mux);
