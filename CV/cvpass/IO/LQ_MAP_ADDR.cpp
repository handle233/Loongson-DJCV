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
#include "LQ_MAP_ADDR.hpp"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void *AddressMap(uint32_t PhyAddr, size_t size)
 * @功能说明：将物理地址映射为虚拟地址
 * @参数说明：PhyAddr  :  物理地址
 * @参数说明：size     :  映射大小
 * @函数返回：成功返回虚拟地址
 * @调用方法：void *addr = AddressMap(LS_GPIO_BASE_ADDR, PAGE_SIZE)
 * @备注说明：size 采用 .h 文件中的 PAGE_SIZE 即可
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void *AddressMap(uint32_t PhyAddr, size_t size)
{
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(mem_fd < 0){
        perror("Open /dev/mem error");
        exit(EXIT_FAILURE);
    }
    void *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PhyAddr & ~(PAGE_SIZE - 1));
    close(mem_fd);
    if(addr == MAP_FAILED){
        perror("mmap error");
        exit(EXIT_FAILURE);
    }
    return (void*)((uintptr_t)addr + (PhyAddr & (PAGE_SIZE - 1)));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void GpioReuse(uint8_t gpio)
 * @功能说明：复用 GPIO
 * @参数说明：gpio  :  想要复用的 GPIO 引脚号
 * @参数说明：mux   :  复用的模式(需填入一个二进制数)
 * @函数返回：无
 * @调用方法：GpioReuse(88, 0b11);
 * @备注说明：在输入第二个参数时，请参考《龙芯2k0300用户处理器手册》
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void GpioReuse(uint8_t gpio, int mux)
{
    void *GPIO_REUSE_REG = AddressMap(LS_GPIO_REUSE_ADDR + (gpio / 16) * LS_GPIO_REUSE_OFS, PAGE_SIZE);
    LS_writel(GPIO_REUSE_REG, (LS_readl(GPIO_REUSE_REG) & ~(0b11 << (gpio % 16 * 2))) | (mux << (gpio % 16 * 2)));
    munmap(GPIO_REUSE_REG, PAGE_SIZE);
}
