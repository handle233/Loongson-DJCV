#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <unistd.h>

#include <iostream>
#include <thread>
#include "zf_common_headfile.h"
#include "get_radar.h"
#include <queue>
#include "radar_data_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include <stdio.h>      // 标准输入输出
#include <stdlib.h>     // 标准库
#include <mqueue.h>     // 消息队列
#include <string.h>     // 字符串操作
#include <sys/stat.h>   // 文件状态
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include "get_config.h"

// 定义全局变量，存储串口文件描述符
static int serial_fd = 0;
#define BUFFER_SIZE 4096  // 设置缓冲区大小为4096字节

// 打开串口
int open_serial_port(const char *port) 
{
    int fd = open(port, O_RDWR | O_NOCTTY);
    // struct termios options;
    if (fd == -1) 
    {
        perror("open_port: Unable to open serial port");

        return -1;
    }
    // // 获取当前串口属性
    // if (tcgetattr(fd, &options) != 0) {
    //     perror("open_port: Unable to get attributes - ");
    //     close(fd);
    //     return -1;
    // }

    // // 设置缓冲区大小（部分系统可能不支持）
    // if (ioctl(fd, F_SETPIPE_SZ, BUFFER_SIZE) == -1) {
    //     perror("open_port: Unable to set buffer size - ");
    // }
    return fd;
}

// 设置串口属性
int set_serial_config(int fd, speed_t baudrate) 
{
    struct termios tty;
    // 清空结构体
    memset(&tty, 0, sizeof(tty)); 

    // 获取当前串口属性
    if (tcgetattr(fd, &tty) != 0) { 
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    // 设置输入波特率
    cfsetispeed(&tty, baudrate); 
    // 设置输出波特率
    cfsetospeed(&tty, baudrate); 

    // 禁用奇偶校验
    tty.c_cflag &= ~PARENB; 
    // 设置停止位为1
    tty.c_cflag &= ~CSTOPB; 
    // 清除数据位设置
    tty.c_cflag &= ~CSIZE; 
    // 设置数据位为8位
    tty.c_cflag |= CS8; 
    // 禁用硬件流控
    tty.c_cflag &= ~CRTSCTS; 
    // 启用接收器，本地连接
    tty.c_cflag |= CREAD | CLOCAL; 

    // 设置输入模式：非规范模式，不回显，不产生信号
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    // 禁用软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
    // 设置输出模式
    tty.c_oflag &= ~OPOST; 

    // 禁用输入模式中的回车转换为换行
    tty.c_iflag &= ~(ICRNL);
    // 禁用输出模式中的换行转换为回车换行
    tty.c_oflag &= ~(ONLCR);

    // //清bit位 关闭字符映射 0x0a 0x0d
    // tty.c_iflag &= ~(INLCR|ICRNL);
    // //清bit位 关闭流控字符 0x11 0x13
    // tty.c_iflag &= ~(IXON);

    // 至少读取1个字符
    tty.c_cc[VMIN] = 1; 
    // 无超时
    tty.c_cc[VTIME] = 0; 

    // 应用新的串口属性
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { 
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    // 刷新输入缓冲区
    tcflush(fd, TCIFLUSH); 

    return 0;
}

// 向串口写入数据
int write_to_serial(int fd, const unsigned char *data, int length) {
    int written = write(fd, data, length);
    if (written == -1) {
        perror("write");
        return -1;
    }
    return written;
}

// 从串口读取数据
int32 read_from_serial(int fd, unsigned char *buffer, int length) 
{
    int32 bytes_read = read(fd, buffer, length);
    if (bytes_read == -1)
    {
        perror("read");
        return -1;
    }
    return bytes_read;
}


vuint32 temp_sum;
// int init_serial_port("/dev/ttyS1", B230400);

int init_serial_port(const char* serial_port, uint16 baud)
{

    int fd = open_serial_port(serial_port);
    if (fd == -1) {
        return -1;
    }

    if (set_serial_config(fd, baud) != 0) 
    {  
        // 设置波特率为baud，可按需调整
        close(fd);
        return -1;
    }
    
    return fd;
}


static std::queue<uint8_t> uart_que;
// static LiDARFrameTypeDef radar_strct;

// 定义消息队列的键值，可自定义
#define MSG_KEY 1234

uint8 uart_arr[47];

void get_ladar_data(uint8 uart_data)
{
    static uint8 index = 0;

    if(index == 0 && uart_data == 0x54)
    {
        uart_arr[index] = uart_data;
        index = 1;
    }
    else if(index == 1 && uart_data == 0x2C)
    {
        uart_arr[index] = uart_data;
        index = 2;
    }
    else if(index >= 2)
    {
        // if(index == 3 && uart_data == 0x0A)
        // {
        //     uart_data = 0x0D;
        // }

        uart_arr[index] = uart_data;
        index++;

    

        if(index >= 47)
        {
            index = 0;
            if((uint8)CalCRC8(uart_arr, 46) == (uint8)uart_arr[46])
            {
                printf("O\n");
            }
            else
            {
                printf("E\n");
            }
        }
    }

    printf("%02X ", uart_data);
}

void get_radar_thd_entry() 
{
    // std::cout << "get_radar_thd_entry ID: " << std::this_thread::get_id() << std::endl;


    // // 尝试设置新的高优先级,值越大优先级越高（1-99）
	// struct sched_param param;
	// param.sched_priority = RADAR_THD_PRO;
	// pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);


    // int ret = 0;

    // // 打开串口
    // ret = get_radar_init(UART_PATH, UART_BAUD);
    // if(ret < 0)
    // {
    //     printf("uart_recv_thd_init error \r\n");
    //     exit(0);
    // }

    // // 接收数据的缓冲区
    // uint8_t receive_buffer[512];
    // int32 received_bytes;

    // uint8 *radar_data = (uint8 *)&radar_strct;


    // int msgid;

    // struct message msg;

    // 创建或获取消息队列
    // msgget 函数用于创建或访问消息队列
    // MSG_KEY 是消息队列的键值
    // IPC_CREAT 表示若队列不存在则创建
    // 0666 是权限设置，意味着所有用户可读可写
    // msgid = msgget(MSG_KEY, IPC_CREAT | 0666);
    // if (msgid == -1) {
    //     perror("msgget");
    //     exit(EXIT_FAILURE);
    // }

    // while(1)
    // {
        // received_bytes = read_from_serial(serial_fd, receive_buffer, sizeof(receive_buffer));

        // if (received_bytes > 0) 
        // {
        //     if (msgsnd(msgid, radar_data, sizeof(LiDARFrameTypeDef), 0) == -1) 
        //     {
        //         perror("msgsnd");
        //         exit(EXIT_FAILURE);
        //     }
        // }

        
        // if (received_bytes > 0) 
        // {
        //     for(uint16 i = 0; i < received_bytes; i++)
        //     {
        //         // get_ladar_data(receive_buffer[i]);
        //         uart_que.push(receive_buffer[i]);

        //         // if(receive_buffer[i] == 0x54 && receive_buffer[i + 1] == 0x2C)
        //         // {
        //         //     printf("\r\n");
        //         // }
        //         // printf("%02X ", receive_buffer[i]);
        //     }
        // }

        
        // // 雷达数据一共47个字节
        // while(uart_que.size() >= sizeof(LiDARFrameTypeDef))
        // {
        //     radar_data[0] = uart_que.front();   // 获取队列头
        //     uart_que.pop(); // 出队
            
        //     // 获取帧头
        //     if(radar_data[0] == HEADER)
        //     {
        //         radar_data[1] = uart_que.front();   // 获取队列头
        //         uart_que.pop(); // 出队

        //         // 获取第二个数据
        //         if(radar_data[1] == 0x2C)
        //         {
        //             for(uint8 i = 2; i < sizeof(LiDARFrameTypeDef); i++)
        //             {
        //                 radar_data[i] = uart_que.front(); // 获取队列头
        //                 uart_que.pop(); // 出队
        //             }

        //             if( (uint8)CalCRC8(radar_data, sizeof(LiDARFrameTypeDef) - 1) == (uint8)radar_data[sizeof(LiDARFrameTypeDef) - 1])
        //             {
        //                 // printf("CRC 0K\n");

        //                 pthread_mutex_lock(&g_tcp_sensor_mutex);
                        
        //                 g_tcp_sensor.radar.start_angle = radar_strct.start_angle;
        //                 g_tcp_sensor.radar.end_angle = radar_strct.end_angle;
        //                 memcpy((uint8_t *)&g_tcp_sensor.radar.point[0], (uint8_t *)&radar_strct.point[0], sizeof(LidarPointStructDef) * 12);

        //                 pthread_mutex_unlock(&g_tcp_sensor_mutex);
        //                 // // 数据校验完成，发送消息队列到TCP线程中。
        //                 // if (msgsnd(msgid, radar_data, sizeof(LiDARFrameTypeDef), 0) == -1) 
        //                 // {
        //                 //     perror("msgsnd");
        //                 //     exit(EXIT_FAILURE);
        //                 // }

        //             }
        //             else
        //             {

        //                 // printf("uart_que.size() = %d\r\n", uart_que.size());
        //                 // for(uint8 i = 0; i < sizeof(LiDARFrameTypeDef); i++)
        //                 // {
        //                 //     printf("%02X ", radar_data[i]);
        //                 // }
        //                 // printf("\r\n");
        //                 printf("E %02X\n", CalCRC8(radar_data, sizeof(LiDARFrameTypeDef) - 1));
        //                 // printf("radar_strct = 0x%02X\n", CalCRC8(radar_data, sizeof(LiDARFrameTypeDef) - 1));
        //             }
        //         }
        //     }
        // }

        // // system_delay_ms(1000);
    // }
}

// uart_recv_thd_init("/dev/ttyS1", B115200)
int get_radar_init(const char* port_name, uint16 baud) 
{
    // 初始化串口
    serial_fd = init_serial_port(port_name, baud);
    if (serial_fd == -1) 
    {
        return -1;
    }

    return 0;
}


int get_radar_uart_fd()
{
    return serial_fd;
}