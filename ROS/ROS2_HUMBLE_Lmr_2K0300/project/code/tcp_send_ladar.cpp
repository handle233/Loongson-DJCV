// 传输雷达数据
// 直接使用TCP上传串口接收到的雷达数据。

#include "zf_common_headfile.h"
#include "tcp_send_ladar.h"
#include "radar_data_protocol.h"

#include "get_radar.h"
#include "get_config.h"


uint32  read_len = 0;
uint8   recv_buff[1024];
uint8   temp_str[] = "seekfree this is TCP demo.\r\n";
uint8   read_str[] = "read data:\r\n";


// // 定义消息队列的键值，要和发送端保持一致
// #define MSG_KEY 1234

// static LiDARFrameTypeDef radar_strct;

static int tcp_client_send_ladar_socket;

void close_tcp_send_ladar(void)
{
    close(tcp_client_send_ladar_socket);
}


void tcp_send_ladar_thd_entry(void)
{
    // 尝试设置新的高优先级,值越大优先级越高（1-99）
	struct sched_param param;
	param.sched_priority = TCP_SEND_LADAR_THD_PRO;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    // printf("tcp_init send ing... \r\n");
    

    uint8_t receive_buffer[512];
    int32 received_bytes;
    int uart_fd = 0;
    int ret = 0;


    while(1)
    {
        do
        {
            tcp_client_send_ladar_socket = tcp_init(SERVER_IP, SEND_LADAR_PORT);
            if(tcp_client_send_ladar_socket < 0)
            {
                system_delay_ms(1000);
                printf("tcp_client_send_ladar_socket error \r\n");
            }
            else
            {
                printf("tcp_client_send_ladar_socket OK \r\n");
            }
            
        } while (tcp_client_send_ladar_socket < 0);
        

        if(uart_fd == 0)
        {
            // 打开串口
            ret = get_radar_init(UART_PATH, UART_BAUD);
            if(ret < 0)
            {
                printf("uart_recv_thd_init error \r\n");
                exit(0);
            }

            uart_fd = get_radar_uart_fd();
        }

        while(1)
        {
            received_bytes = read_from_serial(uart_fd, receive_buffer, sizeof(receive_buffer));
            if(received_bytes > 0)
            {
                // TCP周期发送传感器数据
                if(tcp_client_send_data(tcp_client_send_ladar_socket, receive_buffer, received_bytes) < 0)
                {
                    break;
                }
            }
        }
    }

}

