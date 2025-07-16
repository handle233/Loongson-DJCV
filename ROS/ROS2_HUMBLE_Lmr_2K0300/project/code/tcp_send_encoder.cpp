#include "tcp_send_encoder.h"
#include "get_config.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>

tcp_send_encoder::tcp_send_encoder()
{
    net.conn(SERVER_IP, SEND_ENCODER_PORT);
    encoder.init(); // 初始化编码器
}

tcp_send_encoder::~tcp_send_encoder()
{
    stop();
    net.shutdown();
}

void tcp_send_encoder::start()
{
    running_ = true;
    data_thread_ = std::thread(&tcp_send_encoder::get_data_thread, this);
    send_thread_ = std::thread(&tcp_send_encoder::send_data_thread, this);
}

void tcp_send_encoder::stop()
{
    running_ = false;
    if (data_thread_.joinable())
        data_thread_.join();
    if (send_thread_.joinable())
        send_thread_.join();
}

void tcp_send_encoder::get_data_thread()
{
    while (running_)
    {
        int flag = g_car_control.forward;
        double dist = encoder.Get_10ms_dist();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            encoder_distance_10ms_ += flag ? dist : -dist;
            printf("Encoder distance: %f\n", encoder_distance_10ms_);
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void tcp_send_encoder::send_data_thread()
{
    static double interg = 0;
    while (running_)
    {
        double data_to_send = 0;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            data_to_send = encoder_distance_10ms_;
            encoder_distance_10ms_ = 0;
        }
        interg += data_to_send; // 累加距离数据
        // 示例：发送 4 字节 int 数据
        int ret = net.send(&data_to_send,sizeof(data_to_send));
        data_to_send = 0;
        if (ret < 0)
        {
            printf("Failed to send encoder data\n");
        }
        else
        {
            //printf("Encoder data sent: %.2f\n", interg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每 100ms 发送一次
    }
}