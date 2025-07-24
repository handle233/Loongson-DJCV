#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "PZY.hpp"
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <vector>

#define LINUX
#include "NetComm.h" // 引入 NetComm 类

class tcp_send_imu // 前向声明
{
    
public:
    NetComm net; // 实例化 NetComm 对象，用于网络通信
    // 构造函数，传入串口名
    tcp_send_imu(const char *port_name);

    // 析构函数，释放资源
    ~tcp_send_imu();

    // 启动串口读取和发送线程
    void start();

    // 停止串口读取和发送线程
    void stop();

private:
    int serial_fd_;
    typedef struct {
        uint8_t imu_data[11];
        double encoder_data;
    }data_pack;
    std::queue<data_pack> data_queue;
    std::mutex queue_mutex;
    std::condition_variable data_cond;
    std::atomic<bool> run_flag{true};
    std::thread t_reader;
    std::thread t_sender;
    // double encoder_distance_10ms_ = 0;
    Encoder10msDist encoder;
    
    // 打开串口
    void open_serial(const char *port_name);

    // 读取串口数据的线程函数
    void read_thread();

    // 发送串口数据的线程函数
    void send_thread();
};

#endif 




