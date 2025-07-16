#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <vector>

#define LINUX
#include "NetComm.h" // ���� NetComm ��

class tcp_send_imu // ǰ������
{
    
public:
    NetComm net; // ʵ���� NetComm ������������ͨ��
    // ���캯�������봮����
    tcp_send_imu(const char *port_name);

    // �����������ͷ���Դ
    ~tcp_send_imu();

    // �������ڶ�ȡ�ͷ����߳�
    void start();

    // ֹͣ���ڶ�ȡ�ͷ����߳�
    void stop();

private:
    int serial_fd_;
    std::queue<std::vector<uint8_t>> data_queue;
    std::mutex queue_mutex;
    std::condition_variable data_cond;
    std::atomic<bool> run_flag{true};

    std::thread t_reader;
    std::thread t_sender;

    // �򿪴���
    void open_serial(const char *port_name);

    // ��ȡ�������ݵ��̺߳���
    void read_thread();

    // ���ʹ������ݵ��̺߳���
    void send_thread();
};

#endif 




