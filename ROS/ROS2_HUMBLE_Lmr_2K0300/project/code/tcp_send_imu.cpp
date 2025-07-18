#include "tcp_send_imu.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include "get_config.h"
#define BAUDRATE B115200

tcp_send_imu::tcp_send_imu(const char *port_name)
{
    net.conn(SERVER_IP, SEND_IMU_PORT);
    open_serial(port_name);
    encoder.init(); // 初始化编码器
}

tcp_send_imu::~tcp_send_imu()
{
    close(serial_fd_);
    net.shutdown();
}

void tcp_send_imu::start()
{
    run_flag = true;
    t_reader = std::thread(&tcp_send_imu::read_thread, this);
    t_sender = std::thread(&tcp_send_imu::send_thread, this);
}

void tcp_send_imu::stop()
{
    run_flag = false;
    data_cond.notify_all();
    t_reader.join();
    t_sender.join();
}

void tcp_send_imu::open_serial(const char *port_name)
{
    serial_fd_ = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1)
    {
        perror("open serial failed");
        exit(1);
    }

    struct termios tty{};
    tcgetattr(serial_fd_, &tty);
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(serial_fd_, TCSANOW, &tty);
}

void tcp_send_imu::read_thread()
{
    uint8_t recv_buf[128];
    uint8_t frame_buf[11];
    uint8_t key_ = 0;
    data_pack data;

    while (run_flag)
    {
        int n = read(serial_fd_, recv_buf, sizeof(recv_buf));
        if (n <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        for (int i = 0; i < n; ++i)
        {
            uint8_t byte = recv_buf[i];
            data.imu_data[key_++] = byte;

            if (data.imu_data[0] != 0x55)
            {
                key_ = 0;
                continue;
            }

            if (key_ < 11)
                continue;
            
            int flag = g_car_control.forward;
            double dist = encoder.Get_10ms_dist();
            static double total_dist = 0;

            total_dist += dist;
            data.encoder_data = flag>0 ? dist : -dist;
            //printf("total distance: %f\n", total_dist);  
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                data_queue.emplace(data);
            }

            data_cond.notify_one();
            key_ = 0;
        }
        
    
    }
}

void tcp_send_imu::send_thread()
{
    while (run_flag)
    {
        // 捕获 'this' 以便在 Lambda 中访问类的成员变量
        std::unique_lock<std::mutex> lock(queue_mutex);
        data_cond.wait(lock, [this] { return !data_queue.empty() || !run_flag; });

        while (!data_queue.empty())
        {
            data_pack frame = data_queue.front();
            data_queue.pop();
            lock.unlock();
            net.send(&frame, sizeof(frame));
            lock.lock();
        }
    }
}
