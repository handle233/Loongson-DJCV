#ifndef TCP_SEND_ENCODER_H
#define TCP_SEND_ENCODER_H
#include "PZY.hpp"
#include "NetComm.h" // “˝»Î NetComm ¿‡
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <vector>

class tcp_send_encoder {
public:
    tcp_send_encoder();
    ~tcp_send_encoder();

    void start();
    void stop();
    

private:
    void get_data_thread();
    void send_data_thread();

    std::thread data_thread_;
    std::thread send_thread_;
    std::atomic<bool> running_{false};

    double encoder_distance_10ms_ = 0;
    std::mutex data_mutex_;
    NetComm net;
    Encoder10msDist encoder;
    
};
    
#endif