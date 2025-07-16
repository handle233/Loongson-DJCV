#include <atomic>
#include <thread>
#include "LQ_HW_GPIO.hpp"

#define NUM_ENCODER_LINE 512

class Encoder10msDist{
    public:
        std::atomic<unsigned long long> *distance_10ms_lcyx; // 10ms累计路程

        Encoder10msDist();
        ~Encoder10msDist();
        void init();

        double Get_10ms_dist();

        bool run;
        thread *countthread;
        HWGpio* PwmB;
};
void Encoder_count_100us(Encoder10msDist* pthis);

// 轮子参数结构体
struct WheelParams {
    double diameter = 0.05f; // 单位：米
    double gear_ratio = 2.0f;
    double circum() const { return diameter * 3.1415926f; }
};
const WheelParams wheel;