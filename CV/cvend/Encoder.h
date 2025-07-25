#include <atomic>
#include <thread>
#include "LQ_HW_GPIO.hpp"

#define NUM_ENCODER_LINE 512

class Encoder10msDist{
    public:
        std::atomic<unsigned long long> *distance_10ms_lcyx; // 10ms�ۼ�·��

        Encoder10msDist();
        ~Encoder10msDist();
        void init();

        double Get_10ms_dist();

        bool run;
        thread *countthread;
        HWGpio* PwmB;
};
void Encoder_count_100us(Encoder10msDist* pthis);

// ���Ӳ����ṹ��
struct WheelParams {
    double diameter = 0.05f; // ��λ����
    double gear_ratio = 2.0f;
    double circum() const { return diameter * 3.1415926f; }
};
const WheelParams wheel;