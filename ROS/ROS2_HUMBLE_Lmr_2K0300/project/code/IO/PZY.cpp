#include "PZY.hpp"
// atomic<double> Encoder10msDist::distance_10ms_lcyx; // 10ms累计路程
// ThreadWrapper Encoder10msDist::countthread(Encoder10msDist::Encoder_count_100us);
// LS_PwmEncoder Encoder10msDist::Encoder(3,1);

Encoder10msDist::Encoder10msDist()
{
    distance_10ms_lcyx = nullptr;countthread = nullptr,PwmB = nullptr;
}

void Encoder10msDist::init(){
    distance_10ms_lcyx = new std::atomic<unsigned long long>;
    *distance_10ms_lcyx = 0;

    //Encoder = new LS_PwmEncoder(3,1);
    PwmB = new HWGpio(73,GPIO_Mode_In);

    countthread = new thread(Encoder_count_100us,this);

    int policy = SCHED_FIFO; // 设置线程调度策略为实时
    int priority = 10;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr,policy);
    sched_param param;
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr,&param);
    pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
    pthread_t handle = countthread->native_handle();
    pthread_setschedparam(handle,policy,&param);
}

void Encoder_count_100us(Encoder10msDist* pthis)
{
    //auto last_time = std::chrono::steady_clock::now();
    while(pthis->PwmB==nullptr);
    while(pthis->distance_10ms_lcyx==nullptr);

    while (1)//encoder_running
    {
        while(pthis->PwmB->GetGpioValue()!=true){
            usleep(50);
            //cout<<"wait high"<<endl;
        }
        //cout<<"check high"<<endl;
        pthis->distance_10ms_lcyx->exchange(pthis->distance_10ms_lcyx->load()+1);
        
        while(pthis->PwmB->GetGpioValue()!=false){
            usleep(50);
        }
        //cout<<"check low"<<endl;

        // double speed = pthis->Encoder->Update(); // 假设返回圈/秒
        // auto now = std::chrono::steady_clock::now();
        // double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count()/1e6f; // 转换为秒

        // double delta_s = speed * dt * wheel.circum() / wheel.gear_ratio;
        // double tmp = pthis->distance_10ms_lcyx->load();
        // tmp += delta_s;
        // pthis->distance_10ms_lcyx->store(tmp);

        // std::this_thread::sleep_for(std::chrono::microseconds(50));
        // last_time = now;
        // //cout<<"threadrunning"<<endl;
    }
}

double Encoder10msDist::Get_10ms_dist()
{
    
    if(PwmB==nullptr || distance_10ms_lcyx==nullptr){
        return 0;
    }
    unsigned long long count = distance_10ms_lcyx->exchange(0);
    double distan = ((1. * count) / NUM_ENCODER_LINE) / wheel.gear_ratio * wheel.circum();

    return distan;
}
