#include "Encoder.h"
// atomic<double> Encoder10msDist::distance_10ms_lcyx; // 10ms累计路程
// ThreadWrapper Encoder10msDist::countthread(Encoder10msDist::Encoder_count_100us);
// LS_PwmEncoder Encoder10msDist::Encoder(3,1);

Encoder10msDist::Encoder10msDist()
{
    distance_10ms_lcyx = nullptr;countthread = nullptr,PwmB = nullptr;
}

Encoder10msDist::~Encoder10msDist()
{
    run = 0;
    countthread->join();

    delete distance_10ms_lcyx;

    delete PwmB;

    delete countthread;
}

void Encoder10msDist::init(){
    distance_10ms_lcyx = new std::atomic<unsigned long long>;
    *distance_10ms_lcyx = 0;

    //Encoder = new LS_PwmEncoder(3,1);
    PwmB = new HWGpio(73,GPIO_Mode_In);

    run = 1;
    countthread = new thread(Encoder_count_100us,this);
    //countthread->detach();
}

void Encoder_count_100us(Encoder10msDist* pthis)
{
    //auto last_time = std::chrono::steady_clock::now();
    while(pthis->PwmB==nullptr);
    while(pthis->distance_10ms_lcyx==nullptr);

    while (pthis->run)//encoder_running
    {
        while(pthis->PwmB->GetGpioValue()!=true){
            usleep(10);
            //cout<<"wait high"<<endl;
        }
        //cout<<"check high"<<endl;
        pthis->distance_10ms_lcyx->exchange(pthis->distance_10ms_lcyx->load()+1);
        
        while(pthis->PwmB->GetGpioValue()!=false){
            usleep(10);
        }
    }
    cout<<"quit"<<endl;
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
