#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include <thread>
#include <mutex>

#include <opencv.hpp>
#include <LCYX.hpp>
#define LINUX
#include "NetComm.h"
#include "PID.h"
#include "Encoder.h"
#include <sys/time.h>

using namespace std;
using namespace cv;

#define WIDTH 640
#define HEIGHT 330
#define SPEED 18

void dotway(cv::Mat img, vector<int>& diff, vector<int>& redline, vector<int>& weight);

void camthread(Mat *pPict);

void lineprocess(vector<int> &path,vector<int> &dpath,double &axis);
void angleprocess(double axis,double& angle, double& pid);

void judgePark(vector<int>& weight, vector<int> &dpath, vector<int>& path);

void parking();

//共享数据
int endloop=0,readyflag = 0;
int k1,k2;
double axis,pid;

double Kp,Ki,Kd;

double pathcore, dpathcore;


MotorController Motor;
ServoController roll;

mutex lck;

int main(int argc,char* argv[]){
    //socket通信
    cv::Mat dst;
    
    thread webcam(camthread,&dst);
    webcam.detach();


    while(1){
        
        //打开摄像头设备
        cv::VideoCapture cam("/dev/video0");
        char readbuf[256] = "",sendbuf[256] = "";

        if(!cam.isOpened()){
            std::cout<<"open failed"<<std::endl;
            continue;
        }

        cv::Mat img;
        
        cam >> img;

        cv::Mat gray;

        timeval tv;
        long begintime,nowtime;
        while(endloop){
            gettimeofday(&tv,nullptr);
            begintime = tv.tv_sec*1000000+tv.tv_usec;
            //获取图像数据
            cam>>img;

            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"获取图像时间:"<<nowtime-begintime<<"us"<<endl;

            //设置ROI
            Rect ROI(0,0,640,330);
            Mat roiimg(img,ROI);

            //转180度
            Mat rot;
            rotate(roiimg,rot,ROTATE_180);

            //灰度化
            cv::cvtColor(rot,gray,cv::COLOR_BGR2GRAY);

            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"灰度处理时间:"<<nowtime-begintime<<"us"<<endl;

            //模糊
		    cv::Mat blur;
		    cv::GaussianBlur(gray, blur, cv::Size(3,3), 0);
            //取阈值
            lck.lock();

		    cv::threshold(blur, dst, 95, 255, cv::THRESH_BINARY);

            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"模糊取阈时间:"<<nowtime-begintime<<"us"<<endl;
            
            vector<int> path,dpath,weight;
            dotway(dst,path,dpath,weight);
            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"采样画点时间:"<<nowtime-begintime<<"us"<<endl;
            judgePark(weight,dpath,path);
            cout<<"to main"<<endl;
            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"寻找车库时间:"<<nowtime-begintime<<"us"<<endl;
            readyflag = 1;
            lineprocess(path, dpath, axis);
            double angle;
            angleprocess(axis, angle, pid);
            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"计算角度时间:"<<nowtime-begintime<<"us"<<endl;
            

            roll.setAngle(angle);
            cout<<"角度："<<angle<<endl;
            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"总共花费时间:"<<nowtime-begintime<<"us"<<endl;

            lck.unlock();
            int deltatime = (1000000/60)-(nowtime-begintime);
            if(deltatime<0){
                cout<<"主函时间不够"<<endl;
                deltatime=0;
            }
            cout<<endl;
            usleep(deltatime);
            //printf("\033[H\033[J");
        }
        
        Motor.move(0,true);
        roll.setAngle(90);
    }

    return 0;
}

int rightsum,drightsum;
vector<int> rightseq,drightseq;
int firstrun=0;

void lineprocess(vector<int> &path,vector<int> &dpath, double& axis){
    int pathsum = 0;
    if(firstrun==0){
        rightseq.resize(path.size());
        drightseq.resize(dpath.size());

        for (int a = 0; a < path.size(); a++) {
            if (a < path.size() / 3) {
                rightsum += a;
                rightseq[a] = a;
            }
            if(a>path.size()/3){
                rightsum += (path.size() / 3 - (a - path.size() / 3) / 2);
                rightseq[a] = (path.size() / 3 - (a - path.size() / 3) / 2);
            }
        }
        
        for (int a = 0; a < dpath.size(); a++) {
            if (a < dpath.size() / 3) {
                drightsum += a;
                drightseq[a] = a;
            }
        if(a>dpath.size()/3){
                drightsum += (dpath.size() / 3 - (a - dpath.size() / 3) / 2)+0.65;
                drightseq[a] = (dpath.size() / 3 - (a - dpath.size() / 3) / 2);
            }
        }

        firstrun = 1;
    }

    for (int a = 0; a < path.size(); a++) {
         pathsum += path[a] * rightseq[a];
    }

    int dpathsum = 0;
    for (int a = 0; a < dpath.size(); a++) {
        dpathsum += dpath[a] * drightseq[a];
    }


    pathcore = ((1. * pathsum) / rightsum / 100) * 1.*k1/1000.;
    dpathcore = ((1. * dpathsum) / drightsum) * 1.*k2/1000.;
    axis = (pathcore  + dpathcore);//0.2
}

void angleprocess(double axis,double& angle, double& pid) {

    static PIDController AnglePID(Kp, Ki, Kd, -1.2, 1.2);

    pid = AnglePID.compute(axis, 1. / 60);

    angle = 90 - pid * 90.0 / 1;

    if (angle < 0) {
        angle = 0;
    }
    else if (angle > 180) {
        angle = 180;
    }
}


int inbegin=-1, inend=-1, outbegin=-1, outend=-1;

static int parkcount = 0;

void camthread(Mat *pPict){
    NetComm Server;
    Server.serv(1234);
    while(1){
        
        Server.dispach(25);

        int l = Server.recvpacklength();
        Server.recvpack(&k1,l);
        l = Server.recvpacklength();
        Server.recvpack(&k2,l);
    
        l = Server.recvpacklength();
        Server.recvpack(&Kp,l);
        l = Server.recvpacklength();
        Server.recvpack(&Ki,l);
        l = Server.recvpacklength();
        Server.recvpack(&Kd,l);
        cout<<"k1: "<<k1<<" k2: "<<k2<<endl;
        cout<<"PID: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;

        endloop = 1;

        Motor.move(SPEED,true);

    
        timeval tv;
        long begintime = 0,nowtime = 0;
        while(endloop==1){
            gettimeofday(&tv,nullptr);
            begintime = tv.tv_sec*1000000+tv.tv_usec;

            while(readyflag==0){
                usleep(100);
            }

            lck.lock();

            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            cout<<"主函同步时间:"<<nowtime-begintime<<"us"<<endl;

            Server.sendpack(&pathcore,8);
            Server.sendpack(&dpathcore,8);
            Server.sendpack(&axis,8);
            Server.sendpack(&pid,8);

            Server.sendpack(&inbegin,4);
            Server.sendpack(&inend,4);
            Server.sendpack(&outbegin,4);
            Server.sendpack(&outend,4);


            std::vector<unsigned char> jpgvec;
            cv::imencode(".jpg",*pPict,jpgvec);

            readyflag = 0;
            lck.unlock();
        
            unsigned char *jpgbuf = new unsigned char[jpgvec.size()];
                    
            unsigned char *pbuf = jpgbuf;
            for(auto i : jpgvec){
                *pbuf = i;
                pbuf++;
            }
                    
            int ret = Server.sendpack(jpgbuf,jpgvec.size());
            if(ret==-1){
                cout<<"[ERROR]:send cam error."<<errno<<endl;
                Server.shutdown();
                endloop = 0;
                readyflag = 0;
                parkcount = 0;
                continue;
            }
    
            delete[] jpgbuf;
            gettimeofday(&tv,nullptr);
            nowtime = tv.tv_sec*1000000+tv.tv_usec;
            int deltatime = 1000000/16-(nowtime-begintime);
            cout<<"图传剩余时间:"<<deltatime<<"us"<<endl;
            if(deltatime<0){
                deltatime=0;
                cout<<"剩余时间不够"<<endl;
            }
            usleep(deltatime);
        }
    }
    cout<<"quit thread"<<endl;
}


#define LIMIT(x) (x<0?0:(x>=640?639:x))

void dotway(Mat img, vector<int>& path, vector<int>& dpath, vector<int>& weight) {
    //640x330
path.resize(img.rows);
dpath.resize(img.rows);
weight.resize(img.rows);
int Allegate = img.rows;

for (int l = img.rows - 1; l >= 0; l--) {
    //行缓冲区
    vector<uchar> line(img.cols);
    for (int a = 0; a < img.cols; a++) {
        line[a] = img.at<uchar>(l, a);
    }
    //上一轮左右点
    static int lastlow = 0, lasthigh = 0;

    //first line find line
    if (l == img.rows - 1) {
        int countw = 0, middle = 0;
        //统计白色
        for (auto& i : line) {
            if (i == 255)
                countw++;
        }
        //找中间白色
        for (int i = 0; i < img.cols; i++) {
            if (line[i] == 255)
                middle++;

            if (middle == countw / 2) {
                middle = i;
                break;
            }
        }

        path[0] = middle;
        //cout << "0" << middle << endl;

        lastlow = 0, lasthigh = 640;
        //寻找左极限
        for (int i = middle; i >= 0; i--) {
            if (line[i] == 0) {
                lastlow = i + 1;
                break;
            }
        }
        //寻找右极限
        for (int i = middle; i < img.cols; i++) {
            if (line[i] == 0) {
                lasthigh = i - 1;
                break;
            }
        }
        weight[0] = lasthigh - lastlow;
        continue;
    }

    //第二行寻线

    int rownum = (img.rows - 1) - l;
    //查看线的连续性
    if (line[LIMIT(path[rownum - 1])] == 255) {
        //连续
        int low = 0, high = 640;
        //寻找左极限
        for (int i = LIMIT(path[rownum - 1]); i >= 0; i--) {
            if (line[i] == 0) {
                low = i + 1;
                break;
            }
        }
        //寻找右极限
        for (int i = LIMIT(path[rownum - 1]); i < img.cols; i++) {
            if (line[i] == 0) {
                high = i - 1;
                break;
            }
        }
        weight[rownum] = high - low;
        //出现碰壁
        if (low < 0 && high < 0) {
            path[rownum] = 640/2;
            lastlow = 0, lasthigh = 640;
            //cout << "||" << endl;
        }
        else if (low < 0) {
            path[rownum] = path[rownum - 1] + (high - lasthigh);
            lastlow = 0, lasthigh = high;
            //cout << "|0" << path[rownum] << endl;
        }
        else if (high < 0) {
            path[rownum] = path[rownum - 1] + (low - lastlow);
            lastlow = low, lasthigh = 0;
            //cout << "0|" << path[rownum] << endl;
        }
        else {
            //没有碰壁
            path[rownum] = (low + high) / 2;
            lastlow = low, lasthigh = high;
            //cout << "00" << path[rownum] << endl;
        }
        dpath[rownum - 1] = path[rownum] - path[rownum - 1];
        //cout << "delta" << dpath[rownum - 1] << endl;
    }
    else {
        //不连续
        Allegate = (img.rows - 1) - l;
        break;
        //直接结束巡线
    }
}

    //输出修改图
    for (int a = img.rows - 1; a >= 0; a--) {
        path[img.rows-1 - a] -= 640 / 2;
    }
    path.resize(Allegate);
    dpath.resize(Allegate);
    weight.resize(Allegate);
}


void judgePark(vector<int>& weight,vector<int> &dpath, vector<int>& path) {
    inbegin=-1,inend=-1,outbegin=-1,outend=-1;

    vector<int>dweight(weight.size()-1);

    for (int i = 1; i < weight.size(); i++) {
        dweight[i - 1] = weight[i] - weight[i - 1];
    }

    int find_in=0;//0 not found 1 counting 2 find over
    int find_out = 0;//0 not found 1 counting 2 findover
    for (int i = 0; i < dweight.size(); i++) {

        //判定进入
        if (dweight[i] > 3 && find_in == 0) {
            find_in = 1;
            inbegin = i;
        }
        if (dweight[i] < -1 && find_in == 1) {
            find_in = 2;
            inend = i;
        }
        int interweight = 0;

        for (int i = inbegin; i <= inend && find_in == 2; i++) {
            interweight += dweight[i];
        }

        if (interweight < 15 && find_in == 2) {
            find_in = 0;
            //cout << "累计长度不足车库入口"<<i << endl;
            //continue;
        }

        //判定出去
        if (dweight[i] < -5 && find_out == 0) {
            find_out = 1;
            outbegin = i;
        }
        if (dweight[i] > -5 && find_out == 1) {
            find_out = 2;
            outend = i;
            interweight = 0;
        }

        for (int i = outbegin; i <= outend && find_out == 2; i++) {
            interweight += dweight[i];
        }

        if (interweight > -10 && find_out == 2) {
            find_out = 0;
            //cout << "累计长度不足车库出口" << i << endl;
        }
    }

    if (find_in == 2 && find_out == 2 && (outbegin>inend)) {
        //线性拟合
        int delta = path[outend] - path[inbegin];
        double axis = 1. * delta / (1.*(outend - inbegin));
    
        for (int a = inbegin; a < outend; a++) {
            path[a] = path[inbegin] + axis * (a - inbegin);
        }
        for (int a = inbegin; a < outend; a++) {
            dpath[a] = 0;
        }
    }


    if(find_out==2){
        cout<<"找到出口！"<<endl;
        if(outend<50){
            lck.unlock();
            parking();
            //parking();
            lck.lock();
            Motor.move(SPEED,1);
            cout<<"exit judge park"<<endl;
        }
    }
}


void handparking(){
    Encoder10msDist Encoder;
    Motor.move(0,1);
    Encoder.init();
    sleep(1);
    double dist = 0.;

    while(1){
        dist += Encoder.Get_10ms_dist();
        cout<<"dist:"<<dist<<endl;
        usleep(1000*200);
    }
}
void second_parking(){
    double interg = 0.;
    Encoder10msDist Encoder;
    Encoder.init();
    roll.setAngle(90);
    Motor.move(12,1);
    
    while(interg < 0.10){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;

    roll.setAngle(0);
    Motor.move(12,0);

    while(interg < 0.20){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;

    roll.setAngle(90);
    Motor.move(12,0);

    while(interg < 0.25){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;

    roll.setAngle(180);
    Motor.move(12,0);

    while(interg < 0.15){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;
    //入库出库
    roll.setAngle(180);
    Motor.move(12,1);

    while(interg < 0.15){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;

    roll.setAngle(90);
    Motor.move(12,1);

    while(interg < 0.07){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;
    
    roll.setAngle(0);
    Motor.move(12,1);

    while(interg < 0.15){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;


    Motor.move(15,1);//前进
    usleep(1000*300);
}
void first_parking(){
    double interg = 0.;
    Encoder10msDist Encoder;
    Encoder.init();

    Motor.move(12,0);
    while(interg < 0.1){
        interg += Encoder.Get_10ms_dist();
    }
    interg = 0;
    Motor.move(12,0);

    roll.setAngle(0);

    interg = 0;

    while(interg < 0.40){
        interg += Encoder.Get_10ms_dist();
    }

    roll.setAngle(90);

    interg = 0;

    while(interg < 0.20){
        interg += Encoder.Get_10ms_dist();
    }
    Motor.move(0,0);
    sleep(5);

    Motor.move(12,1);//前进

    interg = 0;

    while(interg < 0.20){
        interg += Encoder.Get_10ms_dist();
    }

    roll.setAngle(0);

    interg = 0;

    while(interg < 0.45){
        interg += Encoder.Get_10ms_dist();
    }
    roll.setAngle(90);

    Motor.move(15,1);//前进
    usleep(1000*500);

    //Motor.move(12,0);
    cout<<"exit parking"<<endl;
}

void parking(){
    static long long lasttime = 0;
    timeval tv;
    gettimeofday(&tv,nullptr);

    long long time = tv.tv_sec*1000000 + tv.tv_usec;

    if(time-lasttime < 1000*5000){
        cout<<"时间太近不算"<<endl;
        return;
    }
    lasttime = time;

    switch (parkcount)
    {
    case 0:
        first_parking();
        break;
    case 1:
        second_parking();
        break;
    default:
        handparking();
        break;
    }
    parkcount++;
}