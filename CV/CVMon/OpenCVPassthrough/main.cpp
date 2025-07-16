/**
* project : OpenCVPassthrough
* file	  : main.cpp
* Copyright <c> handle 2025 all rights reserved.
**/
#define WINDOWS
#include <iostream>
#include <string>

#include "opencv2/opencv.hpp"
#include "NetComm.h"
#include "PID.h"

#define WIDTH 640
#define HEIGHT 330

#pragma warning(disable:4996)

using namespace std;
using namespace cv;

#define LIMIT(x) (x<0?0:(x>=640?639:x))

#define HEIGHT 330

void dotway(Mat img, Mat imgshow, vector<int>& path, vector<int>& dpath, vector<int>& weight) {
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
    for (int a = imgshow.rows - 1; a >= 0; a--) {
        //cout << "get" << imgshow.rows - 1 - a << endl;
        int t = LIMIT(path[imgshow.rows - 1 - a]);
        imgshow.at<Vec3b>(a, t)[0] = 0;
        imgshow.at<Vec3b>(a, t)[1] = 255;
        imgshow.at<Vec3b>(a, t)[2] = 0;
        path[imgshow.rows-1 - a] -= 640 / 2;
        t = LIMIT(dpath[imgshow.rows-1 - a] + 640 / 2);
        imgshow.at<Vec3b>(a, t)[0] = 255;
        imgshow.at<Vec3b>(a, t)[1] = 0;
        imgshow.at<Vec3b>(a, t)[2] = 0;
        t = LIMIT(weight[imgshow.rows - 1 - a]);
        imgshow.at<Vec3b>(a, t)[0] = 0;
        imgshow.at<Vec3b>(a, t)[1] = 0;
        imgshow.at<Vec3b>(a, t)[2] = 255;
    }

    path.resize(Allegate);
    dpath.resize(Allegate);
    weight.resize(Allegate);
}

int k1, k2;
double pathcore, dpathcore;
int firstrun = 0;
vector<int> rightseq, drightseq;
int rightsum, drightsum;
void lineprocess(vector<int>& path, vector<int>& dpath, double& axis) {
    int pathsum = 0;
    if (firstrun == 0) {
        rightseq.resize(HEIGHT);
        drightseq.resize(HEIGHT);

        for (int a = 0; a < HEIGHT; a++) {
            if (a < HEIGHT / 3) {
                rightsum += a;
                rightseq[a] = a;
            }
            if (a > HEIGHT / 3) {
                rightsum += (HEIGHT / 3 - (a - HEIGHT / 3) / 2);
                rightseq[a] = (HEIGHT / 3 - (a - HEIGHT / 3) / 2);
            }
        }

        for (int a = 0; a < HEIGHT; a++) {
            if (a < HEIGHT / 3) {
                drightsum += a;
                drightseq[a] = a;
            }
            if (a > HEIGHT / 3) {
                drightsum += (HEIGHT / 3 - (a - HEIGHT / 3) / 2);
                drightseq[a] = (HEIGHT / 3 - (a - HEIGHT / 3) / 2);
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

    pathcore = (1. * pathsum) / rightsum / 100;
    if (pathcore < 0) {
        pathcore = pathcore / 2.;
    }
    dpathcore = (1. * dpathsum) / drightsum;
    axis = (pathcore * 1.*k1/1000. + dpathcore * 1.*k2/1000.);//0.2

}

void angleprocess(double axis,double& angle, double& pid) {

    static PIDController AnglePID(1, 0.1, 1, -1.2, 1.2);


    pid = AnglePID.compute(axis, 1. / 8.);

    angle = 90 - axis * 90.0 / 1;

    if (angle < 0) {
        angle = 0;
    }
    else if (angle > 180) {
        angle = 180;
    }
}

void judgePark( Mat imgshow,int inbegin,int inend,int outbegin,int outend) {
    
    //画新线
    int l = imgshow.rows;

    for (int a = 0; a < 640 && inbegin != -1; a++) {
        imgshow.at<Vec3b>(l - 1 - inbegin, a)[0] = 0;
        imgshow.at<Vec3b>(l - 1 - inbegin, a)[1] = 0;
        imgshow.at<Vec3b>(l - 1 - inbegin, a)[2] = 255;
    }
    for (int a = 0; a < 640 && inend != -1; a++) {
        imgshow.at<Vec3b>(l - 1 - inend, a)[0] = 0;
        imgshow.at<Vec3b>(l - 1 - inend, a)[1] = 0;
        imgshow.at<Vec3b>(l - 1 - inend, a)[2] = 255;
    }
    for (int a = 0; a < 640 && outbegin != -1; a++) {
        imgshow.at<Vec3b>(l - 1 - outbegin, a)[0] = 0;
        imgshow.at<Vec3b>(l - 1 - outbegin, a)[1] = 255;
        imgshow.at<Vec3b>(l - 1 - outbegin, a)[2] = 0;
    }
    for (int a = 0; a < 640 && outend != -1; a++) {
        imgshow.at<Vec3b>(l - 1 - outend, a)[0] = 0;
        imgshow.at<Vec3b>(l - 1 - outend, a)[1] = 255;
        imgshow.at<Vec3b>(l - 1 - outend, a)[2] = 0;
    }
}

double Kp, Ki, Kd;
int main() {
	NetComm client;

connect:
	if (client.conn("192.168.3.12",1234) == -1) {
		printf("connect invalid host.\n");
		Sleep(500);
		goto connect;
	}

	int lasttime = 0, waittime = 0;
	int end = 1;

    k1 = 220, k2 = 590;

    client.sendpack(&k1, 4);
    client.sendpack(&k2, 4);

    Kp = 1.0;
    Ki = 0.7;
    Kd = 0.05;

    client.sendpack(&Kp, 8);
    client.sendpack(&Ki, 8);
    client.sendpack(&Kd, 8);

	int count = 0;


	while (end) {
        //读取小车数据
        double pid, axis;
        int readlen = client.recvpacklength();
        client.recvpack(&pathcore, 8);
        readlen = client.recvpacklength();
        client.recvpack(&dpathcore, 8);

        readlen = client.recvpacklength();
        client.recvpack(&axis, 8);
        readlen = client.recvpacklength();
        client.recvpack(&pid, 8);

        int inbegin, inend, outbegin, outend;
        readlen = client.recvpacklength();
        client.recvpack(&inbegin, 4);
        readlen = client.recvpacklength();
        client.recvpack(&inend, 4);
        readlen = client.recvpacklength();
        client.recvpack(&outbegin, 4);
        readlen = client.recvpacklength();
        client.recvpack(&outend, 4);

        cout << "actuall: axis " << setprecision(2) << axis << " pid " << pid;
        cout << " path "<< setprecision(2) << pathcore << " dpath " << dpathcore << endl;
        //图像读取
		readlen = client.recvpacklength();
		unsigned char* jpgbmp = new unsigned char[readlen];

		if (client.recvpack(jpgbmp, readlen) == -1) {
            client.shutdown();
            cout << "link failed" << errno << endl;
            end = 0;
		}

		std::vector<unsigned char> jpgvec(readlen);

		unsigned char* pjpg = jpgbmp;
		for (auto& i : jpgvec) {
			i = *pjpg;
			pjpg++;
		}

		delete[] jpgbmp;
		cv::Mat img = cv::imdecode(jpgvec, IMREAD_COLOR);
		//cv::Mat img(rows, cols, CV_8UC1, jpgbmp);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        vector<int> path, dpath,weight;
        dotway(gray,img,path,dpath,weight);

        judgePark(img,inbegin,inend,outbegin,outend);

        double axisp;
        //lineprocess(path, dpath, axisp);

        double angle, pidp;
        //angleprocess(axisp, angle, pidp);
        //cout << "predict: axis " << setprecision(2) << axisp << " pid " << pidp << endl;
        //cout << "path "<< setprecision(2) << pathcore << "dpath" << dpathcore << endl;

        double drawx = 1.*(img.cols/2) / axis;
        cv::Point start(img.cols / 2, img.rows),
            ends(img.cols / 2 + drawx * axis, img.rows - abs(drawx));
        cv::line(img, start, ends, Scalar(255, 255, 0));

        drawx = 1. * (img.cols / 2) / pid;
        start = Point(img.cols / 2, img.rows),
            ends = Point(img.cols / 2 + drawx * pid, img.rows - abs(drawx));
        cv::line(img, start, ends, Scalar(0, 255, 255));


		//cv::imshow("cam", gray);
        cv::imshow("process", img);

		int key = cv::waitKey(1);
		if (key =='q') {
			end = 0;
		}
		if (key == 's') {
			std::string fstr = "out";
			fstr += count+48;
			fstr += ".jpg";
			cv::imwrite(fstr.c_str(), img);
			fstr += "hold.jpg";
			count++;
		}

		lasttime = timeGetTime();
        //printf("\033[H\033[J");
	}

	client.shutdown();

	system("pause");
	return 0;
}