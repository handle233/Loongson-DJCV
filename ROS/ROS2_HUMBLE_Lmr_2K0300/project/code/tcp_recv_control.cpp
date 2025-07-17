#include "zf_common_headfile.h"
#include "tcp_recv_control.h"
#include "radar_data_protocol.h"
#include "get_config.h"

#include "LCYX.hpp"

car_control_typedef tcp_car_control;
car_control_typedef g_car_control;

static int tcp_client_recv_socket;


extern ServoController Servo;
extern MotorController Motor;

void close_tcp_recv_control(void)
{
    close(tcp_client_recv_socket);
}

void tcp_recv_control_thd_entry(void)
{
    // 尝试设置新的高优先级,值越大优先级越高（1-99）
	struct sched_param param;
	param.sched_priority = TCP_RECV_CONTROL_THD_PRO;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    printf("tcp_init recv ing... \r\n");

    while(1)
    {
        // 初始化 tcp server
        do{
            tcp_client_recv_socket = tcp_init(SERVER_IP, RECV_CONTROL_PORT);
            if(tcp_client_recv_socket < 0){
                system_delay_ms(1000);
                printf("tcp_client_recv_socket error \r\n");
            }else{
                printf("tcp_client_recv_socket OK \r\n");
            }
        }while (tcp_client_recv_socket <= 0);

        while(1){
            // TCP周期接收控制数据
            int str_len = tcp_client_read_data(tcp_client_recv_socket, (uint8_t *)&tcp_car_control, sizeof(car_control_typedef));
            
            if(str_len <= 0){
                tcp_car_control.left_speed = 0;
                tcp_car_control.right_speed = 0;
                tcp_car_control.servo_duty = 90;
                // 下面是LCYX的扩展
                tcp_car_control.servo_angle = 0; // 新增
                tcp_car_control.speed = 0;       // 新增
                tcp_car_control.forward = 0;     // 新增
                //上面是LCYX的扩展
                break;
            }else if(str_len > 0){
                //抽样显示
                static long long lasttime = 0;
                timeval tv;
                gettimeofday(&tv,nullptr);
                long long time = tv.tv_sec*1000000+tv.tv_usec;

                if(time-lasttime > 1000*100){
                    cout << "left_speed = " << tcp_car_control.left_speed << 
                    ", right_speed = " << tcp_car_control.right_speed <<
                    ", servo_duty = " << tcp_car_control.servo_duty << endl;
                    // LCYX的扩展
                    lasttime = time;
                }
            }
    
            g_car_control.left_speed = tcp_car_control.left_speed; // LCYX: 调整速度比例
            g_car_control.right_speed = tcp_car_control.right_speed; // LCYX: 调整速度比例
            g_car_control.servo_duty = tcp_car_control.servo_duty;
            if(tcp_car_control.forward!=0){
                g_car_control.forward = tcp_car_control.forward;
            }
            

            //设置电机
            if (g_car_control.servo_duty < 0 ){
                g_car_control.servo_duty = 0; // 限制舵机角度最小值为0
            }else if (g_car_control.servo_duty > 180){
                g_car_control.servo_duty = 180; // 限制舵机角
            }
            Servo.setAngle(g_car_control.servo_duty);  // 设置舵机角度

            if(g_car_control.left_speed>=0){
                Motor.setLeftWheel(g_car_control.left_speed,1);
            }else{
                Motor.setLeftWheel(-g_car_control.left_speed,0);
            }
            if(g_car_control.right_speed>=0){
                Motor.setRightWheel(g_car_control.right_speed,1);
            }else{
                Motor.setRightWheel(-g_car_control.right_speed,0);
            }
            
            cout<<"left "<<g_car_control.left_speed<<"right "<<g_car_control.right_speed<<endl;
        }
    }
}

