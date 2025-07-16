#ifndef _class_posix_pit_h_
#define _class_posix_pit_h_

#include "zf_common_typedef.h"


#include <iostream>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>

class PosixPIT
{
	private:
		timer_t timerid;
		struct sigevent sev;
		struct itimerspec its;
		uint16_t ms;
		void (*call_back_func)(union sigval sv);
		void (*call_back_signal)(int signum);
	
	public:

		PosixPIT(uint16_t ms, void (*call_back_signal)(int signum)) : ms(ms), call_back_signal(call_back_signal)
		{
			// 配置定时器事件，使用线程回调通知
			// sev.sigev_notify = SIGEV_THREAD;

			sev.sigev_notify = SIGEV_SIGNAL;
			sev.sigev_signo = SIGUSR1;

			// 注册信号处理函数
			signal(SIGUSR1, call_back_signal);


			// 创建定时器
			if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
				perror("timer_create");
				return;
			}

			// 设置定时器的初始值和间隔时间
			its.it_value.tv_sec = ms / 1000;
			its.it_value.tv_nsec = (ms % 1000) * 1000000;
			its.it_interval.tv_sec = ms / 1000;
			its.it_interval.tv_nsec = (ms % 1000) * 1000000;

			// 启动定时器
			if (timer_settime(timerid, 0, &its, NULL) == -1) {
				perror("timer_settime");
				return;
			}

			printf("使用 SIGEV_SIGNAL 方式的定时器已启动，等待信号...\n");
			return;
		}

		PosixPIT(uint16_t ms, void (*call_back_func)(union sigval sv)) : ms(ms), call_back_func(call_back_func)
		{
			// 配置定时器事件，使用线程回调通知
			sev.sigev_notify = SIGEV_THREAD;
			sev.sigev_notify_function = call_back_func;
			sev.sigev_notify_attributes = NULL;
			sev.sigev_value.sival_ptr = &timerid;

			// 创建定时器
			if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
				perror("timer_create");
				return;
			}

			// 设置定时器的初始值和间隔时间
			its.it_value.tv_sec = ms / 1000;
			its.it_value.tv_nsec = (ms % 1000) * 1000000;
			its.it_interval.tv_sec = ms / 1000;
			its.it_interval.tv_nsec = (ms % 1000) * 1000000;

			// 启动定时器
			if (timer_settime(timerid, 0, &its, NULL) == -1) {
				perror("timer_settime");
				return;
			}

			std::cout << "定时器已启动，等待触发..." << std::endl;
			return;
		}
	
	
		~PosixPIT() {
			// 删除定时器
			if (timer_delete(timerid) == -1) {
				perror("timer_delete");
			}
		}
};



class PosixTimer {
	private:
		timer_t timerid;
		struct sigevent sev;
		struct itimerspec its;
	
	public:
		PosixTimer(uint16_t ms, void (*call_back_func)(union sigval sv)) {
			// 配置定时器到期时的行为
			sev.sigev_notify = SIGEV_THREAD;
			sev.sigev_notify_function = call_back_func;
			sev.sigev_value.sival_int = 123;  // 传递给回调函数的参数
	
			// 创建定时器
			if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
				perror("timer_create");
				return;
			}
	
			// 将毫秒转换为秒和纳秒
			its.it_value.tv_sec = ms / 1000;
			its.it_value.tv_nsec = (ms % 1000) * 1000000;
			its.it_interval.tv_sec = 0;
			its.it_interval.tv_nsec = 0;
	
			// 设置定时器时间
			if (timer_settime(timerid, 0, &its, NULL) == -1) {
				perror("timer_settime");
				timer_delete(timerid);
				return;
			}
	
			std::cout << "定时器已启动，等待超时事件..." << std::endl;
		}
	
		~PosixTimer() {
			// 删除定时器
			timer_delete(timerid);
		}
	};
	

#endif
