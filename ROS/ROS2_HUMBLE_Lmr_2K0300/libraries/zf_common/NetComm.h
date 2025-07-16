/**
 * project : cvpass
 * file    : NetComm.h
 * Copytight <c> handle 2025 all rights reserved.
**/
/**
* NetComm模块
* 主要工作是为其他服务提供稳定可靠的简单tcp/ip服务，
* 通过宏选择平台，使用netcomm类对连接进行管理
* 类没有线程安全，类有针对wsa的初始化。
**/
#pragma once 
#ifndef NETCOMM
#define NETCOMM
//通过宏选择平台
#define LINUX
//#define WINDOWS

#ifdef LINUX
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//适配用定义
typedef int SOCKET;
#define closesocket close
#define SOCK_SEND(sock,data,len) ::send(sock,data,len,MSG_NOSIGNAL)
#define SOCK_RECV(sock,data,len) ::recv(sock,data,len,MSG_NOSIGNAL)
#endif

#ifdef WINDOWS
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#pragma comment(lib,"opencv_world4100d")
#pragma comment(lib,"winmm")
#pragma comment(lib,"ws2_32")
//适配用定义
#define SHUT_RDWR SD_BOTH
#define SOCK_SEND(sock,data,len) ::send(sock,(char*)data,len,0)
#define SOCK_RECV(sock,data,len) ::recv(sock,(char*)data,len,0)
#endif

//错误处理宏
#ifndef SOCK_ERROR
#define SOCK_ERROR(qt) {std::cout<<"[ERROR][NetComm]:"<<qt<<std::endl;return -1;}
#define SOCK_INFO(qt) {std::cout<<"[NetComm]:"<<qt<<std::endl;}
#endif 

/********
* 类     NetComm
* 作用   提供简单tcp连接
********/
class NetComm {
public:
    /********
    * 函数   NetComm
    * 作用   构造函数
    * 参数   无
    * 返回值 无
    ********/
    NetComm();
    /********
    * 函数   serv
    * 作用   进入服务器模式，用于启动服务器
    * 参数   1-端口
    * 返回值 0 成功,其他 失败
    ********/
    int serv(int port);
    /********
    * 函数   dispach
    * 作用   用于接受客户端连接请求，会进入阻塞状态
    * 参数   1-最大可等待客户端数
    * 返回值 0 成功,其他 失败
    ********/
    int dispach(int maxwait);
    /********
    * 函数   conn
    * 作用   进入客户端模式，用于连接服务器
    * 参数   1-ip地址(字符串),2-连接端口
    * 返回值 0 成功,其他 失败
    ********/
    int conn(const char* ipaddr, int port);
    /********
    * 函数   send
    * 作用   无论处于什么模式，send将消息发送出去
    * 参数   1-发送数据指针,2-发送数据长度
    * 返回值 默认返回发送数据长度,-1 失败
    ********/
    int send(void* pdata, const int& length);
    /********
    * 函数   recv
    * 作用   无论处于什么模式，recv接收数据(不堵塞)
    * 参数   1-接收缓冲区,2-最大接收长度
    * 返回值 默认返回接受数据长度,-1 失败
    ********/
    int recv(void* pbuf, const int& buflength);

    /********
    * 函数   sendpack
    * 作用   通过包模式发送数据，可以解决包粘连问题.
    *        (需要和recvpack和recvpacklength使用)
    * 参数   1-发送缓冲区,2-发送数据长度
    * 返回值 0成功,-1 失败
    ********/
    int sendpack(void* pdata, const int& length);
    /********
    * 函数   recvpacklength
    * 作用   通过包模式获取下一个包的长度,以创建缓冲内存
    *        (该函数应该在使用recvpack之前使用)
    * 参数   无
    * 返回值 默认返回数据包长度,-1 失败
    ********/
    int recvpacklength();
    /********
    * 函数   recvpacklength
    * 作用   通过包模式获取数据包，保证获取到完整的数据包
    *        (该函数应该在使用recvpacklength之后使用)
    * 参数   1-接收缓冲区,2-最大接收长度
    * 返回值 0成功,-1 失败
    ********/
    int recvpack(void* pbuf, const int& length);
    /********
    * 函数   shutdown
    * 作用   优雅的结束连接，作为客户端角色则断开与服务器的连接。
    作为服务器则断开与当前客户端的连接，之后可以使用dispach获取下一连接
    * 参数   无
    * 返回值 0 成功,其他 失败
    ********/
    int shutdown();
    /********
    * 函数   ~NetComm
    * 作用   析构函数
    * 参数   无
    * 返回值 无
    ********/
    ~NetComm();

private:
    /********
    * 变量   Actor
    * 作用   标记当前处于的角色
    * 取值   0-未连接,1-服务器角色,2-客户端角色
    ********/
    int Actor;
    /********
    * 变量   sockin
    * 作用   socket套接字填充变量
    * 取值   *
    ********/
    sockaddr_in sockin;
    /********
    * 变量   sock
    * 作用   主操作套接字，recv和send的对象
    * 取值   0 无效
    ********/
    SOCKET sock;
    /********
    * 变量   servsock
    * 作用   服务器套接字，服务器状态的对象
    * 取值   0 无效
    ********/
    SOCKET servsock;

#ifdef WINDOWS
    //windows内部处理
    WSADATA wsadata;
    static int countofwsa;
#endif
    /********
    * 函数   reset
    * 作用   强行结束服务端或客户端链接并重置类
    * 参数   无
    * 返回值 0-成功
    ********/
    int reset();
};

#endif