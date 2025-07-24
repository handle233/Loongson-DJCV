/**
 * project : cvpass
 * file    : NetComm.cpp
 * Copytight <c> handle 2025 all rights reserved.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#define WINDOWS
#include "NetComm.h"

#ifdef WINDOWS
int countofwsa = 0;
WSADATA wsadata;
#endif

//NetComm
NetComm::NetComm() :Actor(0)
{
    //初始化wsa，windows专门
#ifdef WINDOWS
    wsadata = { 0 };
    if (countofwsa < 1) {
        WSAStartup(MAKEWORD(2, 2), &wsadata);
    }
    countofwsa++;
#endif
    //初始化变量
    sockin = { 0 };
    sock = servsock = 0;
}

//serv
int NetComm::serv(int port)
{
    //排除已经进入角色
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //初始化sockaddr结构体
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    sockin.sin_addr.s_addr = htonl(INADDR_ANY);
    //初始化套接字
    servsock = socket(AF_INET, SOCK_STREAM, 0);
    if (servsock == -1) {
        SOCK_ERROR("Create serv socket error.");
    }
    //绑定为服务器套接字
    if (bind(servsock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1) {
        SOCK_ERROR("Bind socket error.");
    }

    SOCK_INFO("Successfully setup server.");
    //设置角色
    Actor = 1;
    return 0;
}

//dispach
int NetComm::dispach(int maxwait)
{
    //排除已进入角色状态
    if (Actor != 1) {
        SOCK_ERROR("Current Actor dismatch function");
    }
    //进入监听
    listen(servsock, maxwait);
    //接受连接
    SOCKET client = accept(servsock, nullptr, nullptr);
    if (client == -1)
        SOCK_ERROR("Error when accepting connection");
    //接收到合法客户端连接，复制给操作套接字
    sock = client;

    SOCK_INFO("Successfully accept a connection.");
    return 0;
}

//connect
int NetComm::conn(const char* ipaddr, int port)
{
    //排除已进入角色状态
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //设置sockaddr结构体
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    //windows和linux的ip地址设置情况不同
#ifdef WINDOWS
    inet_pton(AF_INET, ipaddr, &sockin.sin_addr.s_addr);
#endif
#ifdef LINUX
    sockin.sin_addr.s_addr = inet_addr(ipaddr);
#endif
    //设置套接字
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        SOCK_ERROR("Create socket error.");
    }
    //尝试连接服务器
    if (connect(sock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1)
        SOCK_ERROR("Connect failed.");

    SOCK_INFO("Connection established.");
    //设置角色
    Actor = 2;
    return 0;
}

//send
int NetComm::send(void* pdata, const int& length)
{
    //排除非法sock
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //发送数据
    int r = SOCK_SEND(sock, pdata, length);

    //错误判断
    if (r == -1 && errno != 0) {
        //对于不可逆错误直接结束连接
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }
    return r;
}

//recv
int NetComm::recv(void* pbuf, const int& buflength)
{
    //排除非法套接字
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //收取数据
    int r = SOCK_RECV(sock, pbuf, buflength);

    //错误处理
    if (r == -1 && errno != 0) {
        //对于不可逆错误，直接中断连接
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }
    return r;
}

//sendpack
int NetComm::sendpack(void* pdata, const int& length) {
    //排除非法套接字
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //发送数据长度
    int r = SOCK_SEND(sock, &length, sizeof(int));
    if (r == -1 && errno != 0) {
        goto sendpackerror;
    }
    //发送数据
    r = SOCK_SEND(sock, pdata, length);

    //错误处理
    if (r == -1 && errno != 0) {
    sendpackerror:
        //对于不可逆错误，直接中断连接
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }

    return 0;
}
int NetComm::recvpacklength()
{
    //排除非法套接字
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //发送数据长度
    int recvlength;
    int r = SOCK_RECV(sock, &recvlength, sizeof(int));
    if (r == -1 && errno != 0) {
        //对于不可逆错误，直接中断连接
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }

    return recvlength;
}

// recvpack
int NetComm::recvpack(void* pbuf, const int& length) {
    //排除非法套接字
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");

    long recvlength = 0;
    int errcount = 0;
    char* precv = (char*)pbuf;

    //按长度接收数据
    while (recvlength < length) {
        int r = SOCK_RECV(sock, precv, length - recvlength);
        if (r < 0) {
            SOCK_INFO("Receive error with error code" << errno);
            errcount++;
            if (errcount > 3) {
                shutdown();
                SOCK_ERROR("Receive error with 3 retries.");
            }
            continue;
        }
        recvlength += r;
        precv += r;
    }

    return 0;
}

//shutdown
int NetComm::shutdown()
{
    //根据角色不同决定不同结束内容
    if (Actor == 1) {
        //服务器角色
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        sock = 0;
        //服务器shutdown之后角色不变，可以接收新的连接请求
        SOCK_INFO("server:endup the connection.");
    }
    else if (Actor == 2) {
        //客户端角色
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        //复位各种变量
        sock = 0;
        Actor = 0;
        SOCK_INFO("client:endup the connection.");
    }
    return 0;
}

//~NetComm
NetComm::~NetComm()
{
    //解除wsa加载
#ifdef WINDOWS
    countofwsa--;
    if (countofwsa < 1) {
        WSACleanup();
    }
#endif
    //复位全部套接字
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    sock = servsock = 0;
}

//reset
int NetComm::reset()
{
    //复位全部套接字
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    //复位全部变量
    sock = servsock = 0;
    Actor = 0;

    SOCK_INFO("Active connection reset.");
    return 0;
}

