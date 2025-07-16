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
    //��ʼ��wsa��windowsר��
#ifdef WINDOWS
    wsadata = { 0 };
    if (countofwsa < 1) {
        WSAStartup(MAKEWORD(2, 2), &wsadata);
    }
    countofwsa++;
#endif
    //��ʼ������
    sockin = { 0 };
    sock = servsock = 0;
}

//serv
int NetComm::serv(int port)
{
    //�ų��Ѿ������ɫ
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //��ʼ��sockaddr�ṹ��
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    sockin.sin_addr.s_addr = htonl(INADDR_ANY);
    //��ʼ���׽���
    servsock = socket(AF_INET, SOCK_STREAM, 0);
    if (servsock == -1) {
        SOCK_ERROR("Create serv socket error.");
    }
    //��Ϊ�������׽���
    if (bind(servsock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1) {
        SOCK_ERROR("Bind socket error.");
    }

    SOCK_INFO("Successfully setup server.");
    //���ý�ɫ
    Actor = 1;
    return 0;
}

//dispach
int NetComm::dispach(int maxwait)
{
    //�ų��ѽ����ɫ״̬
    if (Actor != 1) {
        SOCK_ERROR("Current Actor dismatch function");
    }
    //�������
    listen(servsock, maxwait);
    //��������
    SOCKET client = accept(servsock, nullptr, nullptr);
    if (client == -1)
        SOCK_ERROR("Error when accepting connection");
    //���յ��Ϸ��ͻ������ӣ����Ƹ������׽���
    sock = client;

    SOCK_INFO("Successfully accept a connection.");
    return 0;
}

int NetComm::conn(const char* ipaddr, int port)
{
    //�ų��ѽ����ɫ״̬
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //����sockaddr�ṹ��
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    //windows��linux��ip��ַ���������ͬ
#ifdef WINDOWS
    inet_pton(AF_INET, ipaddr, &sockin.sin_addr.s_addr);
#endif
#ifdef LINUX
    sockin.sin_addr.s_addr = inet_addr(ipaddr);
#endif
    //�����׽���
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        SOCK_ERROR("Create socket error.");
    }
    //�������ӷ�����
    if (connect(sock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1)
        SOCK_ERROR("Connect failed.");

    SOCK_INFO("Connection established.");
    //���ý�ɫ
    Actor = 2;
    return 0;
}

//send
int NetComm::send(void* pdata, const int& length)
{
    //�ų��Ƿ�sock
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //��������
    int r = SOCK_SEND(sock, pdata, length);

    //�����ж�
    if (r == -1 && errno != 0) {
        //���ڲ��������ֱ�ӽ�������
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }
    return r;
}

//recv
int NetComm::recv(void* pbuf, const int& buflength)
{
    //�ų��Ƿ��׽���
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //��ȡ����
    int r = SOCK_RECV(sock, pbuf, buflength);

    //������
    if (r == -1 && errno != 0) {
        //���ڲ��������ֱ���ж�����
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }
    return r;
}

//sendpack
int NetComm::sendpack(void* pdata, const int& length) {
    //�ų��Ƿ��׽���
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //�������ݳ���
    int r = SOCK_SEND(sock, &length, sizeof(int));
    if (r == -1 && errno != 0) {
        goto sendpackerror;
    }
    //��������
    r = SOCK_SEND(sock, pdata, length);

    //������
    if (r == -1 && errno != 0) {
    sendpackerror:
        //���ڲ��������ֱ���ж�����
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }

    return 0;
}

int NetComm::recvpacklength()
{
    //�ų��Ƿ��׽���
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //�������ݳ���
    int recvlength;
    int r = SOCK_RECV(sock, &recvlength, sizeof(int));
    if (r == -1 && errno != 0) {
        //���ڲ��������ֱ���ж�����
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }

    return recvlength;
}

// recvpack
int NetComm::recvpack(void* pbuf, const int& length) {
    //�ų��Ƿ��׽���
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");

    long recvlength = 0;
    int errcount = 0;
    char* precv = (char*)pbuf;

    //�����Ƚ�������
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
    //���ݽ�ɫ��ͬ������ͬ��������
    if (Actor == 1) {
        //��������ɫ
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        sock = 0;
        //������shutdown֮���ɫ���䣬���Խ����µ���������
        SOCK_INFO("server:endup the connection.");
    }
    else if (Actor == 2) {
        //�ͻ��˽�ɫ
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        //��λ���ֱ���
        sock = 0;
        Actor = 0;
        SOCK_INFO("client:endup the connection.");
    }
    return 0;
}

//~NetComm
NetComm::~NetComm()
{
    //���wsa����
#ifdef WINDOWS
    countofwsa--;
    if (countofwsa < 1) {
        WSACleanup();
    }
#endif
    //��λȫ���׽���
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    sock = servsock = 0;
}

//reset
int NetComm::reset()
{
    //��λȫ���׽���
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    //��λȫ������
    sock = servsock = 0;
    Actor = 0;

    SOCK_INFO("Active connection reset.");
    return 0;
}
