/**
 * project : cvpass
 * file    : NetComm.cpp
 * Copytight <c> handle 2025 all rights reserved.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#define LINUX
#include "NetComm.h"

#ifdef WINDOWS
int NetComm::countofwsa = 0;
#endif

//NetComm
NetComm::NetComm() :Actor(0)
{
    //閿熸枻鎷峰閿熸枻鎷穡sa閿熸枻鎷穡indows涓撻敓鏂ゆ嫹
#ifdef WINDOWS
    wsadata = { 0 };
    if (countofwsa < 1) {
        WSAStartup(MAKEWORD(2, 2), &wsadata);
    }
    countofwsa++;
#endif
    //閿熸枻鎷峰閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
    sockin = { 0 };
    sock = servsock = 0;
}

//serv
int NetComm::serv(int port)
{
    //閿熻剼绛规嫹閿熺獤鎾呮嫹閿熸枻鎷烽敓鏂ゆ嫹閿熺即锟�
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //閿熸枻鎷峰閿熸枻鎷穝ockaddr閿熺粨鏋勯敓鏂ゆ嫹
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    sockin.sin_addr.s_addr = htonl(INADDR_ANY);
    //閿熸枻鎷峰閿熸枻鎷烽敓闃舵枻鎷烽敓鏂ゆ嫹
    servsock = socket(AF_INET, SOCK_STREAM, 0);
    if (servsock == -1) {
        SOCK_ERROR("Create serv socket error.");
    }
    //閿熸枻鎷蜂负閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓闃舵枻鎷烽敓鏂ゆ嫹
    if (bind(servsock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1) {
        SOCK_ERROR("Bind socket error.");
    }

    SOCK_INFO("Successfully setup server.");
    //閿熸枻鎷烽敓鐭枻鎷疯壊
    Actor = 1;
    return 0;
}

//dispach
int NetComm::dispach(int maxwait)
{
    //閿熻剼绛规嫹閿熺獤鏂ゆ嫹閿熸枻鎷烽敓缂搭倲鍒猴拷
    if (Actor != 1) {
        SOCK_ERROR("Current Actor dismatch function");
    }
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
    listen(servsock, maxwait);
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    SOCKET client = accept(servsock, nullptr, nullptr);
    if (client == -1)
        SOCK_ERROR("Error when accepting connection");
    //閿熸枻鎷烽敓绉哥鎷烽敓杈冨嚖鎷烽敓閰典紮鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎺ワ綇鎷烽敓鏂ゆ嫹閿熺嫛闈╂嫹閿熸枻鎷烽敓鏂ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    sock = client;

    SOCK_INFO("Successfully accept a connection.");
    return 0;
}

int NetComm::conn(const char* ipaddr, int port)
{
    //閿熻剼绛规嫹閿熺獤鏂ゆ嫹閿熸枻鎷烽敓缂搭倲鍒猴拷
    if (Actor) {
        SOCK_ERROR("Actor has been set.");
    }
    //閿熸枻鎷烽敓鏂ゆ嫹sockaddr閿熺粨鏋勯敓鏂ゆ嫹
    memset(&sockin, 0, sizeof(sockaddr_in));
    sockin.sin_family = AF_INET;
    sockin.sin_port = htons(port);
    //windows閿熸枻鎷穕inux閿熸枻鎷穒p閿熸枻鎷峰潃閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熼叺锟�
#ifdef WINDOWS
    inet_pton(AF_INET, ipaddr, &sockin.sin_addr.s_addr);
#endif
#ifdef LINUX
    sockin.sin_addr.s_addr = inet_addr(ipaddr);
#endif
    //閿熸枻鎷烽敓鏂ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        SOCK_ERROR("Create socket error.");
    }
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎺ュ嚖鎷烽敓鏂ゆ嫹閿熸枻鎷�
    if (connect(sock, (sockaddr*)&sockin, sizeof(sockaddr_in)) == -1)
        SOCK_ERROR("Connect failed.");

    SOCK_INFO("Connection established.");
    //閿熸枻鎷烽敓鐭枻鎷疯壊
    Actor = 2;
    return 0;
}

//send
int NetComm::send(void* pdata, const int& length)
{
    //閿熻剼绛规嫹閿熻鍑ゆ嫹sock
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    int r = SOCK_SEND(sock, pdata, length);

    //閿熸枻鎷烽敓鏂ゆ嫹閿熷彨璁规嫹
    if (r == -1 && errno != 0) {
        //閿熸枻鎷烽敓鑺傝鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熻鎲嬫嫹鍜忛敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }
    return r;
}

//recv
int NetComm::recv(void* pbuf, const int& buflength)
{
    //閿熻剼绛规嫹閿熻鍑ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock == 0) {
        SOCK_ERROR("Connection is unestablishing.");
    }

    //閿熸枻鎷峰彇閿熸枻鎷烽敓鏂ゆ嫹
    int r = SOCK_RECV(sock, pbuf, buflength);

    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
    if (r == -1 && errno != 0) {
        //閿熸枻鎷烽敓鑺傝鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熻鎲嬫嫹閿熸枻鎷峰嵏閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }
    return r;
}

//sendpack
int NetComm::sendpack(void* pdata, const int& length) {
    //閿熻剼绛规嫹閿熻鍑ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎹风鎷烽敓鏂ゆ嫹
    int r = SOCK_SEND(sock, &length, sizeof(int));
    if (r == -1 && errno != 0) {
        goto sendpackerror;
    }
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    r = SOCK_SEND(sock, pdata, length);

    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
    if (r == -1 && errno != 0) {
    sendpackerror:
        //閿熸枻鎷烽敓鑺傝鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熻鎲嬫嫹閿熸枻鎷峰嵏閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        shutdown();
        SOCK_ERROR("Transfer error with code :" << errno);
    }

    return 0;
}

int NetComm::recvpacklength()
{
    //閿熻剼绛规嫹閿熻鍑ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");
    //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎹风鎷烽敓鏂ゆ嫹
    int recvlength;
    int r = SOCK_RECV(sock, &recvlength, sizeof(int));
    if (r == -1 && errno != 0) {
        //閿熸枻鎷烽敓鑺傝鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熻鎲嬫嫹閿熸枻鎷峰嵏閿熸枻鎷烽敓鏂ゆ嫹閿燂拷
        shutdown();
        SOCK_ERROR("Receive error with code :" << errno);
    }

    return recvlength;
}

// recvpack
int NetComm::recvpack(void* pbuf, const int& length) {
    //閿熻剼绛规嫹閿熻鍑ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock == 0)
        SOCK_ERROR("Connection is unestablishing");

    long recvlength = 0;
    int errcount = 0;
    char* precv = (char*)pbuf;

    //閿熸枻鎷烽敓鏂ゆ嫹閿熼ズ鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
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
    //閿熸枻鎷烽敓鎹锋枻鎷疯壊閿熸枻鎷峰悓閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰悓閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    if (Actor == 1) {
        //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鑹�
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        sock = 0;
        //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷穝hutdown涔嬮敓鏂ゆ嫹閿熺即顐嫹閿熸枻鎷锋磳顒婃嫹閿熸枻鎷疯秺閿熸枻鎷烽敓鏂ゆ嫹纰岄敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
        SOCK_INFO("server:endup the connection.");
    }
    else if (Actor == 2) {
        //閿熼叺浼欐嫹閿熷壙鏂ゆ嫹鑹�
        if (sock) {
            ::shutdown(sock, SHUT_RDWR);
            closesocket(sock);
        }
        //閿熸枻鎷蜂綅閿熸枻鎷烽敓琛楁唻鎷烽敓鏂ゆ嫹
        sock = 0;
        Actor = 0;
        SOCK_INFO("client:endup the connection.");
    }
    return 0;
}

//~NetComm
NetComm::~NetComm()
{
    //閿熸枻鎷烽敓绲痵a閿熸枻鎷烽敓鏂ゆ嫹
#ifdef WINDOWS
    countofwsa--;
    if (countofwsa < 1) {
        WSACleanup();
    }
#endif
    //閿熸枻鎷蜂綅鍏ㄩ敓鏂ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    sock = servsock = 0;
}

//reset
int NetComm::reset()
{
    //閿熸枻鎷蜂綅鍏ㄩ敓鏂ゆ嫹閿熼樁鏂ゆ嫹閿熸枻鎷�
    if (sock)
        closesocket(sock);
    if (servsock)
        closesocket(servsock);
    //閿熸枻鎷蜂綅鍏ㄩ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    sock = servsock = 0;
    Actor = 0;

    SOCK_INFO("Active connection reset.");
    return 0;
}
