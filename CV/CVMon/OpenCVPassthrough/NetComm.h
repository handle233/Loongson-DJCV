/**
 * project : cvpass
 * file    : NetComm.h
 * Copytight <c> handle 2025 all rights reserved.
**/
/**
* NetCommģ��
* ��Ҫ������Ϊ���������ṩ�ȶ��ɿ��ļ�tcp/ip����
* ͨ����ѡ��ƽ̨��ʹ��netcomm������ӽ��й���
* ��û���̰߳�ȫ���������wsa�ĳ�ʼ����
**/

//ͨ����ѡ��ƽ̨
//#define LINUX
//#define WINDOWS

#ifdef LINUX
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//�����ö���
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
//�����ö���
#define SHUT_RDWR SD_BOTH
#define SOCK_SEND(sock,data,len) ::send(sock,(char*)data,len,0)
#define SOCK_RECV(sock,data,len) ::recv(sock,(char*)data,len,0)
#endif

//�������
#ifndef SOCK_ERROR
#define SOCK_ERROR(qt) {std::cout<<"[ERROR][NetComm]:"<<qt<<std::endl;return -1;}
#define SOCK_INFO(qt) {std::cout<<"[NetComm]:"<<qt<<std::endl;}
#endif 

/********
* ��     NetComm
* ����   �ṩ��tcp����
********/
class NetComm {
public:
    /********
    * ����   NetComm
    * ����   ���캯��
    * ����   ��
    * ����ֵ ��
    ********/
    NetComm();
    /********
    * ����   serv
    * ����   ���������ģʽ����������������
    * ����   1-�˿�
    * ����ֵ 0 �ɹ�,���� ʧ��
    ********/
    int serv(int port);
    /********
    * ����   dispach
    * ����   ���ڽ��ܿͻ����������󣬻��������״̬
    * ����   1-���ɵȴ��ͻ�����
    * ����ֵ 0 �ɹ�,���� ʧ��
    ********/
    int dispach(int maxwait);
    /********
    * ����   conn
    * ����   ����ͻ���ģʽ���������ӷ�����
    * ����   1-ip��ַ(�ַ���),2-���Ӷ˿�
    * ����ֵ 0 �ɹ�,���� ʧ��
    ********/
    int conn(const char* ipaddr, int port);
    /********
    * ����   send
    * ����   ���۴���ʲôģʽ��send����Ϣ���ͳ�ȥ
    * ����   1-��������ָ��,2-�������ݳ���
    * ����ֵ Ĭ�Ϸ��ط������ݳ���,-1 ʧ��
    ********/
    int send(void* pdata, const int& length);
    /********
    * ����   recv
    * ����   ���۴���ʲôģʽ��recv��������(������)
    * ����   1-���ջ�����,2-�����ճ���
    * ����ֵ Ĭ�Ϸ��ؽ������ݳ���,-1 ʧ��
    ********/
    int recv(void* pbuf, const int& buflength);

    /********
    * ����   sendpack
    * ����   ͨ����ģʽ�������ݣ����Խ����ճ������.
    *        (��Ҫ��recvpack��recvpacklengthʹ��)
    * ����   1-���ͻ�����,2-�������ݳ���
    * ����ֵ 0�ɹ�,-1 ʧ��
    ********/
    int sendpack(void* pdata, const int& length);
    /********
    * ����   recvpacklength
    * ����   ͨ����ģʽ��ȡ��һ�����ĳ���,�Դ��������ڴ�
    *        (�ú���Ӧ����ʹ��recvpack֮ǰʹ��)
    * ����   ��
    * ����ֵ Ĭ�Ϸ������ݰ�����,-1 ʧ��
    ********/
    int recvpacklength();
    /********
    * ����   recvpacklength
    * ����   ͨ����ģʽ��ȡ���ݰ�����֤��ȡ�����������ݰ�
    *        (�ú���Ӧ����ʹ��recvpacklength֮��ʹ��)
    * ����   1-���ջ�����,2-�����ճ���
    * ����ֵ 0�ɹ�,-1 ʧ��
    ********/
    int recvpack(void* pbuf, const int& length);
    /********
    * ����   shutdown
    * ����   ���ŵĽ������ӣ���Ϊ�ͻ��˽�ɫ��Ͽ�������������ӡ�
    ��Ϊ��������Ͽ��뵱ǰ�ͻ��˵����ӣ�֮�����ʹ��dispach��ȡ��һ����
    * ����   ��
    * ����ֵ 0 �ɹ�,���� ʧ��
    ********/
    int shutdown();
    /********
    * ����   ~NetComm
    * ����   ��������
    * ����   ��
    * ����ֵ ��
    ********/
    ~NetComm();

private:
    /********
    * ����   Actor
    * ����   ��ǵ�ǰ���ڵĽ�ɫ
    * ȡֵ   0-δ����,1-��������ɫ,2-�ͻ��˽�ɫ
    ********/
    int Actor;
    /********
    * ����   sockin
    * ����   socket�׽���������
    * ȡֵ   *
    ********/
    sockaddr_in sockin;
    /********
    * ����   sock
    * ����   �������׽��֣�recv��send�Ķ���
    * ȡֵ   0 ��Ч
    ********/
    SOCKET sock;
    /********
    * ����   servsock
    * ����   �������׽��֣�������״̬�Ķ���
    * ȡֵ   0 ��Ч
    ********/
    SOCKET servsock;

    /********
    * ����   reset
    * ����   ǿ�н�������˻�ͻ������Ӳ�������
    * ����   ��
    * ����ֵ 0-�ɹ�
    ********/
    int reset();
};

/********
* ��     UdpComm
* ����   �ṩ��udp����
********/
class UdpComm {
public:
    UdpComm();

    int serv(int port);
    int conn(const char* addr,int port);

    int send(void* pdata, const int& length);
    int recv(void* pbuf, const int& buflength);
private:
    //sockaddr_in 
};