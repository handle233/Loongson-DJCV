#ifndef _zf_driver_tcp_client_h
#define _zf_driver_tcp_client_h


#include "zf_common_typedef.h"

int8 tcp_client_init(const char *ip_addr, uint32 port);
int tcp_init(const char *ip_addr, uint32 port);
int32 tcp_client_send_data(int socket, const uint8 *buff, uint32 length);
int32 tcp_client_read_data(int socket, uint8 *buff, uint32 length);


#endif
