#ifndef _Radar_data_protocol_h_
#define _Radar_data_protocol_h_

#include "zf_common_typedef.h"

#define POINT_PER_PACK 12
#define HEADER 0x54

typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
}LiDARFrameTypeDef;


uint8_t CalCRC8(uint8_t *p, uint8_t len);

#endif
