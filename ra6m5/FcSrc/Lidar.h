#include "SysConfig.h"

#define LIDAR_DATA_LEN 8

#define LIDAR_FRAME_HEAD 0xAA
#define LIDAR_FRAME_END 0x55

#define LIDAR_SET_ANGLE 24.82f

typedef enum
{
    LIDAR_WAIT_HEAD,  // 等待帧头
    LIDAR_WAIT_LEN,   // 等待长度
    LIDAR_WAIT_DATA,  // 等待数据
    LIDAR_WAIT_CHECK, // 等待校验和
    LIDAR_WAIT_END    // 等待帧尾
} lidar_rx_state_t;

typedef struct
{
    u8 head;                 // 帧头
    u8 length;               // 数据长度
    u8 data[LIDAR_DATA_LEN]; // 数据
    u8 checksum;             // 校验和
    u8 end;                  // 帧尾 0x55
} lidar_frame_t;

extern u8 lidar_start_flag;

void Lidar_Data_Receive(u8 data);
