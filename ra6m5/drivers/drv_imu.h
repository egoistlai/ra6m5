#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* 功能码定义 */
#define IMU_FUNC_RAW_ACCEL 0x04
#define IMU_FUNC_RAW_GYRO 0x0A
#define IMU_FUNC_RAW_MAG 0x10
#define IMU_FUNC_QUAT 0x16
#define IMU_FUNC_EULER 0x26

/* IMU 数据结构体 */
typedef struct
{
    float accel[3]; // 加速度 (g)
    float gyro[3];  // 角速度 (deg/s)
    float mag[3];   // 磁场
    float euler[3]; // 欧拉角: 0=Roll, 1=Pitch, 2=Yaw (度)
    bool is_updated;
} imu_data_t;

/* 暴露给外部线程（特别是 GUI 线程）的全局共享变量 */
extern volatile imu_data_t g_imu_data;

/* 解析状态机入口（外部每次收到一个字节就调用一次） */
void drv_imu_parse_byte(uint8_t byte);

#endif // DRV_IMU_H