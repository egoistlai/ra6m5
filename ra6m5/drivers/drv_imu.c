#include "drv_imu.h"
#include <string.h>
#include <math.h>

volatile imu_data_t g_imu_data = {0};

/* 协议解析：将小端 16 位合成并转为 float */
static void drv_imu_decode(const uint8_t *buf)
{
    uint8_t func = buf[3];

    // 解析欧拉角 (功能码 0x26)
    if (func == IMU_FUNC_EULER)
    {
        float roll_rad, pitch_rad, yaw_rad;

        // 【核心修复】：使用 memcpy 安全地把 4 个字节拷入 float 变量，完美解决字节序和内存对齐问题
        memcpy(&roll_rad, &buf[4], 4);
        memcpy(&pitch_rad, &buf[8], 4);
        memcpy(&yaw_rad, &buf[12], 4);

        // 协议传过来的是弧度(rad)，为了 LVGL 屏幕显示直观，我们将其转换为角度(deg)
        // 1 弧度 ≈ 57.2957795 度
        g_imu_data.euler[0] = roll_rad * (180.0f / 3.14159265f);
        g_imu_data.euler[1] = pitch_rad * (180.0f / 3.14159265f);
        g_imu_data.euler[2] = yaw_rad * (180.0f / 3.14159265f);

        g_imu_data.is_updated = true;
    }
    // 解析加速度、角速度 (功能码 0x04)
    else if (func == IMU_FUNC_RAW_ACCEL)
    {
        // 1. 解析加速度 (单位: g)
        int16_t ax = (int16_t)(buf[4] | (buf[5] << 8));
        int16_t ay = (int16_t)(buf[6] | (buf[7] << 8));
        int16_t az = (int16_t)(buf[8] | (buf[9] << 8));
        g_imu_data.accel[0] = ax * (16.0f / 32767.0f);
        g_imu_data.accel[1] = ay * (16.0f / 32767.0f);
        g_imu_data.accel[2] = az * (16.0f / 32767.0f);

        // 2. 解析角速度 (协议原生单位为 rad/s，我们将其保留为 deg/s 方便屏幕查看)
        int16_t gx = (int16_t)(buf[10] | (buf[11] << 8));
        int16_t gy = (int16_t)(buf[12] | (buf[13] << 8));
        int16_t gz = (int16_t)(buf[14] | (buf[15] << 8));
        g_imu_data.gyro[0] = gx * (2000.0f / 32767.0f);
        g_imu_data.gyro[1] = gy * (2000.0f / 32767.0f);
        g_imu_data.gyro[2] = gz * (2000.0f / 32767.0f);

        // 3. 解析磁力计 (由于是 6 轴 IMU，这里的 raw 数据通常是 0)
        int16_t mx = (int16_t)(buf[16] | (buf[17] << 8));
        int16_t my = (int16_t)(buf[18] | (buf[19] << 8));
        int16_t mz = (int16_t)(buf[20] | (buf[21] << 8));
        g_imu_data.mag[0] = mx * (800.0f / 32767.0f);
        g_imu_data.mag[1] = my * (800.0f / 32767.0f);
        g_imu_data.mag[2] = mz * (800.0f / 32767.0f);
    }
}

/* 高效字节流状态机，直接替代了原本庞大的环形缓冲区 */
void drv_imu_parse_byte(uint8_t byte)
{
    static uint8_t rx_buf[64];
    static uint8_t rx_cnt = 0;
    static uint8_t expected_len = 0;

    // 状态 1：寻找包头 0x7E
    if (rx_cnt == 0 && byte != 0x7E)
        return;
    // 状态 2：寻找包头 0x23
    if (rx_cnt == 1 && byte != 0x23)
    {
        rx_cnt = 0;
        return;
    }

    rx_buf[rx_cnt++] = byte;

    // 状态 3：提取帧长
    if (rx_cnt == 3)
    {
        expected_len = byte;
        if (expected_len > 64 || expected_len < 5)
        { // 长度不合法，复位
            rx_cnt = 0;
            return;
        }
    }

    // 状态 4：接收完整帧并校验
    if (rx_cnt >= 3 && rx_cnt == expected_len)
    {
        uint8_t sum = 0;
        // 校验位计算：从包头累加到校验位前
        for (int i = 0; i < expected_len - 1; i++)
        {
            sum += rx_buf[i];
        }

        if (sum == rx_buf[expected_len - 1])
        { // 校验通过
            drv_imu_decode(rx_buf);
        }
        rx_cnt = 0; // 准备接收下一帧
    }
}