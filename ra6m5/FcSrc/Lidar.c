#include "Lidar.h"
#include "math.h"
#include "My_Kalman.h"
#include "My_Task.h"

lidar_rx_state_t lidar_rx_state = LIDAR_WAIT_HEAD;
lidar_frame_t rx_lidar_frame;
u8 rx_lidar_data_cnt = 0;
u8 rx_lidar_checksum = 0;

u8 lidar_init_flag = 0;
u8 lidar_start_flag = 0;     // 雷达启动标志位，0表示未启动，1表示已启动
float lidar_initial_yaw = 0; // 安装角度差，雷达在最上方，俯视视角逆时针旋转角度为正，范围-180到180，x正方向为机头，y正方向为机架左侧，z正方向为机架上方
float lidar_sin_yaw = 0;
float lidar_cos_yaw = 0;

s16 parse_s16_le(u8 low, u8 high)
{
    u16 unsigned_value = (high << 8) | low;
    return (s16)unsigned_value;
}

void Lidar_Process_Frame(lidar_frame_t *frame)
{
    if (!lidar_init_flag)
    {
        lidar_sin_yaw = sinf(LIDAR_SET_ANGLE * PI_180);
        lidar_cos_yaw = cosf(LIDAR_SET_ANGLE * PI_180);
        lidar_init_flag = 1;
    }
    sensor_buffer.lidar.yaw_lidar = parse_s16_le(frame->data[6], frame->data[7]) / 100.0f;
    float temp_x = -parse_s16_le(frame->data[0], frame->data[1]) / 10.0f;
    float temp_y = -parse_s16_le(frame->data[2], frame->data[3]) / 10.0f;
    sensor_buffer.lidar.x_lidar = temp_x * lidar_cos_yaw - temp_y * lidar_sin_yaw;
    sensor_buffer.lidar.y_lidar = temp_x * lidar_sin_yaw + temp_y * lidar_cos_yaw;
    sensor_buffer.lidar.z_lidar = parse_s16_le(frame->data[4], frame->data[5]) / 10.0f;
    alt_cm = sensor_buffer.lidar.z_lidar;
    sensor_buffer.lidar.timestamp = my_time_stamp;
    fc_my1_data.st_data.lidar_x = float_to_s16(sensor_buffer.lidar.x_lidar);
    fc_my1_data.st_data.lidar_y = float_to_s16(sensor_buffer.lidar.y_lidar);
    fc_my1_data.st_data.lidar_z = parse_s16_le(frame->data[4], frame->data[5]);
    fc_my1_data.st_data.lidar_yaw = parse_s16_le(frame->data[6], frame->data[7]);
    update_pos(&my_kf, sensor_buffer.lidar.x_lidar, sensor_buffer.lidar.y_lidar);
}

void Lidar_Data_Receive(u8 data)
{
    switch (lidar_rx_state)
    {
    case LIDAR_WAIT_HEAD:
        if (data == LIDAR_FRAME_HEAD)
        {
            rx_lidar_frame.head = data;
            lidar_rx_state = LIDAR_WAIT_LEN;
            rx_lidar_checksum = data;
        }
        break;

    case LIDAR_WAIT_LEN:
        if (data == LIDAR_DATA_LEN)
        {
            rx_lidar_frame.length = data;
            rx_lidar_checksum += data;
            rx_lidar_data_cnt = 0;
            lidar_rx_state = data > 0 ? LIDAR_WAIT_DATA : LIDAR_WAIT_CHECK;
        }
        else
        {
            lidar_rx_state = LIDAR_WAIT_HEAD;
        }
        break;

    case LIDAR_WAIT_DATA:
        rx_lidar_frame.data[rx_lidar_data_cnt++] = data;
        rx_lidar_checksum += data;
        if (rx_lidar_data_cnt >= rx_lidar_frame.length)
        {
            lidar_rx_state = LIDAR_WAIT_CHECK;
        }
        break;

    case LIDAR_WAIT_CHECK:
        rx_lidar_frame.checksum = data;
        if (data == rx_lidar_checksum)
        {
            lidar_rx_state = LIDAR_WAIT_END;
        }
        else
        {
            lidar_rx_state = LIDAR_WAIT_HEAD;
        }
        break;

    case LIDAR_WAIT_END:
        if (data == LIDAR_FRAME_END)
        {
            rx_lidar_frame.end = data;
            Lidar_Process_Frame(&rx_lidar_frame);
        }
        lidar_rx_state = LIDAR_WAIT_HEAD;
        break;

    default:
        lidar_rx_state = LIDAR_WAIT_HEAD;
        break;
    }
}
