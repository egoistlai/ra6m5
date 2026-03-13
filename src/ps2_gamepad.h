#ifndef PS2_GAMEPAD_H_
#define PS2_GAMEPAD_H_

#include <stdint.h>
#include <stdbool.h>

/* PS2 手柄全局数据结构 */
typedef struct
{
    uint8_t left_x;    // 左摇杆 X 轴 (0~255, 中间 127/128)
    uint8_t left_y;    // 左摇杆 Y 轴 (0~255, 中间 127/128)
    uint8_t right_x;   // 右摇杆 X 轴 (0~255, 中间 127/128)
    uint8_t right_y;   // 右摇杆 Y 轴 (0~255, 中间 127/128)
    uint16_t buttons;  // 按键状态掩码 (由于各个接收器协议不同，后续可通过十六进制提取)
    bool is_connected; // 手柄接收器是否已连接
} ps2_gamepad_t;

/* 声明一个全局外部变量，供所有包含了此头文件的线程使用 */
extern volatile ps2_gamepad_t g_ps2_data;

#endif /* PS2_GAMEPAD_H_ */