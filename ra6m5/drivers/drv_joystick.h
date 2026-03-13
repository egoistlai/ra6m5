#ifndef DRV_JOYSTICK_H
#define DRV_JOYSTICK_H

#include "hal_data.h"

// 定义方向枚举
typedef enum
{
    JOY_NONE = 0,
    JOY_UP,
    JOY_DOWN,
    JOY_LEFT,
    JOY_RIGHT,
    JOY_CENTER
} joy_dir_t;


// 函数声明
fsp_err_t drv_joystick_init(void);
joy_dir_t drv_joystick_get_dir(void);

#endif