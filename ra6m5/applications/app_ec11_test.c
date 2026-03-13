#include "app.h"
#include "drv_uart.h"
#include "drv_gpio_ec11.h"
#include "drv_joystick.h" // 引入新头文件
#include <stdio.h>

void app_ec11_test(void)
{
    fsp_err_t err;
    joy_dir_t last_dir = JOY_NONE;

    err = drv_uart_init();
    if (FSP_SUCCESS != err)
        __BKPT();

    /* 初始化摇杆 */
    err = drv_gpio_ec11_init();
    if (FSP_SUCCESS != err)
    {
        printf("%s %d\r\n", __FUNCTION__, __LINE__);
        __BKPT();
    }
    err = drv_joystick_init();
    if (FSP_SUCCESS != err)
    {
        printf("Joystick Init Failed!\r\n");
        __BKPT();
    }

    printf("Joystick Test Start (Analog Mode)\r\n");

    while (1)
    {
        /* 1. 检测按键 */
        if (drv_uart_get_pin_state(EC11_PIN_KEY))
        {
            R_BSP_SoftwareDelay(drv_uart_get_pin_press_tick(EC11_PIN_KEY), BSP_DELAY_UNITS_MILLISECONDS);
            drv_uart_set_pin_state(EC11_PIN_KEY, 0);
            printf("pressed (KEY)!\r\n");
        }

        /* 2. 检测方向 */
        joy_dir_t current_dir = drv_joystick_get_dir();

        /* 只有状态改变时才打印，或者持续打印 */
        if (current_dir != JOY_CENTER && current_dir != last_dir)
        {
            switch (current_dir)
            {
            case JOY_UP:
                printf("Direction: UP\r\n");
                break;
            case JOY_DOWN:
                printf("Direction: DOWN\r\n");
                break;
            case JOY_LEFT:
                printf("Direction: LEFT\r\n");
                break;
            case JOY_RIGHT:
                printf("Direction: RIGHT\r\n");
                break;
            default:
                break;
            }
        }

        last_dir = current_dir;
        R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS); // 采样间隔
    }
}