#ifndef DRV_MOTOR_H_
#define DRV_MOTOR_H_

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

/* 电机对象结构体定义 */
typedef struct
{
    // === 硬件配置 (只读，初始化时绑定) ===
    timer_instance_t const *p_pwm_timer; 
    timer_instance_t const *p_enc_timer; 
    uint32_t pwm_channel;                
    bsp_io_port_pin_t dir_pin_1;         
    bsp_io_port_pin_t dir_pin_2;         
    
    // 【修改点】：拆分反转标志位
    bool motor_reversed;  // 电机物理输出反转标志 (调整实际转向)
    bool enc_reversed;    // 编码器读数反转标志 (调整测速正负号)

    // === 运行状态 (动态，自动更新) ===
    int32_t pwm_duty;      
    uint16_t enc_last_cnt; 
    int32_t enc_delta;     
    int32_t enc_total;     
} motor_t;

/* 对外接口 API */
fsp_err_t drv_motor_init(motor_t *m);
void drv_motor_set_pwm(motor_t *m, int32_t duty_per_mille); // 设置千分比占空比 (-1000 ~ 1000)
int32_t drv_motor_update_encoder(motor_t *m);               // 读取编码器增量并更新状态



#endif /* DRV_MOTOR_H_ */