#include "drv_motor.h"

// 宏定义：最大占空比（千分比）
#define MOTOR_MAX_DUTY 1000

motor_t chasis_motors[4] = {
    // 轮子 1 (M1)
    {
        .p_pwm_timer = &g_timer_pwm_m1,
        .p_enc_timer = &g_timer_enc_m1,
        .pwm_channel = GPT_IO_PIN_GTIOCA,
        .dir_pin_1 = BSP_IO_PORT_02_PIN_04, // P204 (AIN1)
        .dir_pin_2 = BSP_IO_PORT_02_PIN_05, // P205 (AIN2)
        .motor_reversed = false,            // 单独控制电机转向
        .enc_reversed = true                // 单独控制编码器极性
    },
    // 轮子 2 (M2)
    {
        .p_pwm_timer = &g_timer_pwm_m2,
        .p_enc_timer = &g_timer_enc_m2,
        .pwm_channel = GPT_IO_PIN_GTIOCA,
        .dir_pin_1 = BSP_IO_PORT_06_PIN_03, // P603 (BIN1)
        .dir_pin_2 = BSP_IO_PORT_06_PIN_04, // P604 (BIN2)
        .motor_reversed = true,
        .enc_reversed = false},
    // 轮子 3 (M3)
    {
        .p_pwm_timer = &g_timer_pwm_m3,
        .p_enc_timer = &g_timer_enc_m3,
        .pwm_channel = GPT_IO_PIN_GTIOCA,
        .dir_pin_1 = BSP_IO_PORT_03_PIN_14, // P314 (CIN1)
        .dir_pin_2 = BSP_IO_PORT_03_PIN_15, // P315 (CIN2)
        .motor_reversed = true,
        .enc_reversed = false},
    // 轮子 4 (M4)
    {
        .p_pwm_timer = &g_timer_pwm_m4,
        .p_enc_timer = &g_timer_enc_m4,
        .pwm_channel = GPT_IO_PIN_GTIOCA,
        .dir_pin_1 = BSP_IO_PORT_07_PIN_04, // P704 (DIN1)
        .dir_pin_2 = BSP_IO_PORT_07_PIN_05, // P705 (DIN2)
        .motor_reversed = false,
        .enc_reversed = true}};

fsp_err_t drv_motor_init(motor_t *m)
{
    fsp_err_t err = FSP_SUCCESS;

    // 1. 初始化 PWM 定时器并启动
    if (m->p_pwm_timer != NULL)
    {
        err = m->p_pwm_timer->p_api->open(m->p_pwm_timer->p_ctrl, m->p_pwm_timer->p_cfg);
        if (err == FSP_SUCCESS)
        {
            m->p_pwm_timer->p_api->start(m->p_pwm_timer->p_ctrl);
        }
    }

    // 2. 初始化 编码器定时器并启动
    if (m->p_enc_timer != NULL)
    {
        err = m->p_enc_timer->p_api->open(m->p_enc_timer->p_ctrl, m->p_enc_timer->p_cfg);
        if (err == FSP_SUCCESS)
        {
            m->p_enc_timer->p_api->start(m->p_enc_timer->p_ctrl);
        }
    }

    // 3. 运行状态清零
    m->pwm_duty = 0;
    m->enc_last_cnt = 0;
    m->enc_delta = 0;
    m->enc_total = 0;

    return err;
}

void drv_motor_set_pwm(motor_t *m, int32_t duty_per_mille)
{
    // 保护限制：占空比不允许超过 ±1000
    if (duty_per_mille > MOTOR_MAX_DUTY)
        duty_per_mille = MOTOR_MAX_DUTY;
    if (duty_per_mille < -MOTOR_MAX_DUTY)
        duty_per_mille = -MOTOR_MAX_DUTY;

    // 处理软件层面的逻辑反转
    int32_t actual_duty = m->motor_reversed ? -duty_per_mille : duty_per_mille;
    m->pwm_duty = duty_per_mille;

    // 1. 设置正反转 GPIO
    if (actual_duty > 0)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_1, BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_2, BSP_IO_LEVEL_LOW);
    }
    else if (actual_duty < 0)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_1, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_2, BSP_IO_LEVEL_HIGH);
    }
    else
    {
        // 0 占空比时，将 IN1 和 IN2 全拉低实现电机停止/滑行
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_1, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, m->dir_pin_2, BSP_IO_LEVEL_LOW);
    }

    // 2. 计算实际向寄存器写入的计数值 (自动适配你在 FSP 里设置的 Period)
    uint32_t period = m->p_pwm_timer->p_cfg->period_counts;
    uint32_t abs_duty = (actual_duty > 0) ? actual_duty : -actual_duty;
    uint32_t duty_counts = (period * abs_duty) / MOTOR_MAX_DUTY;

    // 3. 设置 FSP PWM 占空比
    m->p_pwm_timer->p_api->dutyCycleSet(m->p_pwm_timer->p_ctrl, duty_counts, m->pwm_channel);
}

int32_t drv_motor_update_encoder(motor_t *m)
{
    timer_status_t status;

    // 获取当前 GPT 定时器内部的计数值 (FSP 返回的是 32 位变量)
    m->p_enc_timer->p_api->statusGet(m->p_enc_timer->p_ctrl, &status);

    // 【核心算式】：强行截断为 16 位无符号数
    uint16_t current_cnt = (uint16_t)status.counter;
    uint16_t last_cnt = (uint16_t)m->enc_last_cnt;

    // 利用 16 位无符号相减，再强转为 16 位有符号数，完美解决 0xFFFF 溢出翻转问题！
    int16_t delta_16bit = (int16_t)(current_cnt - last_cnt);

    // 扩展回 32 位参与系统物理量运算
    int32_t delta = (int32_t)delta_16bit;

    // 如果该轮子在软件上配置了反向，读到的编码器增量也要反向，保证速度闭环为正反馈
    if (m->enc_reversed)
    {
        delta = -delta;
    }

    // 更新状态
    m->enc_delta = delta;
    m->enc_total += delta;
    m->enc_last_cnt = current_cnt;

    return delta; // 返回本周期内的增量
}