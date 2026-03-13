#include "motor_thread.h"
#include "drv_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_uart.h"

/* 引入手柄全局数据 */
#include "ps2_gamepad.h"
#include "drv_imu.h" // 【新增】：引入IMU数据头文件
#include <math.h>    // 【新增】：引入数学库用于 atan2 和 sqrt

// 闭环控制周期：20ms (50Hz 控制频率)
#define PID_SAMPLE_TIME_MS 20

// 假设：小车轮子转一圈对应的编码器总脉冲数 (根据你的电机自行修改)
#define PULSES_PER_ROUND 1040.0f

// 定义小车允许的最大转速 (RPM)
#define MAX_MOTOR_RPM 300.0f

#define MAX_ACCEL_RPM_PS 600.0f

/* ========================================================
 * 1. 外部导航变量与状态机定义 (由 GUI 线程传入)
 * ======================================================== */
extern volatile float g_nav_target_x;
extern volatile float g_nav_target_y;
extern volatile bool g_nav_start_flag;

typedef enum
{
    NAV_STATE_IDLE = 0,
    NAV_STATE_INIT,
    NAV_STATE_TURN,
    NAV_STATE_STRAIGHT,
    NAV_STATE_STOP
} nav_state_t;

static nav_state_t current_nav_state = NAV_STATE_IDLE;
static float target_yaw_angle = 0.0f;
static int32_t target_distance_pulses = 0;
static int32_t start_total_pulses[4] = {0};

/* ========================================================
 * 1. 定义 PID 控制器结构体
 * ======================================================== */
typedef struct
{
    float Kp, Ki, Kd;   // PID 参数
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 误差积分
    float max_integral; // 积分限幅 (防积分饱和)
    float max_output;   // 输出限幅 (对应 PWM 最大占空比 1000)
} pid_controller_t;

static pid_controller_t motor_pid[4];
static pid_controller_t yaw_pid;  // 【新增】：偏航角 PID 控制器
static pid_controller_t dist_pid; // 【新增】：距离 PID 控制器

volatile float g_motor_speed_rps[4] = {0.0f};
volatile float g_motor_speed_rpm[4] = {0.0f};
volatile int32_t g_motor_total_pulse[4] = {0};
extern motor_t chasis_motors[4];

volatile float g_pid_kp = 1.5f;
volatile float g_pid_ki = 0.1f;
volatile float g_pid_kd = 0.0f;

/* ========================================================
 * 2. PID 初始化与计算函数
 * ======================================================== */
static void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float max_out, float max_int)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_out;
    pid->max_integral = max_int;
}

static float pid_calc(pid_controller_t *pid, float target, float current)
{
    if (target == 0.0f)
    {
        pid->integral = 0.0f;
    }
    // 1. 计算当前误差
    pid->error = target - current;

    // 2. 计算误差积分 (并进行积分限幅，防止由于长时间堵转导致积分器爆表)
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral)
        pid->integral = -pid->max_integral;

    // 3. 计算误差微分
    float derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    // 4. 计算 PID 最终输出
    float output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 5. 输出限幅 (防止超出定时器的最大 PWM 占空比)
    if (output > pid->max_output)
        output = pid->max_output;
    else if (output < -pid->max_output)
        output = -pid->max_output;

    return output;
}

/* ========================================================
 * 3. 线程主入口
 * ======================================================== */
void motor_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    // 1. 初始化底层硬件与底层轮速 PID
    for (int i = 0; i < 4; i++)
    {
        drv_motor_init(&chasis_motors[i]);
        pid_init(&motor_pid[i], 1.5f, 0.1f, 0.0f, 1000.0f, 500.0f);
    }

    // 2. 初始化导航外环 PID
    // 偏航角 PID (参数需调试)：输出限制为转弯产生的最大转速差 (RPM)
    pid_init(&yaw_pid, 3.0f, 0.02f, 0.5f, 150.0f, 50.0f);
    // 直行距离 PID (参数需调试)：输出限制为最大直行转速 (RPM)
    pid_init(&dist_pid, 0.5f, 0.0f, 0.1f, 200.0f, 50.0f);

    TickType_t last_calc_tick = xTaskGetTickCount();
    static float current_target_rpm[4] = {0.0f};

    while (1)
    {
        TickType_t current_tick = xTaskGetTickCount();
        float actual_dt_s = (float)(current_tick - last_calc_tick) * (1000.0f / configTICK_RATE_HZ) / 1000.0f;
        last_calc_tick = current_tick;
        if (actual_dt_s <= 0.005f)
            actual_dt_s = (PID_SAMPLE_TIME_MS / 1000.0f);

        /* --- A. 控制源解析 (导航状态机 OR 手柄) --- */
        float base_forward_rpm = 0.0f;
        float base_turn_rpm = 0.0f;

        // 导航触发逻辑
        if (g_nav_start_flag && current_nav_state == NAV_STATE_IDLE)
        {
            current_nav_state = NAV_STATE_INIT;
            g_nav_start_flag = false;
        }

        switch (current_nav_state)
        {
        case NAV_STATE_INIT:
        {
            float x = g_nav_target_x;
            float y = g_nav_target_y;

            // 计算需要转动的角度 (X是右，Y是前)
            float angle_to_turn = atan2f(x, y) * 57.29578f;
            // IMU极性：向右减小。目标 Yaw = 当前 Yaw - 需要转的角度
            target_yaw_angle = g_imu_data.euler[2] - angle_to_turn;

            // 计算总行程并转换为脉冲数 (每毫米 6.897 脉冲)
            float dist_mm = sqrtf(x * x + y * y) * 10.0f;
            target_distance_pulses = (int32_t)(dist_mm * 6.897f);

            current_nav_state = NAV_STATE_TURN;
            break;
        }

        case NAV_STATE_TURN:
        {
            // 计算角度误差，并处理 -180 到 180 翻转
            float yaw_error = target_yaw_angle - g_imu_data.euler[2];
            while (yaw_error > 180.0f)
                yaw_error -= 360.0f;
            while (yaw_error < -180.0f)
                yaw_error += 360.0f;

            // 原地转弯 PID 输出
            base_turn_rpm = pid_calc(&yaw_pid, yaw_error, 0.0f);

            // 【新增：死区摩擦力补偿】
            // 假设 35.0f 是刚好能让车原地扭动的最小 RPM，你需要根据地面材质实测调整这个值
            const float MIN_TURN_RPM = 45.0f;

            // 只有当误差大于容差时才进行补偿，防止在目标点附近鬼畜抖动
            if (fabs(yaw_error) > 2.0f)
            {
                if (base_turn_rpm > 0.0f && base_turn_rpm < MIN_TURN_RPM)
                {
                    base_turn_rpm = MIN_TURN_RPM;
                }
                else if (base_turn_rpm < 0.0f && base_turn_rpm > -MIN_TURN_RPM)
                {
                    base_turn_rpm = -MIN_TURN_RPM;
                }
            }

            // 判断是否转到位：
            // 【修改】：放宽原地转弯的阈值到 3.0 度，剩下的误差交给直行阶段的航向锁定去解决！
            if (fabs(yaw_error) < 2.5f && fabs(g_imu_data.gyro[2]) < 5.0f)
            {
                for (int i = 0; i < 4; i++)
                    start_total_pulses[i] = chasis_motors[i].enc_total;
                current_nav_state = NAV_STATE_STRAIGHT;
            }
            break;
        }

        case NAV_STATE_STRAIGHT:
        {
            // 1. 直行位移 PID (求四轮平均已走脉冲数)
            int32_t current_avg_pulses = 0;
            for (int i = 0; i < 4; i++)
            {
                current_avg_pulses += abs(chasis_motors[i].enc_total - start_total_pulses[i]);
            }
            current_avg_pulses /= 4;

            int32_t dist_error = target_distance_pulses - current_avg_pulses;
            base_forward_rpm = pid_calc(&dist_pid, (float)target_distance_pulses, (float)current_avg_pulses);

            // 2. 航向锁定 PID：微调左右轮差速，走出绝对直线！
            float yaw_error = target_yaw_angle - g_imu_data.euler[2];
            while (yaw_error > 180.0f)
                yaw_error -= 360.0f;
            while (yaw_error < -180.0f)
                yaw_error += 360.0f;
            base_turn_rpm = pid_calc(&yaw_pid, yaw_error, 0.0f);

            // 距离目标点误差小于 50 个脉冲（约 7mm）时停车
            if (dist_error < 50)
            {
                current_nav_state = NAV_STATE_STOP;
            }
            break;
        }

        case NAV_STATE_STOP:
        {
            base_forward_rpm = 0.0f;
            base_turn_rpm = 0.0f;
            current_nav_state = NAV_STATE_IDLE; // 完成导航，切回 IDLE
            break;
        }

        case NAV_STATE_IDLE:
        default:
        {
            // IDLE 状态下，完全接管 PS2 手柄控制
            if (g_ps2_data.is_connected)
            {
                int16_t raw_y = g_ps2_data.right_y;
                int16_t raw_x = g_ps2_data.right_x;

                if (raw_y < 118 || raw_y > 138)
                {
                    base_forward_rpm = ((127.0f - raw_y) / 127.0f) * MAX_MOTOR_RPM;
                }
                if (raw_x < 118 || raw_x > 138)
                {
                    base_turn_rpm = ((127.0f - raw_x) / 127.0f) * MAX_MOTOR_RPM;
                }
            }
            break;
        }
        }

        /* --- B. 运动学逆解 (融合直行与转向) --- */
        float desired_rpm[4] = {0.0f};

        // 运动学方程: 左轮 = 前进 - 转向; 右轮 = 前进 + 转向
        float target_rpm_L = base_forward_rpm - base_turn_rpm;
        float target_rpm_R = base_forward_rpm + base_turn_rpm;

        // 最大限幅保护
        if (target_rpm_L > MAX_MOTOR_RPM)
            target_rpm_L = MAX_MOTOR_RPM;
        if (target_rpm_L < -MAX_MOTOR_RPM)
            target_rpm_L = -MAX_MOTOR_RPM;
        if (target_rpm_R > MAX_MOTOR_RPM)
            target_rpm_R = MAX_MOTOR_RPM;
        if (target_rpm_R < -MAX_MOTOR_RPM)
            target_rpm_R = -MAX_MOTOR_RPM;

        desired_rpm[0] = target_rpm_L; // M1 (左)
        desired_rpm[1] = target_rpm_L; // M2 (左)
        desired_rpm[2] = target_rpm_R; // M3 (右)
        desired_rpm[3] = target_rpm_R; // M4 (右)

        /* --- C. 梯形加减速平滑 (Slew Rate Limiter) --- */
        float max_rpm_step = MAX_ACCEL_RPM_PS * actual_dt_s;

        for (int i = 0; i < 4; i++)
        {
            if (current_target_rpm[i] < desired_rpm[i])
            {
                current_target_rpm[i] += max_rpm_step;
                if (current_target_rpm[i] > desired_rpm[i])
                    current_target_rpm[i] = desired_rpm[i];
            }
            else if (current_target_rpm[i] > desired_rpm[i])
            {
                current_target_rpm[i] -= max_rpm_step;
                if (current_target_rpm[i] < desired_rpm[i])
                    current_target_rpm[i] = desired_rpm[i];
            }
        }

        /* --- D. 底层轮速 PID 闭环 --- */
        for (int i = 0; i < 4; i++)
        {
            motor_pid[i].Kp = g_pid_kp;
            motor_pid[i].Ki = g_pid_ki;
            motor_pid[i].Kd = g_pid_kd;

            int32_t delta_pulses = drv_motor_update_encoder(&chasis_motors[i]);

            float raw_speed_rps = ((float)delta_pulses / PULSES_PER_ROUND) / actual_dt_s;
            float raw_speed_rpm = raw_speed_rps * 60.0f;

            const float ALPHA = 0.3f;
            static float filtered_speed_rpm[4] = {0.0f};
            filtered_speed_rpm[i] = (ALPHA * raw_speed_rpm) + ((1.0f - ALPHA) * filtered_speed_rpm[i]);

            g_motor_speed_rps[i] = filtered_speed_rpm[i] / 60.0f;
            g_motor_speed_rpm[i] = filtered_speed_rpm[i];
            g_motor_total_pulse[i] = chasis_motors[i].enc_total;

            float ff_pwm = (current_target_rpm[i] / MAX_MOTOR_RPM) * 1000.0f;
            float pid_pwm = pid_calc(&motor_pid[i], current_target_rpm[i], filtered_speed_rpm[i]);

            float total_pwm = ff_pwm + pid_pwm;

            if (total_pwm > 1000.0f)
                total_pwm = 1000.0f;
            else if (total_pwm < -1000.0f)
                total_pwm = -1000.0f;

            drv_motor_set_pwm(&chasis_motors[i], (int32_t)total_pwm);
        }

        vTaskDelay(pdMS_TO_TICKS(PID_SAMPLE_TIME_MS));
    }
}