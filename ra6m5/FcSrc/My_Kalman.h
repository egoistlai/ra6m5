#include "arm_math.h"
#include "SysConfig.h"

#define STATE_DIM 4
#define MEAS_DIM 2
#define Kalman_R 1.0f
#define Q_pos 20.0f
#define Q_vel 10.0f

typedef struct KalmanFilter
{
    arm_matrix_instance_f32 x_pre; // 先验状态
    arm_matrix_instance_f32 x_now; // 后验状态
    arm_matrix_instance_f32 P_pre; // 先验协方差
    arm_matrix_instance_f32 P_now; // 后验协方差
    arm_matrix_instance_f32 F;     // 状态转移矩阵
    arm_matrix_instance_f32 Hv;    // 速度观测矩阵
    arm_matrix_instance_f32 Hp;    // 位置观测矩阵
    arm_matrix_instance_f32 Q;     // 过程噪声
    arm_matrix_instance_f32 Rv;    // 速度观测噪声
    arm_matrix_instance_f32 Rp;    // 位置观测噪声
    float F_data[STATE_DIM * STATE_DIM];
    float Hv_data[MEAS_DIM * STATE_DIM];
    float Hp_data[MEAS_DIM * STATE_DIM];
    float Q_data[STATE_DIM * STATE_DIM];
    float Rv_data[MEAS_DIM * MEAS_DIM];
    float Rp_data[MEAS_DIM * MEAS_DIM];
    float x_pre_data[STATE_DIM];
    float x_now_data[STATE_DIM];
    float P_pre_data[STATE_DIM * STATE_DIM];
    float P_now_data[STATE_DIM * STATE_DIM];
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf);
void imu_predict(KalmanFilter *kf, float dt, float ax, float ay);
void update_vel(KalmanFilter *kf, float vx, float vy);
void update_pos(KalmanFilter *kf, float x, float y);

extern KalmanFilter my_kf;
