#include "My_Kalman.h"

KalmanFilter my_kf;

void KalmanFilter_Init(KalmanFilter *kf)
{
    arm_mat_init_f32(&kf->x_pre, STATE_DIM, 1, kf->x_pre_data);
    arm_mat_init_f32(&kf->x_now, STATE_DIM, 1, kf->x_now_data);
    arm_mat_init_f32(&kf->P_pre, STATE_DIM, STATE_DIM, kf->P_pre_data);
    arm_mat_init_f32(&kf->P_now, STATE_DIM, STATE_DIM, kf->P_now_data);
    arm_mat_init_f32(&kf->F, STATE_DIM, STATE_DIM, kf->F_data);
    arm_mat_init_f32(&kf->Hv, MEAS_DIM, STATE_DIM, kf->Hv_data);
    arm_mat_init_f32(&kf->Hp, MEAS_DIM, STATE_DIM, kf->Hp_data);
    arm_mat_init_f32(&kf->Q, STATE_DIM, STATE_DIM, kf->Q_data);
    arm_mat_init_f32(&kf->Rv, MEAS_DIM, MEAS_DIM, kf->Rv_data);
    arm_mat_init_f32(&kf->Rp, MEAS_DIM, MEAS_DIM, kf->Rp_data);

    memset(kf->x_pre_data, 0, sizeof(kf->x_pre_data));
    memset(kf->x_now_data, 0, sizeof(kf->x_now_data));

    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->P_pre_data[i * STATE_DIM + i] = 1.0f;
        kf->P_now_data[i * STATE_DIM + i] = 1.0f;
    }
    // float Q[4][4] = {
    //     {Q_pos, 0, 0, 0},
    //     {0, Q_pos, 0, 0},
    //     {0, 0, Q_vel, 0},
    //     {0, 0, 0, Q_vel}
    // };
    memset(kf->Q_data, 0, sizeof(kf->Q_data));
    kf->Q_data[0] = Q_pos;
    kf->Q_data[5] = Q_pos;
    kf->Q_data[10] = Q_vel;
    kf->Q_data[15] = Q_vel;
    // float H[4][4] = {
    //     {1, 0, 0, 0},
    //     {0, 1, 0, 0},
    //     {0, 0, 1, 0},
    //     {0, 0, 0, 1}
    // };
    memset(kf->Hv_data, 0, sizeof(kf->Hv_data));
    kf->Hv_data[2] = 1.0f;
    kf->Hv_data[7] = 1.0f;

    memset(kf->Hp_data, 0, sizeof(kf->Hp_data));
    kf->Hp_data[0] = 1.0f;
    kf->Hp_data[5] = 1.0f;

    // float F[4][4] = {
    //     {1, 0, dt, 0},
    //     {0, 1, 0, dt},
    //     {0, 0, 1, 0},
    //     {0, 0, 0, 1}
    // };
    memset(kf->F_data, 0, sizeof(kf->F_data));
    kf->F_data[0] = 1.0f;
    kf->F_data[5] = 1.0f;
    kf->F_data[10] = 1.0f;
    kf->F_data[15] = 1.0f;

    memset(kf->Rv_data, 0, sizeof(kf->Rv_data));
    kf->Rv_data[0] = Kalman_R;
    kf->Rv_data[3] = Kalman_R;
    memset(kf->Rp_data, 0, sizeof(kf->Rp_data));
    kf->Rp_data[0] = Kalman_R;
    kf->Rp_data[3] = Kalman_R;
}

// 传入ax和ay，单位cm/s^2
void imu_predict(KalmanFilter *kf, float dt, float ax, float ay)
{
    float x_prev[STATE_DIM];
    memcpy(x_prev, kf->x_now_data, sizeof(x_prev));

    kf->x_now_data[0] = x_prev[0] + x_prev[2] * dt + 0.5f * ax * dt * dt;
    kf->x_now_data[1] = x_prev[1] + x_prev[3] * dt + 0.5f * ay * dt * dt;
    kf->x_now_data[2] = x_prev[2] + ax * dt;
    kf->x_now_data[3] = x_prev[3] + ay * dt;

    kf->F_data[2] = dt;
    kf->F_data[7] = dt;

    arm_matrix_instance_f32 FP, Ft;
    float FP_data[STATE_DIM * STATE_DIM], Ft_data[STATE_DIM * STATE_DIM];

    arm_mat_init_f32(&FP, STATE_DIM, STATE_DIM, FP_data);
    arm_mat_init_f32(&Ft, STATE_DIM, STATE_DIM, Ft_data);

    arm_mat_mult_f32(&kf->F, &kf->P_pre, &FP);
    arm_mat_trans_f32(&kf->F, &Ft);
    arm_mat_mult_f32(&FP, &Ft, &kf->P_now);
    arm_mat_add_f32(&kf->P_now, &kf->Q, &kf->P_now);
    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->P_now_data[i * STATE_DIM + i] = fmaxf(kf->P_now_data[i * STATE_DIM + i], 0.1f);
        kf->P_now_data[i * STATE_DIM + i] = fminf(kf->P_now_data[i * STATE_DIM + i], 100.0f);
    }

    // 得到先验估计值和先验协方差矩阵
    memcpy(kf->x_pre_data, kf->x_now_data, sizeof(kf->x_pre_data));
    memcpy(kf->P_pre_data, kf->P_now_data, sizeof(kf->P_pre_data));
}

void update_vel(KalmanFilter *kf, float vx, float vy)
{
    // 计算残差
    arm_matrix_instance_f32 H_x;
    float H_x_data[MEAS_DIM];
    arm_mat_init_f32(&H_x, MEAS_DIM, 1, H_x_data);

    arm_mat_mult_f32(&kf->Hv, &kf->x_now, &H_x);

    float residual[MEAS_DIM] = {
        vx - H_x_data[0],
        vy - H_x_data[1]};
    // 计算增益矩阵K
    arm_matrix_instance_f32 Hvt, S, S_inv, K, temp1;
    float Hvt_data[STATE_DIM * MEAS_DIM];
    float S_data[MEAS_DIM * MEAS_DIM];
    float S_inv_data[MEAS_DIM * MEAS_DIM];
    float K_data[STATE_DIM * MEAS_DIM];
    float temp1_data[MEAS_DIM * STATE_DIM];

    arm_mat_init_f32(&Hvt, STATE_DIM, MEAS_DIM, Hvt_data);
    arm_mat_init_f32(&S, MEAS_DIM, MEAS_DIM, S_data);
    arm_mat_init_f32(&S_inv, MEAS_DIM, MEAS_DIM, S_inv_data);
    arm_mat_init_f32(&K, STATE_DIM, MEAS_DIM, K_data);
    arm_mat_init_f32(&temp1, MEAS_DIM, STATE_DIM, temp1_data);

    arm_mat_trans_f32(&kf->Hv, &Hvt);
    arm_mat_mult_f32(&kf->Hv, &kf->P_now, &temp1);
    arm_mat_mult_f32(&temp1, &Hvt, &S);
    arm_mat_add_f32(&S, &kf->Rv, &S);

    arm_mat_inverse_f32(&S, &S_inv);

    arm_mat_mult_f32(&kf->P_now, &Hvt, &K);
    arm_mat_mult_f32(&K, &S_inv, &K);

    // 更新状态
    arm_matrix_instance_f32 residual_mat, Ky;
    float residual_data[MEAS_DIM] = {residual[0], residual[1]};
    float Ky_data[STATE_DIM];

    arm_mat_init_f32(&residual_mat, MEAS_DIM, 1, residual_data);
    arm_mat_init_f32(&Ky, STATE_DIM, 1, Ky_data);

    arm_mat_mult_f32(&K, &residual_mat, &Ky);

    // kf->x_now_data[2] += Ky_data[2];
    // kf->x_now_data[3] += Ky_data[3];
    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->x_now_data[i] += Ky_data[i];
    }

    // 协方差更新
    arm_matrix_instance_f32 KH, I, I_KH;
    float KH_data[STATE_DIM * STATE_DIM];
    float I_data[STATE_DIM * STATE_DIM];
    float I_KH_data[STATE_DIM * STATE_DIM];

    arm_mat_init_f32(&KH, STATE_DIM, STATE_DIM, KH_data);
    arm_mat_init_f32(&I, STATE_DIM, STATE_DIM, I_data);
    arm_mat_init_f32(&I_KH, STATE_DIM, STATE_DIM, I_KH_data);

    for (int i = 0; i < STATE_DIM; i++)
    {
        for (int j = 0; j < STATE_DIM; j++)
        {
            I_data[i * STATE_DIM + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    arm_mat_mult_f32(&K, &kf->Hv, &KH);
    arm_mat_sub_f32(&I, &KH, &I_KH);
    arm_mat_mult_f32(&I_KH, &kf->P_now, &kf->P_now);
}

void update_pos(KalmanFilter *kf, float x, float y)
{
    // 计算残差
    arm_matrix_instance_f32 H_x;
    float H_x_data[MEAS_DIM];
    arm_mat_init_f32(&H_x, MEAS_DIM, 1, H_x_data);

    arm_mat_mult_f32(&kf->Hp, &kf->x_now, &H_x);

    float residual[MEAS_DIM] = {
        x - H_x_data[0],
        y - H_x_data[1]};
    // 计算增益矩阵K
    arm_matrix_instance_f32 Hpt, S, S_inv, K, temp1;
    float Hpt_data[STATE_DIM * MEAS_DIM];
    float S_data[MEAS_DIM * MEAS_DIM];
    float S_inv_data[MEAS_DIM * MEAS_DIM];
    float K_data[STATE_DIM * MEAS_DIM];
    float temp1_data[MEAS_DIM * STATE_DIM];

    arm_mat_init_f32(&Hpt, STATE_DIM, MEAS_DIM, Hpt_data);
    arm_mat_init_f32(&S, MEAS_DIM, MEAS_DIM, S_data);
    arm_mat_init_f32(&S_inv, MEAS_DIM, MEAS_DIM, S_inv_data);
    arm_mat_init_f32(&K, STATE_DIM, MEAS_DIM, K_data);
    arm_mat_init_f32(&temp1, MEAS_DIM, STATE_DIM, temp1_data);

    arm_mat_trans_f32(&kf->Hp, &Hpt);
    arm_mat_mult_f32(&kf->Hp, &kf->P_now, &temp1);
    arm_mat_mult_f32(&temp1, &Hpt, &S);
    arm_mat_add_f32(&S, &kf->Rp, &S);

    arm_mat_inverse_f32(&S, &S_inv);

    arm_mat_mult_f32(&kf->P_now, &Hpt, &K);
    arm_mat_mult_f32(&K, &S_inv, &K);

    // 更新状态
    arm_matrix_instance_f32 residual_mat, Ky;
    float residual_data[MEAS_DIM] = {residual[0], residual[1]};
    float Ky_data[STATE_DIM];

    arm_mat_init_f32(&residual_mat, MEAS_DIM, 1, residual_data);
    arm_mat_init_f32(&Ky, STATE_DIM, 1, Ky_data);

    arm_mat_mult_f32(&K, &residual_mat, &Ky);

    // kf->x_now_data[0] += Ky_data[0];
    // kf->x_now_data[1] += Ky_data[1];
    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->x_now_data[i] += Ky_data[i];
    }

    // 协方差更新
    arm_matrix_instance_f32 KH, I, I_KH;
    float KH_data[STATE_DIM * STATE_DIM];
    float I_data[STATE_DIM * STATE_DIM];
    float I_KH_data[STATE_DIM * STATE_DIM];

    arm_mat_init_f32(&KH, STATE_DIM, STATE_DIM, KH_data);
    arm_mat_init_f32(&I, STATE_DIM, STATE_DIM, I_data);
    arm_mat_init_f32(&I_KH, STATE_DIM, STATE_DIM, I_KH_data);

    for (int i = 0; i < STATE_DIM; i++)
    {
        for (int j = 0; j < STATE_DIM; j++)
        {
            I_data[i * STATE_DIM + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    arm_mat_mult_f32(&K, &kf->Hp, &KH);
    arm_mat_sub_f32(&I, &KH, &I_KH);
    arm_mat_mult_f32(&I_KH, &kf->P_now, &kf->P_now);
}
