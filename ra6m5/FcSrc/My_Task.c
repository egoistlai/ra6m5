#include "My_Task.h"
#include "Drv_AnoOf.h"
#include "My_Kalman.h"
#include <math.h>

// 图像坐标系定义同世界坐标系，前向为x，左向为y，天向为z
my_PID_t Pos_x_world;
my_PID_t Pos_y_world;
my_PID_t Pos_z_world;
my_PID_t Pos_x_img; // 图像坐标系的x轴位置,输入量像素差
my_PID_t Pos_y_img; // 图像坐标系的y轴位置,输入量像素差
my_PID_t Vel_z_img; // 图像坐标系的z轴位置,输入量速度差
my_PID_t Pos_yaw;
my_PID_t Vel_x;
my_PID_t Vel_y;
my_PID_t Vel_z;
my_PID_t Vel_yaw;

SensorBuffer sensor_buffer;
StateEstimator state_est;
static NoiseStats ax_noise = {0};
static NoiseStats ay_noise = {0};

s16 control_x = 0;   // 头向速度，厘米每秒
s16 control_y = 0;   // 左向速度，厘米每秒
s16 control_z = 0;   // 天向速度，厘米每秒
s16 control_yaw = 0; // 航向转动角速度，度每秒，逆时针为正

u8 task_mode = 0; // 0:无任务, 1:起飞, 2:降落, 3:悬停, 4:定点, 5:追踪
u8 task_start_flag = 0;
u8 hover_start_flag = 0;
u8 task_ok = 0;
u8 listen_flag = 0;
u8 landed_flag = 0;
u8 takeoff_phase = 0;
s16 set_threshold = 10;
u32 my_time_stamp = 0;

u16 takeoff_height = 0;
// 设置点时以下变量为位置单位cm，追踪模式时以下变量为速度单位cm/s
s16 set_x = 0;
s16 set_y = 0;
s16 set_z = 0;
s16 set_yaw = 0;
s16 track_mode = 0; // 追踪模式，0:图像追踪+角度跟踪；1:图像跟踪+角速度跟踪(cam1) 2:图像跟踪+角速度跟踪(cam2)
u8 task_state = 0;
u8 task_count = 0;

// 传感器数据
s16 vel_Z = 0;
float vel_X = 0;
float vel_Y = 0;
s16 vel_X_fix = 0;
s16 vel_Y_fix = 0;
float alt_cm = 0;
float target_alt = 0;
u8 quality = 0;
u8 init_yaw_flag = 0;
float Yaw = 0; // 以机头朝前为0，俯视视角顺时针旋转角度增加，范围-180到180
float init_Yaw = 0;
float Pitch = 0;
float Roll = 0;
float target_yaw = 0;
float target_angle;
float target_xs1;
float target_ys1;
float delta_x_world;
float delta_y_world;
float delta_x_fc;
float delta_y_fc;
float sin_vals;
float cos_vals;
float target_climb_rate;

// PID滤波系数
float alpha = 0.2f;
float d_t = 0.01f;

float x_coordinate = 0;
float y_coordinate = 0;
float set_x_coordinate = 0;
float set_y_coordinate = 0;

int result = 0;
float get_angle(float delta_x, float delta_y)
{
    if (delta_x <= 10e-4 && delta_y < 0)
    {
        return -90.0f;
    }
    else if (delta_x <= 10e-4 && delta_y >= 0)
    {
        return 90.0f;
    }
    float the_angle = atan2f(delta_y, delta_x) * 180 / PI;
    return the_angle;
}

float get_yaw(void)
{
    if (init_yaw_flag)
    {
        // Yaw = fc_att.st_data.yaw_x100 / 100.0f - init_Yaw;
        Yaw = sensor_buffer.lidar.yaw_lidar;
        if (Yaw > 180.0f)
        {
            Yaw = 360.0f - Yaw;
        }
        else if (Yaw < -180.0f)
        {
            Yaw = 360.0f + Yaw;
        }
        return Yaw;
    }
    return 0.0f;
}

s16 alt_u32_to_s16(u32 u_val)
{
    if (u_val <= INT16_MAX)
    {
        return (s16)u_val;
    }

    u16 low16 = u_val & 0xFFFF;

    if (low16 >= 0x8000)
    {
        return 0;
    }

    return INT16_MAX;
}

s16 float_to_s16(float value)
{
    if (isnan(value))
        return 0;
    if (isinf(value))
        return (value > 0) ? INT16_MAX : INT16_MIN;

    if (value > INT16_MAX)
        return INT16_MAX;
    if (value < INT16_MIN)
        return INT16_MIN;

    return (s16)roundf(value);
}

s16 s32_to_s16(s32 value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    else if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    else
    {
        return (s16)value;
    }
}

void imuCallback(void)
{
    sensor_buffer.imu_ready = 1;
    if (sensor_buffer.flow_ready)
    {
        updateStateEstimator(&sensor_buffer, &state_est);
    }
}

void flowCallback(void)
{
    sensor_buffer.flow_ready = 1;
    if (sensor_buffer.imu_ready)
    {
        updateStateEstimator(&sensor_buffer, &state_est);
    }
}

u8 update_of(void)
{
    if (ano_of.work_sta == 1)
    {
        if (ano_of.of2_sta)
        {
            sensor_buffer.flow.vx = (float)ano_of.of2_dx;
            sensor_buffer.flow.vy = (float)ano_of.of2_dy;
            sensor_buffer.flow.quality = ano_of.of_quality / 255.0f;
            sensor_buffer.flow.timestamp = my_time_stamp;
            flowCallback();
            return 0;
        }
        else if (ano_of.of1_sta)
        {
            sensor_buffer.flow.vx = (float)ano_of.of1_dx;
            sensor_buffer.flow.vy = (float)ano_of.of1_dy;
            sensor_buffer.flow.quality = ano_of.of_quality / 255.0f;
            sensor_buffer.flow.timestamp = my_time_stamp;
            flowCallback();
            return 1;
        }
        else if (ano_of.of0_sta)
        {
            sensor_buffer.flow.vx = (float)ano_of.of0_dx;
            sensor_buffer.flow.vy = (float)ano_of.of0_dy;
            sensor_buffer.flow.quality = ano_of.of_quality / 255.0f;
            sensor_buffer.flow.timestamp = my_time_stamp;
            flowCallback();
            return 2;
        }
    }
    sensor_buffer.flow.vx = (float)fc_vel.st_data.vel_x;
    sensor_buffer.flow.vy = (float)fc_vel.st_data.vel_y;
    sensor_buffer.flow.quality = 0.8;
    sensor_buffer.flow.timestamp = my_time_stamp;
    flowCallback();
    return 3;
}

void update_imu(void)
{
    sensor_buffer.imu.vx = (float)fc_vel.st_data.vel_x;
    sensor_buffer.imu.vy = (float)fc_vel.st_data.vel_y;
    sensor_buffer.imu.timestamp = my_time_stamp;
    imuCallback();
}

float adaptive_lpf(float current_val, float prev_val, float noise_variance, float base_cutoff)
{
    // 计算自适应系数（噪声越大，滤波越强）
    alpha = base_cutoff / (base_cutoff + fmaxf(noise_variance, 0.1f));
    return alpha * current_val + (1 - alpha) * prev_val;
}

void init_noise_stats(void)
{
    memset(&ax_noise, 0, sizeof(NoiseStats));
    memset(&ay_noise, 0, sizeof(NoiseStats));
    ax_noise.index = 0;
    ay_noise.index = 0;
}

float calc_noise_variance(float new_val, NoiseStats *stats)
{
    if (stats->count >= NOISE_WINDOW_SIZE)
    {
        float old_val = stats->window[stats->index];
        stats->sum -= old_val;
        stats->sum_sq -= old_val * old_val;
    }
    else
    {
        stats->count++;
    }

    stats->window[stats->index] = new_val;
    stats->sum += new_val;
    stats->sum_sq += new_val * new_val;

    stats->index = (stats->index + 1) % NOISE_WINDOW_SIZE;

    float mean = stats->sum / stats->count;
    float variance = (stats->sum_sq / stats->count) - (mean * mean);

    return fmaxf(variance, 1e-6f);
}

void update_acc(float x_bias, float y_bias)
{
    float ax = (float)fc_acc.st_data.acc_x - x_bias;
    float ay = (float)fc_acc.st_data.acc_y - y_bias;
    float var_ax = calc_noise_variance(ax, &ax_noise);
    float var_ay = calc_noise_variance(ay, &ay_noise);
    float noise_variance = (var_ax + var_ay) * 0.5f;
    sensor_buffer.imu.ax = adaptive_lpf(ax, sensor_buffer.imu.ax, noise_variance, 0.2f);
    sensor_buffer.imu.ay = adaptive_lpf(ay, sensor_buffer.imu.ay, noise_variance, 0.2f);
    Yaw = get_yaw();
    float sin_val = sinf(Yaw * PI_180);
    float cos_val = cosf(Yaw * PI_180);
    float ax_world = sensor_buffer.imu.ax * cos_val - sensor_buffer.imu.ay * sin_val;
    float ay_world = sensor_buffer.imu.ax * sin_val + sensor_buffer.imu.ay * cos_val;
    if (fabs(ax_world) < 5)
    {
        ax_world = 0.0f;
    }
    if (fabs(ay_world) < 5)
    {
        ay_world = 0.0f;
    }
    imu_predict(&my_kf, (my_time_stamp - sensor_buffer.imu.acc_timestamp) * 0.001f, ax_world / 1.37f, ay_world / 1.37f);
    sensor_buffer.imu.acc_timestamp = my_time_stamp;
}

void updateStateEstimator(SensorBuffer *buffer, StateEstimator *state)
{
    u32 current_timestamp = my_time_stamp;

    //float dt = (current_timestamp - state->last_timestamp) * 0.001f;

    if (buffer->flow_ready && buffer->imu_ready)
    {
        float k = buffer->flow.quality;

        state->velocity_x_fused = k * buffer->flow.vx + (1 - k) * buffer->imu.vx;

        state->velocity_y_fused = k * buffer->flow.vy + (1 - k) * buffer->imu.vy;

        buffer->flow_ready = 0;
        buffer->imu_ready = 0;
        state->last_timestamp = current_timestamp;

        Yaw = get_yaw();
        float sin_val = sinf(Yaw * PI_180);
        float cos_val = cosf(Yaw * PI_180);
        float x_world = state->velocity_x_fused * cos_val - state->velocity_y_fused * sin_val;
        float y_world = state->velocity_x_fused * sin_val + state->velocity_y_fused * cos_val;
        update_vel(&my_kf, x_world, y_world);
        // vel_X = state->velocity_x_fused;
        // vel_Y = state->velocity_y_fused;
        vel_X = state->velocity_x_fused;
        vel_Y = state->velocity_y_fused;
        // vel_X = my_kf.x_now_data[2];
        // vel_Y = my_kf.x_now_data[3];
        x_coordinate = sensor_buffer.lidar.x_lidar;
        y_coordinate = sensor_buffer.lidar.y_lidar;
        fc_my_data.st_data.vel_x = float_to_s16(vel_X);
        fc_my_data.st_data.vel_y = float_to_s16(vel_Y);
        // fc_my_data.st_data.x = float_to_s16(x_coordinate);
        // fc_my_data.st_data.y = float_to_s16(y_coordinate);
        // fc_my_data.st_data.z = float_to_s16(alt_cm);
        // fc_my_data.st_data.yaw = float_to_s16(Yaw*100);
        fc_my_data.st_data.control_x = control_x;
        fc_my_data.st_data.control_y = control_y;
        fc_my_data.st_data.control_z = control_z;
        fc_my_data.st_data.control_yaw = control_yaw;
        vel_Z = fc_vel.st_data.vel_z;
    }
    else
    {
        state->velocity_x_fused = buffer->imu.vx;
        state->velocity_y_fused = buffer->imu.vx;
        buffer->imu_ready = 0;
        state->last_timestamp = current_timestamp;

        Yaw = get_yaw();
        float sin_val = sinf(Yaw * PI_180);
        float cos_val = cosf(Yaw * PI_180);
        float x_world = state->velocity_x_fused * cos_val - state->velocity_y_fused * sin_val;
        float y_world = state->velocity_x_fused * sin_val + state->velocity_y_fused * cos_val;
        update_vel(&my_kf, x_world, y_world);
        vel_X = state->velocity_x_fused;
        vel_Y = state->velocity_y_fused;
        // vel_X = my_kf.x_now_data[2]);
        // vel_Y = my_kf.x_now_data[3]);
        x_coordinate = sensor_buffer.lidar.x_lidar;
        y_coordinate = sensor_buffer.lidar.y_lidar;
        vel_Z = fc_vel.st_data.vel_z;
    }
}

s16 my_ABS(s16 x)
{
    return (x < 0) ? -x : x;
}

u8 is_motionless(float vel_threshold)
{
    if (vel_threshold <= 0)
        vel_threshold = 2;

    return (fabsf(vel_X) <= vel_threshold &&
            fabsf(vel_Y) <= vel_threshold);
}

u8 is_reach(s16 pos_threshold)
{
    return (my_ABS(set_z - float_to_s16(alt_cm)) <= pos_threshold &&
            my_ABS(set_x - float_to_s16(x_coordinate)) <= pos_threshold &&
            my_ABS(set_y - float_to_s16(y_coordinate)) <= pos_threshold);
}

int my_takeoff2(void)
{
    Yaw = get_yaw();
    PID_Setpoint(&Pos_z_world, (float)takeoff_height);
    PID_Setpoint(&Vel_x, 0);
    PID_Setpoint(&Vel_y, 0);
    PID_Setpoint(&Pos_yaw, 0);
    control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
    control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
    control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
    control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
    return (my_ABS(float_to_s16(alt_cm) - (s16)takeoff_height) < 3 && (fabs(Yaw) < 4)) && is_motionless(3);
}
int my_takeoff(void)
{
    Yaw = get_yaw();

    switch (takeoff_phase)
    {
    case 0:
        PID_Setpoint(&Pos_z_world, (float)takeoff_height);
        target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
        PID_Setpoint(&Vel_z, target_climb_rate);
        PID_Setpoint(&Vel_x, 0);
        PID_Setpoint(&Vel_y, 0);
        PID_Setpoint(&Pos_yaw, 0);

        float thrust_feedforward = 50;
        control_z = float_to_s16(
            thrust_feedforward +
            PID_Calculate(&Vel_z, vel_Z, d_t));

        control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
        control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));

        control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));

        if (alt_cm > (takeoff_height * 0.8f))
        {
            takeoff_phase = 1;
        }
        break;

    case 1:
        target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
        PID_Setpoint(&Vel_z, target_climb_rate);
        control_z = float_to_s16(PID_Calculate(&Vel_z, vel_Z, d_t));

        PID_Setpoint(&Pos_x_world, 0);
        PID_Setpoint(&Pos_y_world, 0);

        float target_vx = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
        float target_vy = PID_Calculate(&Pos_y_world, y_coordinate, d_t);

        PID_Setpoint(&Vel_x, target_vx);
        PID_Setpoint(&Vel_y, target_vy);

        control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
        control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));

        control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
        if (fabsf(alt_cm - takeoff_height) < 5 &&
            fabsf(Yaw) < 4)
        {
            takeoff_phase = 0;
            return 1;
        }
        break;
    }

    return 0;
}

int my_landing(void)
{
    Yaw = get_yaw();
    if (cam2_placement == 1)
    {
        PID_Setpoint(&Pos_z_world, land_threshold / 2.0f);
        target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
        PID_Setpoint(&Vel_z, target_climb_rate);
        control_z = float_to_s16(PID_Calculate(&Vel_z, (float)vel_Z, d_t));

        PID_Setpoint(&Pos_x_img, cam2_x_center);
        PID_Setpoint(&Pos_y_img, cam2_y_center);
        control_y = PID_Calculate(&Pos_x_img, (float)set_x, d_t);
        control_x = PID_Calculate(&Pos_y_img, (float)set_y, d_t);
        control_z = float_to_s16(PID_Calculate(&Pos_z_world, (float)alt_cm, d_t));
        control_yaw = 0;
        return (fabsf(alt_cm) < land_threshold);
    }
    else
    {
        PID_Setpoint(&Pos_z_world, land_threshold / 2.0f);
        target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
        PID_Setpoint(&Vel_z, target_climb_rate);
        control_z = float_to_s16(PID_Calculate(&Vel_z, (float)vel_Z, d_t));

        PID_Setpoint(&Pos_x_world, (float)set_x);
        PID_Setpoint(&Pos_y_world, (float)set_y);
        delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
        delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
        sin_vals = sinf(Yaw * PI_180);
        cos_vals = cosf(Yaw * PI_180);
        delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
        delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;

        PID_Setpoint(&Vel_x, delta_x_fc);
        PID_Setpoint(&Vel_y, delta_y_fc);
        control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
        control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
        control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
        control_yaw = 0;
        return (fabsf(alt_cm) < land_threshold);
    }
}

int my_hovering2(void)
{
    PID_Setpoint(&Pos_z_world, target_alt);
    PID_Setpoint(&Pos_x_world, set_x_coordinate);
    PID_Setpoint(&Pos_y_world, set_y_coordinate);
    PID_Setpoint(&Pos_yaw, target_yaw);
    control_x = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
    control_y = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
    control_z = PID_Calculate(&Pos_z_world, alt_cm, d_t);
    control_yaw = PID_Calculate(&Pos_yaw, Yaw, d_t);
    return 0;
}

int my_hovering(void)
{
    Yaw = get_yaw();

    PID_Setpoint(&Pos_yaw, target_yaw);
    PID_Setpoint(&Pos_z_world, target_alt);
    target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
    PID_Setpoint(&Vel_z, target_climb_rate);
    control_z = float_to_s16(PID_Calculate(&Vel_z, vel_Z, d_t));

    PID_Setpoint(&Pos_x_world, set_x_coordinate);
    PID_Setpoint(&Pos_y_world, set_y_coordinate);
    delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
    delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
    sin_vals = sinf(Yaw * PI_180);
    cos_vals = cosf(Yaw * PI_180);
    delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
    delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;

    PID_Setpoint(&Vel_x, delta_x_fc);
    PID_Setpoint(&Vel_y, delta_y_fc);

    control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
    control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
    control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
    return 0;
}

int my_move(void)
{
    if (task_mode == TASK_SETPOINT)
    {
        Yaw = get_yaw();
        PID_Setpoint(&Pos_z_world, (float)set_z);
        target_climb_rate = PID_Calculate(&Pos_z_world, alt_cm, d_t);
        PID_Setpoint(&Vel_z, target_climb_rate);
        control_z = float_to_s16(PID_Calculate(&Vel_z, vel_Z, d_t));

        PID_Setpoint(&Pos_x_world, (float)set_x);
        PID_Setpoint(&Pos_y_world, (float)set_y);
        PID_Setpoint(&Pos_yaw, (float)set_yaw / 100.0f);
        delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
        delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
        sin_vals = sinf(Yaw * PI_180);
        cos_vals = cosf(Yaw * PI_180);
        delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
        delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;
        PID_Setpoint(&Vel_x, delta_x_fc);
        PID_Setpoint(&Vel_y, delta_y_fc);

        control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
        control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
        control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
        return is_reach(set_threshold) && (fabsf(set_yaw / 100.0f - Yaw) <= 3);
    }
    else
    {
        switch (track_mode)
        {
        case 0:
            Yaw = get_yaw();
            PID_Setpoint(&Pos_x_img, (float)cam1_x_center);
            PID_Setpoint(&Pos_y_img, (float)cam1_y_center);
            PID_Setpoint(&Vel_z_img, (float)set_z);
            PID_Setpoint(&Pos_yaw, (float)set_yaw / 100.0f);
            // float y_img_dir = PID_Calculate(&Pos_x_img, (float)set_x, d_t);
            // float sin_val = sinf(-Yaw * PI_180);
            // float cos_val = cosf(-Yaw * PI_180);
            // float z_img_vel = vel_X * cos_val + vel_Y * sin_val; // 图像坐标系下的z轴速度
            // float z_img_dir = PID_Calculate(&Vel_z_img, z_img_vel, d_t);
            // control_x = float_to_s16(z_img_dir * cos_val - y_img_dir * sin_val);
            // control_y = float_to_s16(z_img_dir * sin_val + y_img_dir * cos_val);
            control_x = float_to_s16(PID_Calculate(&Vel_z_img, vel_X, d_t));
            control_y = float_to_s16(PID_Calculate(&Pos_x_img, (float)set_x, d_t));
            control_z = float_to_s16(PID_Calculate(&Pos_y_img, (float)set_y, d_t));
            control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
            break;
        case 1:
            // 图像坐标系下x->set_yaw,y->set_y,Δvy->set_x,Δz->set_z 如绕杆set_x=左移速度,set_z=前后速度,set_yaw=转向速度,set_y=500时定高,不等于0时上下移动
            // 定高转向:set_x=0, set_z=0, set_yaw=cam1_x_center±(vel), set_y=500
            PID_Setpoint(&Vel_z_img, (float)set_z);
            control_x = float_to_s16(PID_Calculate(&Vel_z_img, vel_X, d_t));
            PID_Setpoint(&Vel_yaw, (float)cam1_x_center);
            control_yaw = float_to_s16(PID_Calculate(&Vel_yaw, (float)set_yaw, d_t));
            if (set_y == 500)
            {
                PID_Setpoint(&Pos_z_world, target_alt);
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
            }
            else
            {
                PID_Setpoint(&Pos_y_img, (float)cam1_y_center);
                control_z = float_to_s16(PID_Calculate(&Pos_y_img, (float)set_y, d_t));
            }
            PID_Setpoint(&Vel_y, (float)set_x);
            control_y = float_to_s16(PID_Calculate(&Vel_y, (float)vel_Y, d_t));
            break;
        case 2:
            // 图像坐标系下x->set_yaw,y->set_x,Δvy->set_y,Δz->set_z 如巡线set_x=左移速度,set_y=前后速度,set_yaw=转向速度(700时不转向，只平移),set_z=500时定高,不等于0时上下移动
            // 底部摄像头定高跟踪：set_z=-1, set_x=x, set_y=y, set_yaw=700
            // 巡线时: set_y=cam2_y_center±(vel), set_yaw=x, set_z=-1, set_x=0
            if (set_y == cam2_y_center && set_x == cam2_x_center)
            {
                Yaw = get_yaw();
                PID_Setpoint(&Pos_x_world, (float)set_x_coordinate);
                PID_Setpoint(&Pos_y_world, (float)set_y_coordinate);
                delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
                delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
                sin_vals = sinf(Yaw * PI_180);
                cos_vals = cosf(Yaw * PI_180);
                delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
                delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;
                PID_Setpoint(&Vel_x, delta_x_fc);
                PID_Setpoint(&Vel_y, delta_y_fc);

                control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
                control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
            }
            else
            {
                PID_Setpoint(&Pos_y_img, cam2_y_center);
                control_x = float_to_s16(PID_Calculate(&Pos_y_img, set_y, d_t));
            }
            if (set_yaw == 700)
            {
                Yaw = get_yaw();
                if (set_x != cam2_x_center || set_y != cam2_y_center)
                {
                    PID_Setpoint(&Pos_x_img, (float)cam2_x_center);
                    control_y = float_to_s16(PID_Calculate(&Pos_x_img, (float)set_x, d_t));
                }
                PID_Setpoint(&Pos_yaw, target_yaw);
                control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
            }
            else
            {
                PID_Setpoint(&Vel_yaw, (float)cam2_x_center);
                control_yaw = float_to_s16(PID_Calculate(&Vel_yaw, (float)set_yaw, d_t));
                PID_Setpoint(&Vel_y, (float)set_x);
                control_y = float_to_s16(PID_Calculate(&Vel_y, (float)vel_Y, d_t));
            }

            if (set_z == -1)
            {
                PID_Setpoint(&Pos_z_world, target_alt);
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
            }
            else
            {
                PID_Setpoint(&Pos_z_world, (float)set_z);
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
            }
            break;
        case -1:
            // 绕圆环,set_x = x, set_y = y, set_z = z, set_yaw = radius
            switch (task_state)
            {
            case 0:
                target_angle = get_angle((float)set_x - x_coordinate, (float)set_y - y_coordinate);
                PID_Setpoint(&Pos_yaw, target_angle);
                target_xs1 = (float)set_x - (float)set_yaw * cosf(target_angle * PI_180);
                target_ys1 = (float)set_y - (float)set_yaw * sinf(target_angle * PI_180);
                PID_Setpoint(&Pos_z_world, (float)set_z);
                PID_Setpoint(&Pos_x_world, target_xs1);
                PID_Setpoint(&Pos_y_world, target_ys1);
                task_state = 1;
                task_count = 0;
                break;
            case 1:
                Yaw = get_yaw();
                control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
                delta_x_world = PID_Calculate(&Pos_x_world, (float)x_coordinate, d_t);
                delta_y_world = PID_Calculate(&Pos_y_world, (float)y_coordinate, d_t);
                sin_vals = sinf(Yaw * PI_180);
                cos_vals = cosf(Yaw * PI_180);
                delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
                delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;
                PID_Setpoint(&Vel_x, delta_x_fc);
                PID_Setpoint(&Vel_y, delta_y_fc);
                control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
                control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
                if (my_ABS(control_x) <= 6 &&
                    my_ABS(control_y) <= 6 &&
                    my_ABS(control_z) <= 3 &&
                    my_ABS(control_yaw) <= 3)
                {
                    task_state = 2;
                    target_angle += 45;
                    if (target_angle > 180.0f)
                    {
                        target_angle = -180.0f;
                    }
                    target_xs1 = (float)set_x - (float)set_yaw * cosf(target_angle * PI_180);
                    target_ys1 = (float)set_y - (float)set_yaw * sinf(target_angle * PI_180);
                    PID_Setpoint(&Pos_x_world, target_xs1);
                    PID_Setpoint(&Pos_y_world, target_ys1);
                    PID_Setpoint(&Pos_yaw, target_angle);
                }
                break;
            case 2:
                Yaw = get_yaw();
                if (fabsf(target_xs1 - x_coordinate) <= 12 && fabsf(target_ys1 - y_coordinate) <= 12)
                {
                    target_angle += 45;
                    task_count += 1;
                    if (task_count == 8)
                    {
                        target_angle = 0;
                        task_count = 0;
                        task_state = 0;
                        return 2;
                    }
                    if (target_angle > 180.0f)
                    {
                        target_angle = -180.0f;
                    }
                    PID_Setpoint(&Pos_yaw, target_angle);
                    target_xs1 = (float)set_x - (float)set_yaw * cosf(target_angle * PI_180);
                    target_ys1 = (float)set_y - (float)set_yaw * sinf(target_angle * PI_180);
                    PID_Setpoint(&Pos_x_world, target_xs1);
                    PID_Setpoint(&Pos_y_world, target_ys1);
                }
                delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
                delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
                sin_vals = sinf(Yaw * PI_180);
                cos_vals = cosf(Yaw * PI_180);
                delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
                delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;
                PID_Setpoint(&Vel_x, delta_x_fc);
                PID_Setpoint(&Vel_y, delta_y_fc);
                control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
                control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
                control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
                break;
            default:
                break;
            }
            break;
        case -2:
            // 走矩形set_x x方向边长 set_y y方向边长 set_z 高度 set_yaw 未启用
            if (task_state != 4)
            {
                if (task_state == 0)
                {
                    target_yaw = 0.0f;
                    target_xs1 = (float)set_x;
                    target_ys1 = 0.0f;
                }
                else if (task_state == 1)
                {
                    target_yaw = 90.0f;
                    target_xs1 = (float)set_x;
                    target_ys1 = (float)set_y;
                }
                else if (task_state == 2)
                {
                    target_yaw = 180.0f;
                    target_xs1 = 0.0f;
                    target_ys1 = (float)set_y;
                }
                else if (task_state == 3)
                {
                    target_yaw = -90.0f;
                    target_xs1 = 0.0f;
                    target_ys1 = 0.0f;
                }
                Yaw = get_yaw();
                PID_Setpoint(&Pos_z_world, (float)set_z);
                PID_Setpoint(&Pos_x_world, target_xs1);
                PID_Setpoint(&Pos_y_world, target_ys1);
                PID_Setpoint(&Pos_yaw, target_yaw);
                delta_x_world = PID_Calculate(&Pos_x_world, x_coordinate, d_t);
                delta_y_world = PID_Calculate(&Pos_y_world, y_coordinate, d_t);
                sin_vals = sinf(Yaw * PI_180);
                cos_vals = cosf(Yaw * PI_180);
                delta_x_fc = delta_x_world * cos_vals + delta_y_world * sin_vals;
                delta_y_fc = -delta_x_world * sin_vals + delta_y_world * cos_vals;
                PID_Setpoint(&Vel_x, delta_x_fc);
                PID_Setpoint(&Vel_y, delta_y_fc);

                control_x = float_to_s16(PID_Calculate(&Vel_x, vel_X, d_t));
                control_y = float_to_s16(PID_Calculate(&Vel_y, vel_Y, d_t));
                control_z = float_to_s16(PID_Calculate(&Pos_z_world, alt_cm, d_t));
                control_yaw = float_to_s16(PID_Calculate(&Pos_yaw, Yaw, d_t));
                if (fabs(target_xs1 - x_coordinate) < 10.0f && fabs(target_ys1 - y_coordinate) < 10.0f)
                {
                    task_state += 1;
                }
            }
            else
            {
                if (my_landing())
                {
                    landed_flag = 1;
                    task_start_flag = 0;
                    hover_start_flag = 0;
                    task_ok = 1;
                    return 0;
                }
            }
            break;
        case -3:
            control_x = set_x;
            control_y = set_y;
            control_z = set_z;
            control_yaw = set_yaw;

            break;
        default:
            break;
        }
        return 0;
    }
}

void my_task_run(void)
{
    switch (task_mode)
    {
    case TASK_TAKEOFF:
        if (my_takeoff())
        {
            task_start_flag = 0;
            hover_start_flag = 1;
            target_yaw = get_yaw();
            target_alt = (float)takeoff_height;
            set_x_coordinate = x_coordinate;
            set_y_coordinate = y_coordinate;
            task_ok = 1;
        }
        break;
    case TASK_LANDING:
        if (my_landing())
        {
            landed_flag = 1;
            task_start_flag = 0;
            hover_start_flag = 0;
            task_ok = 1;
        }
        break;
    case TASK_HOVERING:
        if (my_hovering())
        {
            task_start_flag = 0;
            task_ok = 1;
        }
        break;
    case TASK_SETPOINT:
    case TASK_TRACK:
        result = my_move();
        if (result == 1)
        {
            task_start_flag = 0;
            hover_start_flag = 1;
            target_yaw = set_yaw / 100.0f;
            target_alt = alt_cm;
            set_x_coordinate = (float)set_x;
            set_y_coordinate = (float)set_y;
            task_ok = 1;
        }
        else if (result == 2)
        {
            if (cam2_placement != 1)
            {
                task_mode = TASK_LANDING;
                set_x = float_to_s16(x_coordinate);
                set_y = float_to_s16(y_coordinate);
            }
            else
            {
                task_start_flag = 0;
                hover_start_flag = 1;
                target_yaw = get_yaw();
                target_alt = alt_cm;
                set_x_coordinate = x_coordinate;
                set_y_coordinate = y_coordinate;
                task_ok = 1;
            }
        }
        break;
    default:
        task_start_flag = 0;
        hover_start_flag = 1;
        target_yaw = get_yaw();
        target_alt = alt_cm;
        set_x_coordinate = x_coordinate;
        set_y_coordinate = y_coordinate;
        break;
    }
}

void PID_Init(my_PID_t *PID, float Kp, float Ki, float Kd, float output_min, float output_max)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->output_min = output_min;
    PID->output_max = output_max;
    PID->integral_min = output_min;
    PID->integral_max = output_max;
    PID->integral = 0;
    PID->prev_error = 0;
    PID->prev_derivative = 0;
    PID->prev_output = 0;
    PID->setpoint = 0;
    PID->output = 0;
    PID->is_saturated = 0;
    PID->error_sum = 0;
}

void PID_Reset(my_PID_t *PID)
{
    PID->integral = 0;
    PID->prev_error = 0;
    PID->prev_derivative = 0;
    PID->prev_output = 0;
    PID->output = 0;
    PID->is_saturated = 0;
    PID->error_sum = 0;
}

void PID_Setpoint(my_PID_t *PID, float setpoint)
{
    PID->setpoint = setpoint;
}

float PID_Calculate(my_PID_t *PID, float current_value, float dt)
{
    float error = PID->setpoint - current_value;
    if (PID == &Pos_yaw)
    {
        if (error > 180.0f)
        {
            error = error - 360.0f;
        }
        else if (error < -180.0f)
        {
            error = error + 360.0f;
        }
    }
    PID->error_sum += fabsf(error);

    float proportional = PID->Kp * error;

    if (!((PID->is_saturated == 1 && error > 0) ||
          (PID->is_saturated == -1 && error < 0)))
    {
        // 没有饱和或误差方向与饱和方向相反时，才更新积分
        PID->integral += error * PID->Ki * dt;
    }

    if (PID->integral > PID->integral_max)
    {
        PID->integral = PID->integral_max;
    }
    else if (PID->integral < PID->integral_min)
    {
        PID->integral = PID->integral_min;
    }

    float integral_term = PID->integral;

    float derivative = (error - PID->prev_error) / dt;
    float derivative_term = PID->Kd * derivative;

    // 一阶低通滤波（平滑微分项）
    derivative_term = alpha * derivative_term + (1 - alpha) * PID->prev_derivative;
    PID->prev_derivative = derivative_term;

    float output_unclamped = proportional + integral_term + derivative_term;

    if (output_unclamped >= PID->output_max)
    {
        PID->output = PID->output_max;
        PID->is_saturated = 1;
    }
    else if (output_unclamped <= PID->output_min)
    {
        PID->output = PID->output_min;
        PID->is_saturated = -1;
    }
    else
    {
        PID->output = output_unclamped;
        PID->is_saturated = 0;
    }

    PID->prev_error = error;
    PID->prev_output = PID->output;

    return PID->output;
}

float PID_CalculateTrapezoidal(my_PID_t *PID, float current_value, float dt)
{
    float error = PID->setpoint - current_value;
    PID->error_sum += fabsf(error);

    float proportional = PID->Kp * error;

    // 梯形积分，精度更高
    float trapezoidal = 0.5f * (error + PID->prev_error) * dt;
    PID->integral += PID->Ki * trapezoidal;

    if (PID->integral > PID->integral_max)
    {
        PID->integral = PID->integral_max;
    }
    else if (PID->integral < PID->integral_min)
    {
        PID->integral = PID->integral_min;
    }

    float integral_term = PID->integral;

    float derivative = (error - PID->prev_error) / dt;
    float derivative_term = PID->Kd * derivative;

    float output = proportional + integral_term + derivative_term;

    if (output > PID->output_max)
    {
        output = PID->output_max;
    }
    else if (output < PID->output_min)
    {
        output = PID->output_min;
    }

    PID->prev_error = error;
    PID->prev_output = output;

    return output;
}

void my_task_init(void)
{
    PID_Init(&Vel_x, 1.2, 0, 0.001, -Vx_Max, Vx_Max);
    PID_Init(&Vel_y, 1.2, 0, 0.001, -Vy_Max, Vy_Max);
    PID_Init(&Vel_z, 0.6, 0, 0, -Vz_Max, Vz_Max);
    PID_Init(&Pos_yaw, 1, 0, 0, -Vyaw_Max, Vyaw_Max);
    PID_Init(&Pos_x_world, 0.7, 0.0001, 0, -Px_Max, Px_Max);
    PID_Init(&Pos_y_world, 0.7, 0.0001, 0, -Py_Max, Py_Max);
    PID_Init(&Pos_z_world, 1, 0.0001, 0, -Pz_Max, Pz_Max);
    PID_Init(&Pos_x_img, 0.2, 0, 0, -Py_Max, Py_Max);
    PID_Init(&Pos_y_img, 0.2, 0, 0, -Py_Max, Py_Max);
    PID_Init(&Vel_z_img, 0.5, 0, 0, -Pz_Max, Pz_Max);
    PID_Init(&Vel_yaw, 0.4, 0, 0, -Vyaw_Max, Vyaw_Max);
}
