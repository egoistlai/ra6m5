#include "SysConfig.h"
#include "ANO_LX.h"
//#include "Nano.h"

#define Vx_Max 70
#define Vy_Max 70
#define Vz_Max 80
#define Px_Max 90
#define Py_Max 90
#define Pz_Max 90
#define Vyaw_Max 40
#define land_threshold 15 // 降落高度阈值，单位cm

// cam1竖放，正面朝向机头，向右为x，向下为y(yaw为0时x对应世界坐标系下y，y对应世界坐标系下z)
// (1)cam2朝下放，cam2正上方与机架左侧平齐，坐标系以右上角为起点向左为x，向下为y(yaw为0时x对应世界坐标系下x，y对应世界坐标系下y)
// (2)cam2 不启用
#define cam2_placement 2
#define cam1_x_center 320
#define cam1_y_center 240
#define cam2_x_center 313
#define cam2_y_center 187
#define PI_180 0.0174533f

#define NOISE_WINDOW_SIZE 50  // 噪声统计窗口大小
#define STATIC_THRESHOLD 0.5f // 静态检测阈值 (m/s²)
#define BASE_CUTOFF 0.2f      // 基础截止频率
#define MAX_DT 0.1f           // 最大允许时间间隔(s)
#define ACCEL_SCALE 1.37f     // 加速度缩放因子（需校准）

typedef struct
{
    float Kp; ///< 比例增益
    float Ki; ///< 积分增益
    float Kd; ///< 微分增益

    float output_min;   ///< 最小输出限制
    float output_max;   ///< 最大输出限制
    float integral_min; ///< 积分最小值
    float integral_max; ///< 积分最大值

    float integral;   ///< 积分项累积值
    float prev_error; ///< 上一时刻的误差

    float prev_derivative; ///< 上一时刻的微分项（用于滤波）
    float prev_output;     ///< 上一时刻的输出（用于抗饱和）

    float setpoint; ///< 目标设定值
    float output;   ///< 当前输出值

    s8 is_saturated; ///< 饱和标志: 1=正饱和, -1=负饱和, 0=未饱和
    float error_sum; ///< 误差总和（用于诊断）
} my_PID_t;

typedef struct
{
    float vx;      // x轴速度 (cm/s)
    float vy;      // y轴速度 (cm/s)
    float quality; // 数据质量 (0.0-1.0)
    u32 timestamp; // 时间戳 (ms)
} OpticalFlowData;

typedef struct
{
    float vx;          // x轴速度 (cm/s)
    float vy;          // y轴速度 (cm/s)
    float ax;          // x轴加速度 (cm/s^2)
    float ay;          // y轴加速度 (cm/s^2)
    u32 timestamp;     // 时间戳 (ms)
    u32 acc_timestamp; // 加速度时间戳 (ms)
} IMUData;

typedef struct
{
    float x_lidar;   // 激光雷达x轴位置 (cm)
    float y_lidar;   // 激光雷达y轴位置 (cm)
    float z_lidar;   // 激光雷达z轴位置 (cm)
    float yaw_lidar; // 激光雷达航向角 (度)
    u32 timestamp;   // 时间戳 (ms)
} LidarData;

typedef struct
{
    float velocity_x_fused; // 融合后的x轴速度 (cm/s)
    float velocity_y_fused; // 融合后的y轴速度 (cm/s)
    float displacement_x;   // x轴估计位移 (cm)
    float displacement_y;   // y轴估计位移 (cm)
    u32 last_timestamp;     // 最后更新时间戳
    u8 radar_updated;       // 雷达更新标志位
} StateEstimator;

typedef struct
{
    OpticalFlowData flow;
    IMUData imu;
    LidarData lidar;
    u8 flow_ready;
    u8 imu_ready;
} SensorBuffer;

typedef struct
{
    float window[NOISE_WINDOW_SIZE]; // 滑动窗口缓冲区
    float sum;                       // 当前窗口和
    float sum_sq;                    // 当前窗口平方和
    int index;                       // 当前窗口索引
    int count;                       // 当前有效数据计数
} NoiseStats;

enum
{
    TASK_TAKEOFF = 1,
    TASK_LANDING,
    TASK_HOVERING,
    TASK_SETPOINT,
    TASK_TRACK,
};

extern s16 control_x;
extern s16 control_y;
extern s16 control_z;
extern s16 control_yaw;
extern s16 set_x;
extern s16 set_y;
extern s16 set_z;
extern s16 set_yaw;
extern s16 track_mode;
extern u8 task_start_flag;
extern u8 listen_flag;
extern u8 task_mode;
extern u8 task_ok;
extern u8 landed_flag;
extern u8 init_yaw_flag;
extern u8 hover_start_flag;
extern u8 takeoff_phase;
extern u16 takeoff_height;
extern u32 my_time_stamp;
extern SensorBuffer sensor_buffer;
extern StateEstimator state_est;
extern float init_Yaw;
extern float alt_cm;
extern float set_x_coordinate;
extern float set_y_coordinate;
extern float x_coordinate;
extern float y_coordinate;
extern float target_yaw;
extern float target_alt;

void my_task_run(void);
void update_imu(void);
void update_acc(float x_bias, float y_bias);
void my_task_init(void);
void updateStateEstimator(SensorBuffer *buffer, StateEstimator *state);
void PID_Setpoint(my_PID_t *PID, float setpoint);
float PID_Calculate(my_PID_t *PID, float current_value, float dt);
float PID_CalculateTrapezoidal(my_PID_t *PID, float current_value, float dt);
float get_yaw(void);
s16 float_to_s16(float value);
u8 update_of(void);
int my_hovering(void);
