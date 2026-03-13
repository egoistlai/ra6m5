#ifndef __ANO_LX_H
#define __ANO_LX_H
//==引用
#include "SysConfig.h"

//==定义/声明

enum
{
    ch_1_rol = 0,
    ch_2_pit,
    ch_3_thr,
    ch_4_yaw,
    ch_5_aux1,
    ch_6_aux2,
    ch_7_aux3,
    ch_8_aux4,
    ch_9_aux5,
    ch_10_aux6,
};

// 0x40
typedef struct
{
    s16 ch_[10]; //

} __attribute__((__packed__)) _rc_ch_st;

typedef union
{
    u8 byte_data[20];
    _rc_ch_st st_data;
} _rc_ch_un;

// 0x41
typedef struct
{
    s16 rol;
    s16 pit;
    s16 thr;
    s16 yaw_dps;
    s16 vel_x;
    s16 vel_y;
    s16 vel_z;

} __attribute__((__packed__)) _rt_tar_st;

typedef union
{
    u8 byte_data[14];
    _rt_tar_st st_data;
} _rt_tar_un;

// 0x0D
typedef struct
{
    u16 voltage_100;
    u16 current_100;

} __attribute__((__packed__)) _fc_bat_st;

typedef union
{
    u8 byte_data[4];
    _fc_bat_st st_data;
} _fc_bat_un;

// 0x01
typedef struct
{
    s16 acc_x;
    s16 acc_y;
    s16 acc_z;
    u16 gyr_x;
    u16 gyr_y;
    u16 gyr_z;
    u8 shock_sta;

} __attribute__((__packed__)) _fc_acc_st;

typedef union
{
    u8 byte_data[13];
    _fc_acc_st st_data;
} _fc_acc_un;

// 0xF1
typedef struct
{
    s16 vel_x;
    s16 vel_y;
    s16 x;
    s16 y;
    s16 z;
    s16 yaw;
    s16 control_x;
    s16 control_y;
    s16 control_z;
    s16 control_yaw;
} __attribute__((__packed__)) _fc_my_st;

typedef union
{
    u8 byte_data[20];
    _fc_my_st st_data;
} _fc_my_un;

// 0xF2
typedef struct
{
    s16 lidar_x;
    s16 lidar_y;
    s16 lidar_z;
    s16 lidar_yaw;
} __attribute__((__packed__)) _fc_my1_st;

typedef union
{
    u8 byte_data[8];
    _fc_my1_st st_data;
} _fc_my1_un;

// 0x03
typedef struct
{
    s16 rol_x100;
    s16 pit_x100;
    s16 yaw_x100;
    u8 state;
} __attribute__((__packed__)) _fc_att_st;

typedef union
{
    u8 byte_data[7];
    _fc_att_st st_data;
} _fc_att_un;

// 0x04
typedef struct
{
    s16 w_x10000;
    s16 x_x10000;
    s16 y_x10000;
    s16 z_x10000;
    u8 state;
} __attribute__((__packed__)) _fc_att_qua_st;

typedef union
{
    u8 byte_data[9];
    _fc_att_qua_st st_data;
} _fc_att_qua_un;

// 0x07
typedef struct
{
    s16 vel_x;
    s16 vel_y;
    s16 vel_z;

} __attribute__((__packed__)) _fc_vel_st;

typedef union
{
    u8 byte_data[6];
    _fc_vel_st st_data;
} _fc_vel_un;
// 0x05
typedef struct
{
    s32 alt_fu;
    s32 alt_add;
    u8 alt_sta;

} __attribute__((__packed__)) _fc_alt_st;

typedef union
{
    u8 byte_data[9];
    _fc_alt_st st_data;
} _fc_alt_un;

typedef struct
{
    u16 pwm_m1;
    u16 pwm_m2;
    u16 pwm_m3;
    u16 pwm_m4;
    u16 pwm_m5;
    u16 pwm_m6;
    u16 pwm_m7;
    u16 pwm_m8;
} _pwm_st;

//==数据声明
extern _fc_att_un fc_att;
extern _fc_att_qua_un fc_att_qua;
extern _fc_vel_un fc_vel;
extern _rt_tar_un rt_tar;
extern _fc_bat_un fc_bat;
extern _pwm_st pwm_to_esc;
extern _fc_alt_un fc_alt;
extern _fc_acc_un fc_acc;
extern _fc_my_un fc_my_data;
extern _fc_my1_un fc_my1_data;
//==函数声明
// static

// public
void ANO_LX_Task(void);

#endif
