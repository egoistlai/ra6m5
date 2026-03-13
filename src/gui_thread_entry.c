#include "gui_thread.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"

/* 引入驱动和 LVGL 接口 */
#include "drv_uart.h"
#include "drv_spi_display.h"
#include "drv_i2c_touchpad.h"
#include "drv_littlefs_log.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_demos.h" // 如果你想运行官方 Demo

#include "My_Task.h"
#include "Drv_AnoOf.h"
#include "ANO_LX.h"
#include "drv_imu.h"

#include "app_queues.h"
#include "ps2_gamepad.h"
#include <string.h>

extern volatile float g_motor_speed_rpm[4];
extern volatile float g_motor_speed_rps[4];
extern volatile int32_t g_motor_total_pulse[4];

#define LOG_PAGE_SIZE 512
static uint32_t current_log_offset = 0;

static lv_obj_t *ui_main_page;
static lv_obj_t *ui_sub_page;
static lv_obj_t *label_imu;
static lv_obj_t *label_flow;
static lv_obj_t *label_lidar;
static lv_obj_t *ui_wifi_page;
static lv_obj_t *label_status;
static lv_obj_t *ta_log; // 文本区域
static lv_obj_t *ui_log_page;
static lv_obj_t *ta_flight_log;
static lv_obj_t *ui_motor_page;
lv_obj_t *label_speed_display;
lv_obj_t *label_ps2_display;
static lv_obj_t *ui_imu_page;     // 新增 IMU 页面句柄
static lv_obj_t *label_imu_euler; // 欧拉角标签
static lv_obj_t *label_imu_accel; // 加速度标签
static lv_obj_t *label_imu_gyro;  // 角速度标签

/* 引入电机线程中定义的 PID 全局变量 */
extern volatile float g_pid_kp;
extern volatile float g_pid_ki;
extern volatile float g_pid_kd;

/* 声明用于显示滑动条数值的 Label */
static lv_obj_t *label_kp_val;
static lv_obj_t *label_ki_val;
static lv_obj_t *label_kd_val;

volatile float g_nav_target_x = 0.0f;   // 目标X坐标 (cm)
volatile float g_nav_target_y = 0.0f;   // 目标Y坐标 (cm)
volatile bool g_nav_start_flag = false; // 启动导航标志位

static lv_obj_t *ui_nav_page;
static lv_obj_t *label_nav_x_val;
static lv_obj_t *label_nav_y_val;
static lv_obj_t *label_nav_status;

/* --- 导航页面回调与构建 --- */
static void slider_nav_x_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    g_nav_target_x = (float)lv_slider_get_value(slider);
    lv_label_set_text_fmt(label_nav_x_val, "X: %d cm", (int)g_nav_target_x);
}

static void slider_nav_y_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    g_nav_target_y = (float)lv_slider_get_value(slider);
    lv_label_set_text_fmt(label_nav_y_val, "Y: %d cm", (int)g_nav_target_y);
}

static void btn_nav_start_handler(lv_event_t *e)
{
    g_nav_start_flag = true; // 触发导航控制状态机
    lv_label_set_text(label_nav_status, "#FF0000 Status: NAVIGATING...#");
}

static void btn_nav_back_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page); // 返回主页
}

static void btn_to_nav_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_nav_page); // 从主页进入导航页
}

void build_nav_page(void)
{
    ui_nav_page = lv_obj_create(NULL);

    /* 返回按钮 */
    lv_obj_t *btn_back = lv_btn_create(ui_nav_page);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_set_size(btn_back, 60, 30);
    lv_obj_add_event_cb(btn_back, btn_nav_back_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");

    /* 标题 */
    lv_obj_t *title = lv_label_create(ui_nav_page);
    lv_label_set_text(title, "Auto Navigation Test");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* --- X 轴坐标设置 --- */
    lv_obj_t *slider_x = lv_slider_create(ui_nav_page);
    lv_obj_set_size(slider_x, 160, 15);
    lv_obj_align(slider_x, LV_ALIGN_CENTER, 0, -60);
    lv_slider_set_range(slider_x, -200, 200); // -200cm 到 +200cm
    lv_slider_set_value(slider_x, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_x, slider_nav_x_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    label_nav_x_val = lv_label_create(ui_nav_page);
    lv_label_set_text(label_nav_x_val, "X: 0 cm");
    lv_obj_align_to(label_nav_x_val, slider_x, LV_ALIGN_OUT_TOP_MID, 0, -10);

    /* --- Y 轴坐标设置 --- */
    lv_obj_t *slider_y = lv_slider_create(ui_nav_page);
    lv_obj_set_size(slider_y, 160, 15);
    lv_obj_align(slider_y, LV_ALIGN_CENTER, 0, 10);
    lv_slider_set_range(slider_y, 0, 500); // 0cm 到 500cm
    lv_slider_set_value(slider_y, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_y, slider_nav_y_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    label_nav_y_val = lv_label_create(ui_nav_page);
    lv_label_set_text(label_nav_y_val, "Y: 0 cm");
    lv_obj_align_to(label_nav_y_val, slider_y, LV_ALIGN_OUT_TOP_MID, 0, -10);

    /* --- 启动按钮 --- */
    lv_obj_t *btn_start = lv_btn_create(ui_nav_page);
    lv_obj_align(btn_start, LV_ALIGN_CENTER, 0, 80);
    lv_obj_set_size(btn_start, 120, 40);
    lv_obj_add_event_cb(btn_start, btn_nav_start_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_start), "START NAV");

    /* 状态文本 */
    label_nav_status = lv_label_create(ui_nav_page);
    lv_label_set_recolor(label_nav_status, true);
    lv_label_set_text(label_nav_status, "#00FF00 Status: IDLE#");
    lv_obj_align(label_nav_status, LV_ALIGN_BOTTOM_MID, 0, -20);
}

static void slider_kp_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    g_pid_kp = lv_slider_get_value(slider) / 10.0f; // 缩放 10 倍以支持一位小数
    lv_label_set_text_fmt(label_kp_val, "%.1f", g_pid_kp);
}

static void slider_ki_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    g_pid_ki = lv_slider_get_value(slider) / 10.0f;
    lv_label_set_text_fmt(label_ki_val, "%.1f", g_pid_ki);
}

static void slider_kd_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    g_pid_kd = lv_slider_get_value(slider) / 10.0f;
    lv_label_set_text_fmt(label_kd_val, "%.1f", g_pid_kd);
}

static void btn_imu_back_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page); // 返回主页
}
static void btn_to_imu_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_imu_page); // 进入 IMU 页
}

static void imu_update_timer_cb(lv_timer_t *timer)
{
    if (lv_scr_act() != ui_imu_page)
        return;

    if (g_imu_data.is_updated)
    {
        lv_label_set_text_fmt(label_imu_euler,
                              "#FF0000 [Euler Angles]#\nRoll:  %6.2f\nPitch: %6.2f\nYaw:   %6.2f",
                              g_imu_data.euler[0], g_imu_data.euler[1], g_imu_data.euler[2]);

        lv_label_set_text_fmt(label_imu_accel,
                              "#00FF00 [Accelerometer (g)]#\nX: %6.3f\nY: %6.3f\nZ: %6.3f",
                              g_imu_data.accel[0], g_imu_data.accel[1], g_imu_data.accel[2]);

        lv_label_set_text_fmt(label_imu_gyro,
                              "#0000FF [Gyroscope (dps)]#\nX: %6.1f\nY: %6.1f\nZ: %6.1f",
                              g_imu_data.gyro[0], g_imu_data.gyro[1], g_imu_data.gyro[2]);
    }
}

void build_imu_page(void)
{
    ui_imu_page = lv_obj_create(NULL);

    /* 返回按钮 */
    lv_obj_t *btn_back = lv_btn_create(ui_imu_page);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_add_event_cb(btn_back, btn_imu_back_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");

    /* 标题 */
    lv_obj_t *title = lv_label_create(ui_imu_page);
    lv_label_set_text(title, "Precision IMU Monitor");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    /* --- 数据容器 --- */
    lv_obj_t *cont = lv_obj_create(ui_imu_page);
    lv_obj_set_size(cont, 300, 360);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    label_imu_euler = lv_label_create(cont);
    lv_label_set_recolor(label_imu_euler, true);
    lv_label_set_text(label_imu_euler, "Waiting for Euler...");

    label_imu_accel = lv_label_create(cont);
    lv_label_set_recolor(label_imu_accel, true);
    lv_label_set_text(label_imu_accel, "Waiting for Accel...");

    label_imu_gyro = lv_label_create(cont);
    lv_label_set_recolor(label_imu_gyro, true);
    lv_label_set_text(label_imu_gyro, "Waiting for Gyro...");

    /* 启动 50ms 刷新定时器 */
    lv_timer_create(imu_update_timer_cb, 50, NULL);
}

static void btn_motor_back_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page); // 返回主页
}

static void btn_to_motor_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_motor_page); // 进入电机页
}

static void motor_update_timer_cb(lv_timer_t *timer)
{
    // 仅在停留在电机页面时才刷新，节省 CPU 资源
    if (lv_scr_act() != ui_motor_page)
        return;

    // 1. 刷新上半部分的电机数据 (去掉了多余的换行，节省空间)
    if (label_speed_display != NULL)
    {
        lv_label_set_text_fmt(label_speed_display,
                              "#FF0000 M1(FL):# %6.1f RPS | Pulse: %d\n"
                              "#00FF00 M2(FR):# %6.1f RPS | Pulse: %d\n"
                              "#0000FF M3(RL):# %6.1f RPS | Pulse: %d\n"
                              "#FF00FF M4(RR):# %6.1f RPS | Pulse: %d",
                              g_motor_speed_rpm[0], g_motor_total_pulse[0],
                              g_motor_speed_rpm[1], g_motor_total_pulse[1],
                              g_motor_speed_rpm[2], g_motor_total_pulse[2],
                              g_motor_speed_rpm[3], g_motor_total_pulse[3]);
    }

    // 2. 刷新下半部分的 PS2 手柄数据
    if (label_ps2_display != NULL)
    {
        if (g_ps2_data.is_connected)
        {
            char btn_str[64] = "None"; // 用于存放按下的按键名字

            // 如果有按键按下，逐个解析 bit 位并拼接字符串
            if (g_ps2_data.buttons != 0)
            {
                btn_str[0] = '\0'; // 清空默认的 "None"

                // 方向键
                if (g_ps2_data.buttons & (1 << 4))
                    strcat(btn_str, "UP ");
                if (g_ps2_data.buttons & (1 << 6))
                    strcat(btn_str, "DOWN ");
                if (g_ps2_data.buttons & (1 << 7))
                    strcat(btn_str, "LEFT ");
                if (g_ps2_data.buttons & (1 << 5))
                    strcat(btn_str, "RIGHT ");

                // 右侧功能键
                if (g_ps2_data.buttons & (1 << 12))
                    strcat(btn_str, "Y ");
                if (g_ps2_data.buttons & (1 << 14))
                    strcat(btn_str, "A ");
                if (g_ps2_data.buttons & (1 << 15))
                    strcat(btn_str, "X ");
                if (g_ps2_data.buttons & (1 << 13))
                    strcat(btn_str, "B ");

                // 顶部扳机键
                if (g_ps2_data.buttons & (1 << 10))
                    strcat(btn_str, "L1 ");
                if (g_ps2_data.buttons & (1 << 8))
                    strcat(btn_str, "L2 ");
                if (g_ps2_data.buttons & (1 << 11))
                    strcat(btn_str, "R1 ");
                if (g_ps2_data.buttons & (1 << 9))
                    strcat(btn_str, "R2 ");

                // 控制键与摇杆按下
                if (g_ps2_data.buttons & (1 << 0))
                    strcat(btn_str, "SELECT ");
                if (g_ps2_data.buttons & (1 << 3))
                    strcat(btn_str, "START ");
                if (g_ps2_data.buttons & (1 << 1))
                    strcat(btn_str, "MODE "); // 映射在 L3
                if (g_ps2_data.buttons & (1 << 2))
                    strcat(btn_str, "R3 ");
            }

            // 更新 UI (摇杆直接显示 0~255 的整形数值)
            lv_label_set_text_fmt(label_ps2_display,
                                  "#00FFFF [PS2 Gamepad Connected]#\n"
                                  "L_Joy: X=%3d, Y=%3d\n"
                                  "R_Joy: X=%3d, Y=%3d\n"
                                  "#FFA500 Btn: %s#",
                                  g_ps2_data.left_x, g_ps2_data.left_y,
                                  g_ps2_data.right_x, g_ps2_data.right_y,
                                  btn_str);
        }
        else
        {
            // 设备拔出或未连接
            lv_label_set_text(label_ps2_display, "#808080 [PS2 Gamepad Disconnected]#\nWaiting for USB...");
        }
    }
}

void build_motor_page(void)
{
    ui_motor_page = lv_obj_create(NULL);

    /* 返回按钮 */
    lv_obj_t *btn_back = lv_btn_create(ui_motor_page);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_set_size(btn_back, 60, 30);
    lv_obj_add_event_cb(btn_back, btn_motor_back_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");

    /* 标题 */
    lv_obj_t *title = lv_label_create(ui_motor_page);
    lv_label_set_text(title, "PID Tuner & Monitor");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* --- 上半区：电机速度 --- */
    label_speed_display = lv_label_create(ui_motor_page);
    lv_obj_set_style_text_align(label_speed_display, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(label_speed_display, LV_ALIGN_TOP_MID, 0, 45); // 上移
    lv_label_set_recolor(label_speed_display, true);
    lv_label_set_text(label_speed_display, "Wait...");

    /* --- 中间区：PID 调参滑动条 --- */
    // Kp Slider (范围 0 ~ 30.0)
    lv_obj_t *slider_kp = lv_slider_create(ui_motor_page);
    lv_obj_set_size(slider_kp, 140, 10);
    lv_obj_align(slider_kp, LV_ALIGN_CENTER, 10, -10);
    lv_slider_set_range(slider_kp, 0, 100);
    lv_slider_set_value(slider_kp, (int)(g_pid_kp * 10), LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_kp, slider_kp_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *label_kp_name = lv_label_create(ui_motor_page);
    lv_label_set_text(label_kp_name, "Kp:");
    lv_obj_align_to(label_kp_name, slider_kp, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    label_kp_val = lv_label_create(ui_motor_page);
    lv_label_set_text_fmt(label_kp_val, "%.1f", g_pid_kp);
    lv_obj_align_to(label_kp_val, slider_kp, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // Ki Slider (范围 0 ~ 10.0)
    lv_obj_t *slider_ki = lv_slider_create(ui_motor_page);
    lv_obj_set_size(slider_ki, 140, 10);
    lv_obj_align(slider_ki, LV_ALIGN_CENTER, 10, 20);
    lv_slider_set_range(slider_ki, 0, 10); // 0.0 ~ 10.0
    lv_slider_set_value(slider_ki, (int)(g_pid_ki * 10), LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_ki, slider_ki_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *label_ki_name = lv_label_create(ui_motor_page);
    lv_label_set_text(label_ki_name, "Ki:");
    lv_obj_align_to(label_ki_name, slider_ki, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    label_ki_val = lv_label_create(ui_motor_page);
    lv_label_set_text_fmt(label_ki_val, "%.1f", g_pid_ki);
    lv_obj_align_to(label_ki_val, slider_ki, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // Kd Slider (范围 0 ~ 5.0)
    lv_obj_t *slider_kd = lv_slider_create(ui_motor_page);
    lv_obj_set_size(slider_kd, 140, 10);
    lv_obj_align(slider_kd, LV_ALIGN_CENTER, 10, 50);
    lv_slider_set_range(slider_kd, 0, 50); // 0.0 ~ 5.0
    lv_slider_set_value(slider_kd, (int)(g_pid_kd * 10), LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_kd, slider_kd_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *label_kd_name = lv_label_create(ui_motor_page);
    lv_label_set_text(label_kd_name, "Kd:");
    lv_obj_align_to(label_kd_name, slider_kd, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    label_kd_val = lv_label_create(ui_motor_page);
    lv_label_set_text_fmt(label_kd_val, "%.1f", g_pid_kd);
    lv_obj_align_to(label_kd_val, slider_kd, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    /* --- 下半区：手柄数据 --- */
    label_ps2_display = lv_label_create(ui_motor_page);
    lv_obj_set_style_text_align(label_ps2_display, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label_ps2_display, LV_ALIGN_BOTTOM_MID, 0, -5); // 贴底
    lv_label_set_recolor(label_ps2_display, true);
    lv_label_set_text(label_ps2_display, "Loading PS2...");

    /* 启动 50ms 刷新定时器 */
    lv_timer_create(motor_update_timer_cb, 50, NULL);
}

static void load_log_page(void)
{
    static char read_buf[LOG_PAGE_SIZE];
    memset(read_buf, 0, sizeof(read_buf));

    // 调用新的分页读取接口
    uint32_t len = drv_log_read_page(read_buf, sizeof(read_buf), current_log_offset);

    if (len > 0)
    {
        lv_textarea_set_text(ta_flight_log, read_buf);
    }
    else
    {
        if (current_log_offset > 0)
        {
            lv_textarea_set_text(ta_flight_log, "--- TOP OF LOG ---\nNo older logs available.");
        }
        else
        {
            lv_textarea_set_text(ta_flight_log, "Log is empty.");
        }
    }
}

static void wifi_ui_timer_cb(lv_timer_t *timer)
{
    // 如果不在 WiFi 页面，不刷新
    if (lv_scr_act() != ui_wifi_page)
        return;

    /* 1. 更新状态文字 */
    switch (g_wifi_status)
    {
    case WIFI_STATUS_DISCONNECTED:
        lv_label_set_text(label_status, "Status: Disconnected");
        break;
    case WIFI_STATUS_CONNECTING:
        lv_label_set_text(label_status, "Status: Connecting...");
        break;
    case WIFI_STATUS_CONNECTED:
        lv_label_set_text(label_status, "Status: Connected (Echo)");
        break;
    case WIFI_STATUS_ERROR:
        lv_label_set_text(label_status, "Status: Error!");
        break;
    }

    /* 2. 从日志队列读取数据并追加到文本框 */
    char ch;
    char str_buf[2] = {0, 0};

    // 每次最多处理 20 个字符，防止卡死 UI
    for (int i = 0; i < 20; i++)
    {
        if (xQueueReceive(g_ui_log_queue, &ch, 0) == pdPASS)
        {
            str_buf[0] = ch;
            lv_textarea_add_text(ta_log, str_buf);
        }
        else
        {
            break;
        }
    }
}

static void btn_connect_handler(lv_event_t *e)
{
    if (g_wifi_status == WIFI_STATUS_DISCONNECTED || g_wifi_status == WIFI_STATUS_ERROR)
    {
        lv_textarea_set_text(ta_log, ""); // 清空日志
        g_wifi_connect_req = true;        // 发送请求给业务线程
    }
}

/* 按钮回调：返回主页 */
static void btn_back_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page);
}

// 按钮回调：看更老的日志 (往前翻页)
static void btn_older_handler(lv_event_t *e)
{
    current_log_offset += (LOG_PAGE_SIZE - 1); // 偏移量增加
    load_log_page();
}

// 按钮回调：看更新的日志 (往后翻页)
static void btn_newer_handler(lv_event_t *e)
{
    if (current_log_offset >= (LOG_PAGE_SIZE - 1))
    {
        current_log_offset -= (LOG_PAGE_SIZE - 1); // 偏移量减少
    }
    else
    {
        current_log_offset = 0; // 回到最新
    }
    load_log_page();
}

// 按钮回调：退出日志页面
static void btn_log_back_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page);
}

// 主页按钮回调：进入日志页面
static void btn_to_log_event_handler(lv_event_t *e)
{
    current_log_offset = 0; // 每次进去，强制从最新的一页开始看
    lv_scr_load(ui_log_page);
    load_log_page();
}

// 构建日志页面 UI
void build_log_page(void)
{
    ui_log_page = lv_obj_create(NULL);

    lv_obj_t *title = lv_label_create(ui_log_page);
    lv_label_set_text(title, "Flight Logs (Paged)");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    ta_flight_log = lv_textarea_create(ui_log_page);
    lv_obj_set_size(ta_flight_log, 300, 360); // 稍微调高一点文本框
    lv_obj_align(ta_flight_log, LV_ALIGN_CENTER, 0, -15);
    // 禁用光标点击，防止滑动时长按误触弹出虚拟键盘
    lv_textarea_set_cursor_click_pos(ta_flight_log, false);

    /* === 底部横排三个按钮 === */
    // 1. Older 按钮 (左边)
    lv_obj_t *btn_older = lv_btn_create(ui_log_page);
    lv_obj_align(btn_older, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(btn_older, btn_older_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_older), "< Older");

    // 2. Newer 按钮 (中间)
    lv_obj_t *btn_newer = lv_btn_create(ui_log_page);
    lv_obj_align(btn_newer, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_newer, btn_newer_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_newer), "Newer >");

    // 3. Back 按钮 (右边)
    lv_obj_t *btn_back = lv_btn_create(ui_log_page);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(btn_back, btn_log_back_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");
}

/* 构建 WiFi 页面 */
void build_wifi_page(void)
{
    ui_wifi_page = lv_obj_create(NULL);

    /* 标题 */
    lv_obj_t *title = lv_label_create(ui_wifi_page);
    lv_label_set_text(title, "WiFi TCP Echo Test");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* 状态栏 */
    label_status = lv_label_create(ui_wifi_page);
    lv_label_set_text(label_status, "Status: Ready");
    lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 40);

    /* 日志显示区 */
    ta_log = lv_textarea_create(ui_wifi_page);
    lv_obj_set_size(ta_log, 280, 250);
    lv_obj_align(ta_log, LV_ALIGN_CENTER, 0, 10);
    lv_textarea_set_cursor_click_pos(ta_log, false); // 禁止点击光标
    lv_textarea_set_text(ta_log, "Log started...\n");

    /* 连接按钮 */
    lv_obj_t *btn_conn = lv_btn_create(ui_wifi_page);
    lv_obj_align(btn_conn, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_set_size(btn_conn, 100, 40);
    lv_obj_add_event_cb(btn_conn, btn_connect_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_conn), "Connect");

    /* 返回按钮 */
    lv_obj_t *btn_back = lv_btn_create(ui_wifi_page);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_add_event_cb(btn_back, btn_back_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");

    /* 启动刷新定时器 */
    lv_timer_create(wifi_ui_timer_cb, 50, NULL);
}

static void update_data_timer_cb(lv_timer_t *timer)
{
    // 仅在副页面刷新，节省资源
    if (lv_scr_act() != ui_sub_page)
        return;

    /* --- 进入临界区：保护数据读取 --- */
    taskENTER_CRITICAL();

    // 1. 读取光流数据 (来自 Drv_AnoOf)
    int16_t of_dx = ano_of.of1_dx;
    int16_t of_dy = ano_of.of1_dy;
    uint8_t of_qual = ano_of.of_quality;

    // 2. 读取雷达数据 (来自 My_Task / Lidar)
    // 根据您的 My_Task.h，sensor_buffer.lidar 包含 float 类型数据
    float lid_x = sensor_buffer.lidar.x_lidar;
    float lid_y = sensor_buffer.lidar.y_lidar;
    float lid_z = sensor_buffer.lidar.z_lidar; // 高度

    // 3. 读取飞控姿态 (来自 ANO_LX)
    // 利用联合体直接读取 st_data，不需要手动拼字节
    // 协议定义 rol_x100 代表 放大100倍的角度值
    float roll = fc_att.st_data.rol_x100 / 100.0f;
    float pitch = fc_att.st_data.pit_x100 / 100.0f;
    float yaw = fc_att.st_data.yaw_x100 / 100.0f;

    /* --- 退出临界区 --- */
    taskEXIT_CRITICAL();

    // --- 更新 UI 显示 (支持 LVGL 颜色格式) ---
    lv_label_set_text_fmt(label_imu,
                          "#FF0000 [FC Att]#\nRoll:  %.2f\nPitch: %.2f\nYaw:   %.2f",
                          roll, pitch, yaw);

    lv_label_set_text_fmt(label_flow,
                          "#0000FF [OptFlow]#\nDX: %d\nDY: %d\nQual: %u",
                          of_dx, of_dy, of_qual);

    lv_label_set_text_fmt(label_lidar,
                          "#008000 [Lidar]#\nHeight: %.2f\nPos X:  %.2f\nPos Y:  %.2f",
                          lid_z, lid_x, lid_y);
}

/* --- 5. 页面跳转逻辑 --- */
static void btn_to_wifi_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_wifi_page);
}

static void btn_to_monitor_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_sub_page);
}

static void btn_back_event_handler(lv_event_t *e)
{
    lv_scr_load(ui_main_page);
}

void build_pages(void)
{
    // --- 主页面 ---
    ui_main_page = lv_obj_create(NULL);
    lv_obj_t *btn = lv_btn_create(ui_main_page);
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_to_monitor_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn), "Enter Monitor");

    lv_obj_t *title = lv_label_create(ui_main_page);
    lv_label_set_text(title, "Flight Control System");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

    /* 按钮 2: 进入 WiFi 测试 (新增) */
    lv_obj_t *btn_wifi = lv_btn_create(ui_main_page);
    lv_obj_set_size(btn_wifi, 140, 50);
    lv_obj_align(btn_wifi, LV_ALIGN_CENTER, 0, 80); // 向下偏移
    lv_obj_add_event_cb(btn_wifi, btn_to_wifi_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl_wifi = lv_label_create(btn_wifi);
    lv_label_set_text(lbl_wifi, "WiFi Test");
    lv_obj_center(lbl_wifi);

    lv_obj_t *btn_log = lv_btn_create(ui_main_page);
    lv_obj_align(btn_log, LV_ALIGN_CENTER, 0, 140);
    lv_obj_add_event_cb(btn_log, btn_to_log_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_log), "View Flight Logs");

    lv_obj_t *btn_motor = lv_btn_create(ui_main_page);
    lv_obj_align(btn_motor, LV_ALIGN_CENTER, 0, 200);
    lv_obj_add_event_cb(btn_motor, btn_to_motor_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_motor), "Motor Test");

    lv_obj_t *btn_imu = lv_btn_create(ui_main_page);
    lv_obj_align(btn_imu, LV_ALIGN_CENTER, 0, 260); // y偏移继续往下排
    lv_obj_add_event_cb(btn_imu, btn_to_imu_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_imu), "IMU Sensor");

    lv_obj_t *btn_to_nav = lv_btn_create(ui_main_page);
    lv_obj_align(btn_to_nav, LV_ALIGN_CENTER, 0, 320); // y偏移继续往下排
    lv_obj_add_event_cb(btn_to_nav, btn_to_nav_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_to_nav), "To Nav");
    // --- 副页面 ---
    ui_sub_page = lv_obj_create(NULL);

    // 返回按钮
    lv_obj_t *btn_back = lv_btn_create(ui_sub_page);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_add_event_cb(btn_back, btn_back_event_handler, LV_EVENT_CLICKED, NULL);
    lv_label_set_text(lv_label_create(btn_back), "Back");

    // 数据容器
    lv_obj_t *cont = lv_obj_create(ui_sub_page);
    lv_obj_set_size(cont, 300, 380);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    // 创建标签
    label_imu = lv_label_create(cont);
    lv_label_set_recolor(label_imu, true);
    lv_obj_set_style_bg_color(label_imu, lv_palette_lighten(LV_PALETTE_RED, 4), 0);
    lv_obj_set_style_bg_opa(label_imu, LV_OPA_COVER, 0);
    lv_obj_set_width(label_imu, LV_PCT(100));
    lv_label_set_text(label_imu, "IMU: Wait...");

    label_flow = lv_label_create(cont);
    lv_label_set_recolor(label_flow, true);
    lv_obj_set_style_bg_color(label_flow, lv_palette_lighten(LV_PALETTE_BLUE, 4), 0);
    lv_obj_set_style_bg_opa(label_flow, LV_OPA_COVER, 0);
    lv_obj_set_width(label_flow, LV_PCT(100));
    lv_label_set_text(label_flow, "OF: Wait...");

    label_lidar = lv_label_create(cont);
    lv_label_set_recolor(label_lidar, true);
    lv_obj_set_style_bg_color(label_lidar, lv_palette_lighten(LV_PALETTE_GREEN, 4), 0);
    lv_obj_set_style_bg_opa(label_lidar, LV_OPA_COVER, 0);
    lv_obj_set_width(label_lidar, LV_PCT(100));
    lv_label_set_text(label_lidar, "Lidar: Wait...");

    // 创建定时器，100ms 刷新一次
    lv_timer_create(update_data_timer_cb, 100, NULL);
}

void gui_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    drv_spi_display_init();  // 初始化屏幕 (RTOS版)
    drv_i2c_touchpad_init(); // 初始化触摸 (RTOS版)

    // 硬件初始化已经在 main 或 hal_entry 中完成，这里主要是 LVGL 逻辑
    lv_init();
		lv_port_disp_init();
		lv_port_indev_init();
    build_pages();
    build_wifi_page();
    build_log_page();
    build_motor_page();
    build_imu_page();
    build_nav_page();
    lv_scr_load(ui_main_page);

    extern TaskHandle_t log_thread; // FSP 通常会生成这个句柄
    if (log_thread == NULL)
    {
        printf("Error: Log thread was NOT created!\r\n");
    }

    while (1)
    {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}