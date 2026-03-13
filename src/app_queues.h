#ifndef APP_QUEUES_H
#define APP_QUEUES_H

#include "FreeRTOS.h"
#include "queue.h"

// 声明外部可见的队列句柄
extern QueueHandle_t g_uart7_rx_queue; // 用于飞控数据
extern QueueHandle_t g_uart2_rx_queue; // 用于光流数据
extern QueueHandle_t g_uart3_rx_queue; // 用于雷达数据
extern QueueHandle_t g_uart5_rx_queue; // 用于我的imu

// 【新增】WiFi 相关队列与标志
extern QueueHandle_t g_uart6_rx_queue; // W800 原始数据接收队列
extern QueueHandle_t g_ui_log_queue;   // 用于显示到屏幕的日志队列 (存放 char)

// WiFi 状态机枚举
typedef enum
{
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_ERROR
} wifi_status_t;

// 全局控制变量
extern volatile wifi_status_t g_wifi_status; // 当前状态
extern volatile bool g_wifi_connect_req;     // GUI 发出的连接请求

#define LOG_QUEUE_LEN 10    // 队列深度
#define LOG_STR_MAX_SIZE 128 // 单条日志最大长度

typedef struct
{
    char text[LOG_STR_MAX_SIZE];
} log_data_t;

extern QueueHandle_t g_log_save_queue; // 处理日志写盘的队列

#endif