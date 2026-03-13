#include "uart_test_thread.h"
#include "drv_uart.h"
#include "drv_wifi.h"
#include "app_queues.h"
#include <stdio.h>

/* 目标服务器配置 */
#define TARGET_SSID "Egoist-py"
#define TARGET_PWD "oppophone"
#define TARGET_IP "10.98.64.104"
//#define TARGET_IP "10.88.29.74"
#define TARGET_PORT 8888

/* === 引入导航控制相关的全局变量 === */
extern volatile float g_nav_target_x;
extern volatile float g_nav_target_y;
extern volatile bool g_nav_start_flag;

/* === 移植自 dev_wifi_bt.c 的状态机变量 === */
static uint8_t gDetectBuff[256] = {0}; // 状态机解析缓冲区
static uint32_t gDetectIdx = 0;        // 缓冲区当前索引
static uint32_t gParseStatus = 0;      // 状态机当前状态
static uint32_t gSocketPort = 0;       // 解析出的 Socket ID
static uint32_t gDataLength = 0;       // 解析出的数据长度
static uint8_t gBackCnt = 0;           // 换行符计数器

/* 接收数据的最终缓冲区 */
static uint8_t gPayloadBuf[1025];
static uint32_t gPayloadCnt = 0;

void WiFi_Parse_Byte(uint8_t data)
{
    // 将数据存入临时解析缓冲
    gDetectBuff[gDetectIdx] = data;

    // 字符串结束符保护，方便 strstr 使用
    if (gDetectIdx < 255)
        gDetectBuff[gDetectIdx + 1] = '\0';

    switch (gParseStatus)
    {
    case 0: // Status 0: 等待头部 "+SKTRPT="
    {
        // 使用 strstr 在缓冲区中查找头部
        if (strstr((char *)gDetectBuff, "+SKTRPT="))
        {
            gParseStatus = 1;
            gDetectIdx = 0; // 重置索引，准备读取下一个字段
            memset(gDetectBuff, 0, sizeof(gDetectBuff));
        }
        else
        {
            // 简单的防溢出处理，如果太长没匹配到，就回滚
            gDetectIdx++;
            if (gDetectIdx >= 250)
                gDetectIdx = 0;
        }
        break;
    }

    case 1: // Status 1: 读取 Socket ID (直到遇到逗号)
    {
        if (data == ',')
        {
            gSocketPort = 0;
            // 解析数字 (buffer 中目前存的是 socket id)
            for (uint32_t m = 0; m < gDetectIdx; m++)
            {
                if (gDetectBuff[m] >= '0' && gDetectBuff[m] <= '9')
                    gSocketPort = gSocketPort * 10 + (gDetectBuff[m] - '0');
            }
            gParseStatus = 2;
            gDetectIdx = 0; // 重置
        }
        else
        {
            gDetectIdx++;
        }
        break;
    }

    case 2: // Status 2: 读取数据长度 (直到遇到逗号)
    {
        if (data == ',')
        {
            gDataLength = 0;
            // 解析长度
            for (uint32_t m = 0; m < gDetectIdx; m++)
            {
                if (gDetectBuff[m] >= '0' && gDetectBuff[m] <= '9')
                    gDataLength = gDataLength * 10 + (gDetectBuff[m] - '0');
            }

            // 长度保护
            if (gDataLength > 1023)
                gDataLength = 1023;

            gParseStatus = 3;
            gBackCnt = 0;   // 清零换行计数
            gDetectIdx = 0; // 重置
        }
        else
        {
            gDetectIdx++;
        }
        break;
    }

    case 3: // Status 3: 寻找两个换行符 (Find double \r\n)
    {
        /* * 原代码逻辑：if(pbuf[i]=='\n') backcnt++; if(backcnt == 2) ...
         * 这意味着它忽略中间的 IP 和 Port，只数换行符。
         */
        if (data == '\n')
        {
            gBackCnt++;
        }

        if (gBackCnt >= 2) // 这里的 >= 2 是根据 dev_wifi_bt.c 的逻辑
        {
            gParseStatus = 4;
            gPayloadCnt = 0; // 准备接收 payload
            // gDetectIdx 在这里不再用于 buffer，而是仅仅占位
        }
        break;
    }

    case 4: // Status 4: 接收真正的 Net Data
    {
        if (gPayloadCnt < gDataLength)
        {
            gPayloadBuf[gPayloadCnt++] = data;

            // 实时发给 GUI 显示
            xQueueSend(g_ui_log_queue, &data, 0);
        }

        // 接收完毕？
        if (gPayloadCnt >= gDataLength)
        {
            /* === 此时已收到完整的一包数据 === */
            // 为字符串解析添加结尾符 '\0'
            gPayloadBuf[gPayloadCnt] = '\0';

            float parsed_x = 0.0f;
            float parsed_y = 0.0f;

            // 【核心】：尝试解析我们的 "NAV:x,y" 协议
            if (sscanf((char *)gPayloadBuf, "NAV:%f,%f", &parsed_x, &parsed_y) == 2)
            {
                // 解析成功！赋值给小车全局目标
                g_nav_target_x = parsed_x;
                g_nav_target_y = parsed_y;

                // 触发电机线程中的导航状态机
                g_nav_start_flag = true;

                // 回传 ACK 确认指令
                char ack_buf[64];
                int ack_len = snprintf(ack_buf, sizeof(ack_buf), "ACK_NAV:%.1f,%.1f\r\n", parsed_x, parsed_y);
                WiFi_Send((uint8_t *)ack_buf, ack_len);
            }
            else
            {
                // 如果格式不对，或者只是普通的聊天测试数据，直接 Echo 原样回传
                WiFi_Send(gPayloadBuf, gDataLength);
            }

            // 2. 状态机复位
            gParseStatus = 0;
            gDetectIdx = 0;
            gDataLength = 0;
            gSocketPort = 0;
            memset(gDetectBuff, 0, sizeof(gDetectBuff));

            // 调试：打印一下收到的长度
            // printf("Pkg Len: %d\r\n", gPayloadCnt);
        }
        break;
    }

    default:
        gParseStatus = 0;
        break;
    }
}

void uart_test_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 1. 初始化 */
    drv_uart6_init(); // 初始化 WiFi 串口

    /* === 1. 完整的初始化流程 (复刻参考代码) === */
    printf("=== WiFi Init Start ===\r\n");

    // (1) 复位
    DrvUart6SendBuf((uint8_t *)"AT+Z\r\n", 6);
    vTaskDelay(pdMS_TO_TICKS(2000)); // 参考代码延时了 2秒
    xQueueReset(g_uart6_rx_queue);

    // (2) 设置为 STA 模式 (关键!)
    if (WiFi_Set_STA_Mode() != 0)
        printf("Set STA Failed\r\n");

    // (3) 开启 DHCP
    if (WiFi_Enable_DHCP() != 0)
        printf("Set DHCP Failed\r\n");

    printf("=== WiFi Init Done ===\r\n");

    uint8_t rx_data;

    while (1)
    {
        /* 状态机 */
        switch (g_wifi_status)
        {
        case WIFI_STATUS_DISCONNECTED:
        case WIFI_STATUS_ERROR:
            /* 等待 GUI 发起连接请求 */
            if (g_wifi_connect_req)
            {
                g_wifi_connect_req = false;
                g_wifi_status = WIFI_STATUS_CONNECTING;

                // 1. 连接 AP
                if (WiFi_Connect_AP(TARGET_SSID, TARGET_PWD) != 0)
                {
                    g_wifi_status = WIFI_STATUS_ERROR;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(3000));
                WiFi_Get_IP();
                vTaskDelay(pdMS_TO_TICKS(3000));

                // 2. 连接 TCP
                if (WiFi_Connect_TCP(TARGET_IP, TARGET_PORT) != 0)
                {
                    g_wifi_status = WIFI_STATUS_ERROR;
                    break;
                }

                printf("Enable Receive Mode...\r\n");
                if (WiFi_Set_RPTM() != 0)
                {
                    printf("Set RPTM Failed!\r\n");
                    // 即使失败也继续尝试，不要 break
                }

                // 3. 连接成功
                g_wifi_status = WIFI_STATUS_CONNECTED;
                xQueueReset(g_uart6_rx_queue); // 清空之前的杂波
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            break;

        case WIFI_STATUS_CONNECTING:
            // 正在连接中，不做处理，等待上面的逻辑完成
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case WIFI_STATUS_CONNECTED:
            if (xQueueReceive(g_uart6_rx_queue, &rx_data, pdMS_TO_TICKS(10)) == pdPASS)
            {
                // 将数据喂给移植好的状态机
                WiFi_Parse_Byte(rx_data);
            }
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}