#include "drv_wifi.h"
#include "drv_uart.h"
#include "app_queues.h"
#include "ring_buffer.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>


/* ================= 配置区域 ================= */
#define WIFI_UART_CTRL &g_uart6_ctrl
#define WIFI_UART_CFG &g_uart6_cfg
#define WIFI_UART_API g_uart6.p_api

/* 缓冲区大小 */
#define WIFI_RX_BUF_SIZE 2048

/* 错误码 */
#define WIFI_SUCCESS 0
#define WIFI_ERROR -1
#define WIFI_TIMEOUT -2

/* ================= 变量定义 ================= */
static SemaphoreHandle_t g_wifi_tx_sem = NULL;
static struct RingBuffer *g_wifi_rb = NULL; // 环形缓冲区对象指针

/* ================= 内部辅助函数声明 ================= */
static int WiFi_Uart_Write(const void *data, uint32_t len);
static int WiFi_Uart_ReadByte(uint8_t *data, uint32_t timeout_ms);
static int WiFiBtDevCmdRet(const char *expect_ret, unsigned int timeout_ms);

/* ================= 核心驱动实现 ================= */

/* 初始化 */
fsp_err_t drv_wifi_init(void)
{
    fsp_err_t err;

    /* 1. 初始化环形缓冲区 */
    /* 注意：RingBufferNew 使用 malloc，请确保 FSP 配置中 BSP Heap Size 足够大 (例如 0x1000) */
    if (g_wifi_rb == NULL)
    {
        g_wifi_rb = RingBufferNew(WIFI_RX_BUF_SIZE);
        if (g_wifi_rb == NULL)
        {
            printf("WiFi RingBuffer Init Failed (Heap too small?)\r\n");
            return FSP_ERR_OUT_OF_MEMORY;
        }
    }
    else
    {
        g_wifi_rb->Clear(g_wifi_rb);
    }

    /* 2. 创建发送信号量 */
    if (g_wifi_tx_sem == NULL)
    {
        g_wifi_tx_sem = xSemaphoreCreateBinary();
    }

    /* 3. 打开串口 */
    err = WIFI_UART_API->open(WIFI_UART_CTRL, WIFI_UART_CFG);
    if (FSP_SUCCESS != err)
    {
        printf("WiFi UART Open Failed: %d\r\n", err);
        return err;
    }

    return FSP_SUCCESS;
}

/* ================= 底层数据操作 ================= */

/* 阻塞发送函数 */
static int WiFi_Uart_Write(const void *data, uint32_t len)
{
    fsp_err_t err;

    err = WIFI_UART_API->write(WIFI_UART_CTRL, data, len);
    if (FSP_SUCCESS != err)
        return WIFI_ERROR;

    /* 等待发送完成 */
    if (xSemaphoreTake(g_wifi_tx_sem, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        return WIFI_TIMEOUT;
    }
    return WIFI_SUCCESS;
}

/* 带超时的单字节读取 - 包含临界区保护 */
static int WiFi_Uart_ReadByte(uint8_t *data, uint32_t timeout_ms)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);
    int ret;

    do
    {
        /* [关键修复 2]：进入临界区
         * 防止 Read 过程中发生 UART 中断调用 Write，导致 RingBuffer 内部指针错乱
         */
        taskENTER_CRITICAL();
        ret = g_wifi_rb->Read(g_wifi_rb, data, 1);
        taskEXIT_CRITICAL();

        if (ret == 1) // Read 返回读取的字节数，1 表示成功
        {
            return WIFI_SUCCESS;
        }

        /* 没数据，延时 1ms 让出 CPU */
        vTaskDelay(1);

    } while ((xTaskGetTickCount() - start_tick) < wait_ticks);

    return WIFI_TIMEOUT;
}

/* 等待特定 AT 响应 (复用 RingBuffer) */
static int WiFiBtDevCmdRet(const char *expect_ret, unsigned int timeout_ms)
{
    unsigned char c;
    char buf[128] = {0};
    unsigned int idx = 0;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < wait_ticks)
    {
        if (WiFi_Uart_ReadByte(&c, 10) == WIFI_SUCCESS)
        {
            if (idx < (sizeof(buf) - 1))
            {
                buf[idx++] = (char)c;
                buf[idx] = '\0';
                // printf("%c", c); // Debug
            }
            else
            {
                /* 滑动窗口 */
                memmove(buf, &buf[1], sizeof(buf) - 2);
                buf[sizeof(buf) - 2] = (char)c;
                buf[sizeof(buf) - 1] = '\0';
            }

            if (strstr(buf, expect_ret))
                return WIFI_SUCCESS;
            if (strstr(buf, "+ERR") || strstr(buf, "ERROR"))
                return WIFI_ERROR;
        }
    }
    return WIFI_TIMEOUT;
}

/* ================= 业务逻辑 (保持不变，已适配底层) ================= */

int WiFiBtDevSetWorkType(WorkType type)
{
    char str[64];
    sprintf(str, "AT+WPRT=%d\r\n", type);
    if (WiFi_Uart_Write(str, strlen(str)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    return WiFiBtDevCmdRet("+OK", 200);
}

int WiFiBtDevEnableDHCP(void)
{
    char *str = "AT+NIP=0\r\n";
    if (WiFi_Uart_Write(str, strlen(str)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    return WiFiBtDevCmdRet("+OK", 200);
}

int WiFiBtDevDisableDHCP(const char *ip, const char *netmask, const char *gateway)
{
    char str[128];
    sprintf(str, "AT+NIP=1,%s,%s,%s\r\n", ip, netmask, gateway);
    if (WiFi_Uart_Write(str, strlen(str)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    return WiFiBtDevCmdRet("+OK", 200);
}

int WiFiBtDevNetScan(void)
{
    const char *str = "AT+WSCAN\r\n";
    if (WiFi_Uart_Write(str, strlen(str)) != WIFI_SUCCESS)
        return WIFI_ERROR;

    unsigned char c;
    TickType_t start_tick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(5000))
    {
        if (WiFi_Uart_ReadByte(&c, 10) == WIFI_SUCCESS)
        {
            printf("%c", c);
        }
    }
    return WIFI_SUCCESS;
}

int WiFiBtDevConnectWiFi(const char *name, const char *password)
{
    char cmd[128];

    sprintf(cmd, "AT+SSID=%s\r\n", name);
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    if (WiFiBtDevCmdRet("+OK", 200) != WIFI_SUCCESS)
        return WIFI_ERROR;

    sprintf(cmd, "AT+KEY=1,0,%s\r\n", password);
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    if (WiFiBtDevCmdRet("+OK", 200) != WIFI_SUCCESS)
        return WIFI_ERROR;

    sprintf(cmd, "AT+WJOIN\r\n");
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;

    vTaskDelay(pdMS_TO_TICKS(100));

    if (WiFiBtDevCmdRet("+OK", 8000) != WIFI_SUCCESS)
        return WIFI_ERROR;

    return WIFI_SUCCESS;
}

int WiFiBtDevDisconnectWiFi(void)
{
    char *cmd = "AT+WLEAV\r\n";
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    return WiFiBtDevCmdRet("+OK", 500);
}

int WiFiBtDevGetLocalIP(void)
{
    char *cmd = "AT+LKSTT\r\n";
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;

    unsigned char c;
    TickType_t start_tick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(1000))
    {
        if (WiFi_Uart_ReadByte(&c, 10) == WIFI_SUCCESS)
        {
            printf("%c", c);
        }
    }
    return WIFI_SUCCESS;
}

int WiFiBtDevConnect(ConnectInfo *info)
{
    char cmd[128];
    sprintf(cmd, "AT+SKCT=%d,%d,%s,%d,%d\r\n",
            info->Protocl,
            info->Role,
            info->IP,
            info->RemotePort,
            info->LocalPort);
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;

    if (WiFiBtDevCmdRet("+OK=", 3000) != WIFI_SUCCESS)
        return WIFI_ERROR;

    /* 解析 Socket ID */
    unsigned char c;
    char buf[32] = {0};
    int idx = 0;

    while (idx < 31)
    {
        if (WiFi_Uart_ReadByte(&c, 100) == WIFI_SUCCESS)
        {
            if (c == '\r' || c == '\n')
                break;
            buf[idx++] = c;
        }
        else
            break;
    }
    buf[idx] = '\0';

    info->SocketPort = atoi(buf);
    printf("WiFi Connected. Target IP:%s - SocketID:%d\r\n", info->IP, info->SocketPort);

    return WIFI_SUCCESS;
}

int WiFiBtDevDisconnect(ConnectInfo info)
{
    char cmd[32];
    sprintf(cmd, "AT+SKCLS=%d\r\n", info.SocketPort);
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;
    return WiFiBtDevCmdRet("+OK", 500);
}

int WiFiBtDevWrite(ConnectInfo info, unsigned char *buf, unsigned int length)
{
    if (NULL == buf || 0 == length)
        return -1;

    char cmd[64];
    /* 1. 发送发送请求：AT+SKSND=<socket>,<len> */
    sprintf(cmd, "AT+SKSND=%d,%d\r\n", info.SocketPort, length);
    if (WiFi_Uart_Write(cmd, strlen(cmd)) != WIFI_SUCCESS)
        return WIFI_ERROR;

    /* 2. 解析模块响应: +OK=xxxxxxxx\r\n\r\n */
    unsigned char c;
    char tmp[64] = {0};
    int tmp_idx = 0;
    int step = 0;
    unsigned int nDataLength = 0;

    TickType_t start_tick = xTaskGetTickCount();

    while (step != 3 && (xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(2000))
    {
        if (WiFi_Uart_ReadByte(&c, 10) == WIFI_SUCCESS)
        {
            if (tmp_idx < 63)
            {
                tmp[tmp_idx++] = c;
                tmp[tmp_idx] = '\0';
            }
            else
            {
                memmove(tmp, &tmp[1], 62);
                tmp[62] = c;
            }

            switch (step)
            {
            case 0:
                if (strstr(tmp, "+OK="))
                    step = 1;
                break;
            case 1:
                if (c >= '0' && c <= '9')
                {
                    nDataLength = nDataLength * 10 + (c - '0');
                }
                if (c == '\r')
                    step = 2;
                break;
            case 2:
                if (strstr(tmp, "\r\n\r\n"))
                    step = 3;
                break;
            default:
                break;
            }
        }
    }

    if (step == 3)
    {
        if (nDataLength < length)
            length = nDataLength;

        if (WiFi_Uart_Write(buf, length) != WIFI_SUCCESS)
            return WIFI_ERROR;
        return WIFI_SUCCESS;
    }

    return WIFI_TIMEOUT;
}

static int WiFi_Wait_Response(const char *expect, uint32_t timeout_ms)
{
    uint8_t ch;
    char buf[64] = {0};
    uint8_t idx = 0;
    TickType_t start_tick = xTaskGetTickCount();

    printf("WAITING FOR: %s\r\n", expect); // 调试打印

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms))
    {
        if (xQueueReceive(g_uart6_rx_queue, &ch, pdMS_TO_TICKS(10)) == pdPASS)
        {
            // 将收到的每一个字符打印出来，看看到底回了什么
            putchar((char)ch);

            /* 滑动窗口逻辑 */
            if (idx < 63)
            {
                buf[idx++] = (char)ch;
                buf[idx] = '\0';
            }
            else
            {
                memcpy(buf, &buf[1], 62);
                buf[62] = (char)ch;
            }

            if (strstr(buf, expect) != NULL)
                return 0;

            // 如果收到 ERROR，立即返回错误
            if (strstr(buf, "ERROR") != NULL)
            {
                printf("\r\n[WIFI Error] Received ERROR response!\r\n");
                return -1;
            }
        }
    }
    printf("\r\n[WIFI Timeout] Expected response not found.\r\n");
    return -1;
}

void WiFi_Init(void)
{
    // 复位指令 (可选)
    // DrvUart6SendBuf((uint8_t*)"AT+Z\r\n", 6);
    // vTaskDelay(pdMS_TO_TICKS(1000));
}

int WiFi_Connect_AP(const char *ssid, const char *pwd)
{
    char cmd[128];

    // 清空队列
    xQueueReset(g_uart6_rx_queue);

    // 1. 设置 SSID
    sprintf(cmd, "AT+SSID=%s\r\n", ssid);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));
    if (WiFi_Wait_Response("+OK", 500) != 0)
        return -1;

    // 2. 设置密码
    sprintf(cmd, "AT+KEY=1,0,%s\r\n", pwd);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));
    if (WiFi_Wait_Response("+OK", 500) != 0)
        return -1;

    // 3. 连接
    sprintf(cmd, "AT+WJOIN\r\n");
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));

    // 连接耗时较长
    return WiFi_Wait_Response("+OK", 8000);
}

int WiFi_Connect_TCP(const char *ip, uint16_t port)
{
    char cmd[128];
    // Protocl=0(TCP), Role=0(Client), IP, RemotePort, LocalPort=0(随机)
    //sprintf(cmd, "AT+SKCT=0,0,%s,%d,0\r\n", ip, port);
    sprintf(cmd, "AT+SKCT=0,0,\"%s\",%d,9090\r\n", ip, port);

    printf("CMD: %s", cmd); // 打印发送的指令
    
    DrvUart6SendBuf((uint8_t*)cmd, strlen(cmd));
    
    // 期待返回 +OK=1 (或其他数字)
    return WiFi_Wait_Response("+OK=", 5000);
}

int WiFi_Send(uint8_t *data, uint32_t len)
{
    char cmd[32];
    // 假设 Socket ID 为 1 (注意：实际项目中应解析 +OK=x 获取 ID)
    // 这里为了简单，默认用 1，如果失败可能是 ID 不对
    sprintf(cmd, "AT+SKSND=1,%d\r\n", len);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));

    // 稍微延时等待模块准备好接收数据 (严格来说应该等 '>')
    vTaskDelay(pdMS_TO_TICKS(20));

    DrvUart6SendBuf(data, len);
    return 0;
}

void WiFi_Get_IP(void)
{
    char cmd[] = "AT+LKSTT\r\n";

    /* 1. 清空接收队列，防止旧数据干扰 */
    xQueueReset(g_uart6_rx_queue);

    /* 2. 发送查询指令 */
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));

    printf("Fetching IP Info...\r\n");

    /* 3. 循环读取并打印响应，设置 1~2 秒超时 */
    /* AT+LKSTT 的返回通常包含多行信息，包括 IP、掩码、网关等 */
    uint8_t ch;
    TickType_t start_tick = xTaskGetTickCount();

    // 等待 1500ms 接收数据
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(1500))
    {
        // 从 W800 接收队列读取一个字节
        if (xQueueReceive(g_uart6_rx_queue, &ch, pdMS_TO_TICKS(10)) == pdPASS)
        {
            // 直接打印到调试串口 (printf)
            putchar((char)ch);
        }
    }
    printf("\r\nIP Check Done.\r\n");
}

int WiFi_Set_STA_Mode(void)
{
    // 设置为 STA 模式 (0)
    char *cmd = "AT+WPRT=0\r\n";
    printf("CMD: %s", cmd);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));
    return WiFi_Wait_Response("+OK", 1000);
}

int WiFi_Enable_DHCP(void)
{
    // 开启 DHCP (0)
    char *cmd = "AT+NIP=0\r\n";
    printf("CMD: %s", cmd);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));
    return WiFi_Wait_Response("+OK", 1000);
}

int WiFi_Set_RPTM(void)
{
    // 开启 socket 接收自动上报模式 (1: 开启, 0: 关闭)
    char *cmd = "AT+SKRPTM=1\r\n";
    printf("CMD: %s", cmd);
    DrvUart6SendBuf((uint8_t *)cmd, strlen(cmd));
    return WiFi_Wait_Response("+OK", 1000);
}