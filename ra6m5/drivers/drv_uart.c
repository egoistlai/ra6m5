/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "drv_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/* 引入外部模块的解析头文件 */
#include "ANO_DT_LX.h"
#include "Lidar.h"
#include "Drv_AnoOf.h"

#include "app_queues.h"

QueueHandle_t g_uart7_rx_queue = NULL;
QueueHandle_t g_uart2_rx_queue = NULL;
QueueHandle_t g_uart3_rx_queue = NULL;
QueueHandle_t g_uart6_rx_queue = NULL;
QueueHandle_t g_uart5_rx_queue = NULL;
QueueHandle_t g_ui_log_queue = NULL;

#define UART_QUEUE_SIZE 256

/* 定义数据接收解析宏，统一管理 */
#define U7GetOneByte(data) ANO_DT_LX_Data_Receive_Prepare(data)
#define U3GetOneByte(data) Lidar_Data_Receive(data)
#define U2GetOneByte(data) AnoOF_GetOneByte(data)

/* 外部实例声明 (防止 hal_data.h 未包含) */
extern const uart_instance_t g_uart2;
extern const uart_instance_t g_uart3;

/***********************************************************************************************************************
 * Private Variables
 **********************************************************************************************************************/
/* 为每个串口定义独立的发送信号量，互不干扰 */
static SemaphoreHandle_t g_uart7_tx_sem = NULL;
static SemaphoreHandle_t g_uart2_tx_sem = NULL;
static SemaphoreHandle_t g_uart3_tx_sem = NULL;
static SemaphoreHandle_t g_uart6_tx_sem = NULL;

volatile wifi_status_t g_wifi_status = WIFI_STATUS_DISCONNECTED;
volatile bool g_wifi_connect_req = false;
/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/* =================================================================================
 * UART7: 飞控/数传
 * ================================================================================= */
fsp_err_t drv_uart7_init(void)
{
    fsp_err_t err;
    if (g_uart7_rx_queue == NULL)
    {
        g_uart7_rx_queue = xQueueCreate(UART_QUEUE_SIZE, sizeof(uint8_t));
    }

    if (g_uart7_tx_sem == NULL)
        g_uart7_tx_sem = xSemaphoreCreateBinary();

    err = g_uart7.p_api->open(g_uart7.p_ctrl, g_uart7.p_cfg);
    return err;
}

void DrvUart7SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    fsp_err_t err;
    err = R_SCI_UART_Write(&g_uart7_ctrl, (uint8_t const *)DataToSend, data_num);
    if (FSP_SUCCESS != err)
        return;

    /* 阻塞等待发送完成，与规范保持一致 */
    if (g_uart7_tx_sem != NULL)
    {
        xSemaphoreTake(g_uart7_tx_sem, pdMS_TO_TICKS(100)); // 100ms超时
    }
}

void uart7_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint8_t data = 0;
    switch (p_args->event)
    {
    case UART_EVENT_RX_CHAR:
        data = (uint8_t)p_args->data;
        if (g_uart7_rx_queue != NULL)
        {
            xQueueSendFromISR(g_uart7_rx_queue, &data, &xHigherPriorityTaskWoken);
        }
        break;
    case UART_EVENT_TX_COMPLETE:
        if (g_uart7_tx_sem)
            xSemaphoreGiveFromISR(g_uart7_tx_sem, &xHigherPriorityTaskWoken);
        break;
    /* 错误处理：释放信号量防止死锁 */
    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
        if (g_uart7_tx_sem)
            xSemaphoreGiveFromISR(g_uart7_tx_sem, &xHigherPriorityTaskWoken);
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void uart5_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_args->event)
    {
    case UART_EVENT_RX_CHAR: // 接收到单字节数据
        if (g_uart5_rx_queue != NULL)
        {
            uint8_t rx_byte = (uint8_t)p_args->data;
            // 将接收到的字节压入队列
            xQueueSendFromISR(g_uart5_rx_queue, &rx_byte, &xHigherPriorityTaskWoken);
        }
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void drv_uart5_init(void)
{
    // 1. 创建长度为 256 的单字节队列 (长度可根据内存充裕程度调整)
    g_uart5_rx_queue = xQueueCreate(256, sizeof(uint8_t));

    // 2. 打开底层 UART5
    R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
}

/* =================================================================================
 * UART2: 光流模块 (Optical Flow)
 * ================================================================================= */
fsp_err_t drv_uart2_init(void)
{
    fsp_err_t err;

    if (g_uart2_rx_queue == NULL)
    {
        g_uart2_rx_queue = xQueueCreate(UART_QUEUE_SIZE, sizeof(uint8_t));
    }
    /* 1. 创建信号量 */
    if (g_uart2_tx_sem == NULL)
    {
        g_uart2_tx_sem = xSemaphoreCreateBinary();
        if (g_uart2_tx_sem == NULL)
            __BKPT(0); // 内存不足
    }

    /* 2. 打开串口 */
    err = g_uart2.p_api->open(g_uart2.p_ctrl, g_uart2.p_cfg);
    return err;
}

void DrvUart2SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    fsp_err_t err;

    /* 调用 FSP 写入 */
    err = g_uart2.p_api->write(g_uart2.p_ctrl, (uint8_t const *)DataToSend, data_num);
    if (FSP_SUCCESS != err)
        return;

    /* 阻塞等待发送完成 (保持规范一致) */
    if (g_uart2_tx_sem != NULL)
    {
        xSemaphoreTake(g_uart2_tx_sem, pdMS_TO_TICKS(100));
    }
}

void uart2_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint8_t data = 0;
    switch (p_args->event)
    {
    case UART_EVENT_RX_CHAR:
        data = (uint8_t)p_args->data;
        // 【修改点】发送到队列
        if (g_uart2_rx_queue != NULL)
        {
            xQueueSendFromISR(g_uart2_rx_queue, &data, &xHigherPriorityTaskWoken);
        }
        break;

    case UART_EVENT_TX_COMPLETE:
        /* 发送完成，释放信号量 */
        if (g_uart2_tx_sem)
            xSemaphoreGiveFromISR(g_uart2_tx_sem, &xHigherPriorityTaskWoken);
        break;

    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
        /* 错误处理：防止发送任务卡死 */
        if (g_uart2_tx_sem)
            xSemaphoreGiveFromISR(g_uart2_tx_sem, &xHigherPriorityTaskWoken);
        break;

    default:
        break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* =================================================================================
 * UART3: 激光雷达 (Lidar)
 * ================================================================================= */
fsp_err_t drv_uart3_init(void)
{
    fsp_err_t err;
    if (g_uart3_rx_queue == NULL)
    {
        g_uart3_rx_queue = xQueueCreate(UART_QUEUE_SIZE, sizeof(uint8_t));
    }

    /* 1. 创建信号量 */
    if (g_uart3_tx_sem == NULL)
    {
        g_uart3_tx_sem = xSemaphoreCreateBinary();
        if (g_uart3_tx_sem == NULL)
            __BKPT(0);
    }

    /* 2. 打开串口 */
    err = g_uart3.p_api->open(g_uart3.p_ctrl, g_uart3.p_cfg);
    return err;
}

void DrvUart3SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    fsp_err_t err;

    /* 调用 FSP 写入 */
    err = g_uart3.p_api->write(g_uart3.p_ctrl, (uint8_t const *)DataToSend, data_num);
    if (FSP_SUCCESS != err)
        return;

    /* 阻塞等待发送完成 (保持规范一致) */
    if (g_uart3_tx_sem != NULL)
    {
        xSemaphoreTake(g_uart3_tx_sem, pdMS_TO_TICKS(100));
    }
}

void uart3_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint8_t data = 0;
    switch (p_args->event)
    {
    case UART_EVENT_RX_CHAR:
        data = (uint8_t)p_args->data;
        // 【修改点】发送到队列
        if (g_uart3_rx_queue != NULL)
        {
            xQueueSendFromISR(g_uart3_rx_queue, &data, &xHigherPriorityTaskWoken);
        }
        break;

    case UART_EVENT_TX_COMPLETE:
        /* 发送完成，释放信号量 */
        if (g_uart3_tx_sem)
            xSemaphoreGiveFromISR(g_uart3_tx_sem, &xHigherPriorityTaskWoken);
        break;

    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
        if (g_uart3_tx_sem)
            xSemaphoreGiveFromISR(g_uart3_tx_sem, &xHigherPriorityTaskWoken);
        break;

    default:
        break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ================= UART6 W800 驱动实现 ================= */
fsp_err_t drv_uart6_init(void)
{
    fsp_err_t err;

    /* 1. 初始化接收队列 (W800数据量大，建议 512 或 1024) */
    if (g_uart6_rx_queue == NULL)
    {
        g_uart6_rx_queue = xQueueCreate(1024, sizeof(uint8_t));
    }

    /* 2. 初始化 UI 日志队列 */
    if (g_ui_log_queue == NULL)
    {
        g_ui_log_queue = xQueueCreate(1024, sizeof(char));
    }

    /* 3. 初始化发送信号量 */
    if (g_uart6_tx_sem == NULL)
    {
        g_uart6_tx_sem = xSemaphoreCreateBinary();
    }

    /* 4. 打开串口 */
    err = g_uart6.p_api->open(g_uart6.p_ctrl, g_uart6.p_cfg);
    return err;
}

void DrvUart6SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    fsp_err_t err;
    err = g_uart6.p_api->write(g_uart6.p_ctrl, DataToSend, data_num);

    /* 阻塞等待发送完成 */
    if (FSP_SUCCESS == err && g_uart6_tx_sem != NULL)
    {
        xSemaphoreTake(g_uart6_tx_sem, pdMS_TO_TICKS(200));
    }
}

void uart6_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t data;

    switch (p_args->event)
    {
    case UART_EVENT_RX_CHAR:
        data = (uint8_t)p_args->data;
        if (g_uart6_rx_queue != NULL)
        {
            xQueueSendFromISR(g_uart6_rx_queue, &data, &xHigherPriorityTaskWoken);
        }
        break;
    case UART_EVENT_TX_COMPLETE:
        if (g_uart6_tx_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_uart6_tx_sem, &xHigherPriorityTaskWoken);
        }
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 重定向 printf (使用 UART7) */
int fputc(int ch, FILE *f)
{
    (void)f;
    if (xPortIsInsideInterrupt())
        return ch;
    DrvUart7SendBuf((unsigned char *)&ch, 1);
    return ch;
}
