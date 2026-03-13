#include "log_thread.h"
#include "app_queues.h"
#include "drv_uart.h"
#include "drv_littlefs_log.h" // 这里假设你已经按前面的建议改用 LittleFS 了

QueueHandle_t g_log_save_queue = NULL;

void log_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
	
	vTaskDelay(pdMS_TO_TICKS(500));

    // 初始化文件系统
    printf("[Log Thread] Init LittleFS...\r\n");
    int fs_err = drv_log_fs_init(); // 假设返回 int 错误码
    if (fs_err != 0)                // 或者是 FSP_SUCCESS
    {
        printf("[Log Thread] W25Q / LittleFS Mount FAILED! Err: %d\r\n", fs_err);
        // 如果这里失败了，说明确实是你猜的 W25Q 没初始化好
    }
    else
    {
        printf("[Log Thread] W25Q / LittleFS Mounted OK!\r\n");
    }

    /* ========================================================= */
    /* 【修改 2】在线程一开始，初始化这个队列                    */
    /* LOG_QUEUE_LEN 和 log_data_t 都在 app_queues.h 中有定义   */
    /* ========================================================= */
    g_log_save_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_data_t));
    

    if (g_log_save_queue == NULL)
    {
        // 队列创建失败，这里可以加个 printf 报错或者直接死机挂起
        printf("Error: Failed to create g_log_save_queue!\r\n");
        while (1)
        {
            vTaskDelay(100);
        }
    }

    
    log_data_t log_item;

    while (1)
    {
        // 阻塞等待队列消息
        if (xQueueReceive(g_log_save_queue, &log_item, portMAX_DELAY) == pdPASS)
        {
            drv_log_save_line(log_item.text);
        }
        vTaskDelay(100);
    }
}