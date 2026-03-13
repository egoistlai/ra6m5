#include "app_queues.h"
#include "drv_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "My_Task.h"

/* 引用解析函数所在的头文件 */
#include "ANO_DT_LX.h" // 解析飞控数据
#include "Lidar.h"     // 解析雷达数据
#include "Drv_AnoOf.h" // 解析光流数据
#include "Drv_BSP.h"
#include "drv_imu.h"

#include <math.h>

#define IMU_CHANGE_THRESHOLD 0.1f
#define LIDAR_CHANGE_THRESHOLD 0.02f

void data_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    All_Init();

    vTaskDelay(pdMS_TO_TICKS(50));

    while (g_uart7_rx_queue == NULL || g_uart2_rx_queue == NULL || g_uart3_rx_queue == NULL || g_uart5_rx_queue == NULL)
    {
        // 尝试再次初始化或打印错误
        // printf("Error: Queue not ready!\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        // 如果你有多个串口，这里也要检查 g_uart2_rx_queue 等
    }

    float last_roll = 0.0f, last_pitch = 0.0f, last_yaw = 0.0f;
    float last_lid_x = 0.0f, last_lid_y = 0.0f, last_lid_z = 0.0f;
    int16_t last_of_dx = 0, last_of_dy = 0;
    uint8_t last_of_qual = 0;
    uint8_t rx_byte;
    TickType_t last_log_tick = xTaskGetTickCount();

    while (1)
    {
        bool has_data = false;

        // 1. 处理飞控数据 (UART7) -> 存入 fc_att / fc_vel 等
        while (xQueueReceive(g_uart7_rx_queue, &rx_byte, 0) == pdPASS)
        {
            ANO_DT_LX_Data_Receive_Prepare(rx_byte);
            AnoOF_GetOneByte(rx_byte);
            Lidar_Data_Receive(rx_byte);
            has_data = true;
        }

        // 2. 处理光流数据 (UART2) -> 存入 ano_of
        while (xQueueReceive(g_uart2_rx_queue, &rx_byte, 0) == pdPASS)
        {
            AnoOF_GetOneByte(rx_byte);
            has_data = true;
        }

        // 3. 处理雷达数据 (UART3) -> 存入 sensor_buffer
        while (xQueueReceive(g_uart3_rx_queue, &rx_byte, 0) == pdPASS)
        {
            Lidar_Data_Receive(rx_byte);
            has_data = true;
        }

        while (xQueueReceive(g_uart5_rx_queue, &rx_byte, 0) == pdPASS)
        {
            drv_imu_parse_byte(rx_byte);
            has_data = true;
        }
        
        float roll = fc_att.st_data.rol_x100 / 100.0f;
        float pitch = fc_att.st_data.pit_x100 / 100.0f;
        float yaw = fc_att.st_data.yaw_x100 / 100.0f;
        int16_t of_dx = ano_of.of1_dx;
        int16_t of_dy = ano_of.of1_dy;
        uint8_t of_qual = ano_of.of_quality;
        float lid_x = sensor_buffer.lidar.x_lidar;
        float lid_y = sensor_buffer.lidar.y_lidar;
        float lid_z = sensor_buffer.lidar.z_lidar;

        bool is_valid_data = !(fabs(roll) < 0.001f && fabs(pitch) < 0.001f && fabs(yaw) < 0.001f &&
                               fabs(lid_z) < 0.001f && of_qual == 0);

        // 3. 检查数据是否发生了变化
        bool is_changed = (fabs(roll - last_roll) > IMU_CHANGE_THRESHOLD ||
                           fabs(pitch - last_pitch) > IMU_CHANGE_THRESHOLD ||
                           fabs(yaw - last_yaw) > IMU_CHANGE_THRESHOLD ||
                           fabs(lid_x - last_lid_x) > LIDAR_CHANGE_THRESHOLD ||
                           fabs(lid_y - last_lid_y) > LIDAR_CHANGE_THRESHOLD ||
                           fabs(lid_z - last_lid_z) > LIDAR_CHANGE_THRESHOLD ||
                           of_dx != last_of_dx ||
                           of_dy != last_of_dy ||
                           of_qual != last_of_qual);

        // 4. 频率限制：即使数据一直在变，也限制最高 100ms (10Hz) 记录一次，保护 Flash
        bool is_time_up = ((xTaskGetTickCount() - last_log_tick) >= pdMS_TO_TICKS(100));

        if (is_valid_data && is_changed && is_time_up)
        {
            last_log_tick = xTaskGetTickCount();

            // 更新历史值
            last_roll = roll;
            last_pitch = pitch;
            last_yaw = yaw;
            last_lid_x = lid_x;
            last_lid_y = lid_y;
            last_lid_z = lid_z;
            last_of_dx = of_dx;
            last_of_dy = of_dy;
            last_of_qual = of_qual;

            log_data_t log_env;

            // 格式化出超长详细信息
            snprintf(log_env.text, sizeof(log_env.text),
                     "[%u] IMU:%.1f,%.1f,%.1f OF:%d,%d,%u LID:%.1f,%.1f,%.1f\n",
                     last_log_tick,
                     roll, pitch, yaw,
                     of_dx, of_dy, of_qual,
                     lid_x, lid_y, lid_z);

            // 发送到日志保存队列
            if (g_log_save_queue != NULL)
            {
                xQueueSend(g_log_save_queue, &log_env, 0); // 队列满了就丢弃，绝不阻塞外设接收
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}