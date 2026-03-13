#include "drv_littlefs_log.h"
#include "drv_w25q.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h" // 引入信号量

lfs_t g_lfs;
lfs_file_t g_log_file;
static bool g_fs_ready = false;

// 定义一把互斥锁，保护 LittleFS
static SemaphoreHandle_t g_lfs_mutex = NULL;

static uint8_t lfs_read_buf[256];
static uint8_t lfs_prog_buf[256];
static uint8_t lfs_lookahead_buf[16];

extern int W25QDrvRead_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
extern int W25QDrvProg_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
extern int W25QDrvErase_LittleFS(const struct lfs_config *c, lfs_block_t block);
extern int W25QDrvSync_LittleFS(const struct lfs_config *c);

const struct lfs_config cfg = {
    .read = W25QDrvRead_LittleFS,
    .prog = W25QDrvProg_LittleFS,
    .erase = W25QDrvErase_LittleFS,
    .sync = W25QDrvSync_LittleFS,

    // 【修改点】：匹配 W25Q 的硬件特性
    .read_size = 256,    // 读缓冲大小改为 256
    .prog_size = 256,    // 页编程大小改为 256 (绝不能是 1)
    .block_size = 4096,  // 擦除扇区大小 4KB，这个是对的
    .block_count = 2048, // 8MB / 4KB = 2048，这个也是对的
    .cache_size = 256,   // 缓存大小与页编程大小一致
    .lookahead_size = 16,
    .block_cycles = 500,
};

fsp_err_t drv_log_fs_init(void)
{
    W25QDrvInit();

    // 1. 创建互斥锁
    if (g_lfs_mutex == NULL)
    {
        g_lfs_mutex = xSemaphoreCreateMutex();
    }

    xSemaphoreTake(g_lfs_mutex, portMAX_DELAY); // 上锁

    int err = lfs_mount(&g_lfs, &cfg);
    if (err)
    {
        // 提示用户正在格式化
        printf("FS Mount failed. Formatting Flash... PLEASE WAIT FOR 1~2 MINUTES!\r\n");
        lfs_format(&g_lfs, &cfg);
        err = lfs_mount(&g_lfs, &cfg);
    }

    if (!err)
    {
        g_fs_ready = true;
        lfs_file_open(&g_lfs, &g_log_file, "flight_log.txt", LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
        printf("LittleFS init success.\r\n");
        xSemaphoreGive(g_lfs_mutex); // 解锁
        return FSP_SUCCESS;
    }
    xSemaphoreGive(g_lfs_mutex);
    return FSP_ERR_ASSERTION;
}

void drv_log_save_line(const char *str)
{
    if (!g_fs_ready || str == NULL || g_lfs_mutex == NULL)
        return;

    xSemaphoreTake(g_lfs_mutex, portMAX_DELAY); // 上锁
    lfs_file_write(&g_lfs, &g_log_file, str, strlen(str));
    lfs_file_sync(&g_lfs, &g_log_file);
    xSemaphoreGive(g_lfs_mutex); // 解锁
}

// 优化：只读取最后一段日志，防止日志过大卡死 UI
uint32_t drv_log_read_all(char *buf, uint32_t buf_len)
{
    if (!g_fs_ready || buf == NULL || buf_len <= 1 || g_lfs_mutex == NULL)
        return 0;

    xSemaphoreTake(g_lfs_mutex, portMAX_DELAY); // 上锁

    // 获取文件当前总大小
    lfs_soff_t file_size = lfs_file_size(&g_lfs, &g_log_file);
    if (file_size <= 0)
    {
        xSemaphoreGive(g_lfs_mutex);
        return 0;
    }

    // 计算读取起始点 (只取最后 buf_len - 1 个字节)
    lfs_soff_t read_pos = 0;
    lfs_ssize_t to_read = buf_len - 1;
    if (file_size > to_read)
    {
        read_pos = file_size - to_read;
    }
    else
    {
        to_read = file_size;
    }

    // 将指针移动到计算好的位置
    lfs_file_seek(&g_lfs, &g_log_file, read_pos, LFS_SEEK_SET);

    // 读取内容
    lfs_ssize_t read_bytes = lfs_file_read(&g_lfs, &g_log_file, buf, to_read);
    if (read_bytes >= 0)
    {
        buf[read_bytes] = '\0';
    }

    // 【重要】将指针恢复到文件末尾，以免下一次 append 写入位置错误
    lfs_file_seek(&g_lfs, &g_log_file, 0, LFS_SEEK_END);

    xSemaphoreGive(g_lfs_mutex); // 解锁
    return (read_bytes > 0) ? (uint32_t)read_bytes : 0;
}

uint32_t drv_log_read_page(char *buf, uint32_t buf_len, uint32_t offset_from_end)
{
    if (!g_fs_ready || buf == NULL || buf_len <= 1 || g_lfs_mutex == NULL)
        return 0;

    xSemaphoreTake(g_lfs_mutex, portMAX_DELAY);

    // 1. 获取文件总大小
    lfs_soff_t file_size = lfs_file_size(&g_lfs, &g_log_file);

    // 如果文件为空，或者偏移量已经超过了文件总大小（翻到头了）
    if (file_size <= 0 || (lfs_soff_t)offset_from_end >= file_size)
    {
        xSemaphoreGive(g_lfs_mutex);
        return 0;
    }

    // 2. 计算实际读取长度和起始指针
    lfs_ssize_t to_read = buf_len - 1;
    lfs_soff_t read_pos = file_size - offset_from_end - to_read;

    // 3. 处理越界：如果算出来的起始位置小于 0，说明到了文件开头的最老的数据
    if (read_pos < 0)
    {
        to_read = file_size - offset_from_end; // 剩下多少读多少
        read_pos = 0;                          // 从头开始读
    }

    // 4. 移动指针并读取
    lfs_file_seek(&g_lfs, &g_log_file, read_pos, LFS_SEEK_SET);
    lfs_ssize_t read_bytes = lfs_file_read(&g_lfs, &g_log_file, buf, to_read);

    if (read_bytes >= 0)
    {
        buf[read_bytes] = '\0'; // 确保字符串安全结束
    }

    // 5. 【极其重要】将指针恢复到文件末尾，避免下一次 drv_log_save_line 写入位置错乱！
    lfs_file_seek(&g_lfs, &g_log_file, 0, LFS_SEEK_END);

    xSemaphoreGive(g_lfs_mutex);
    return (read_bytes > 0) ? (uint32_t)read_bytes : 0;
}