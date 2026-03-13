/* drv_littlefs_log.h */
#ifndef DRV_LITTLEFS_LOG_H_
#define DRV_LITTLEFS_LOG_H_

#include "hal_data.h"
#include "lfs.h" // 从 ff.h 替换为 lfs.h

/* 错误代码定义 (保留原有结构，方便兼容) */
typedef enum
{
    LOG_OK = 0,
    LOG_ERR_MOUNT,
    LOG_ERR_OPEN,
    LOG_ERR_WRITE,
    LOG_ERR_READ
} log_res_t;

/* 函数声明：接口名称保持不变，业务线程可以直接调用 */
fsp_err_t drv_log_fs_init(void);                        // 初始化文件系统
void drv_log_save_line(const char *str);                // 追加写入一行日志
uint32_t drv_log_read_all(char *buf, uint32_t buf_len); // 读取日志内容
uint32_t drv_log_read_page(char *buf, uint32_t buf_len, uint32_t offset_from_end);

#endif /* DRV_LITTLEFS_LOG_H_ */