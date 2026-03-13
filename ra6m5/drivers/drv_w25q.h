/* drv_w25q.h */
#ifndef DRV_W25Q_H_
#define DRV_W25Q_H_

#include "hal_data.h"
#include "lfs.h" // 新增：引入 LittleFS 头文件，以识别 lfs_block_t 等类型

/* --- 原有的基本接口 --- */
void W25QDrvInit(void);
unsigned int W25QDrvReadID(void);
int W25QDrvWrite(unsigned int dwAddr, unsigned char *buf, unsigned int dwSize);
int W25QDrvRead(unsigned int dwAddr, unsigned char *buf, unsigned int dwSize);

/* --- 新增：专供 LittleFS 调用的底层接口 --- */
int W25QDrvRead_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int W25QDrvProg_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int W25QDrvErase_LittleFS(const struct lfs_config *c, lfs_block_t block);
int W25QDrvSync_LittleFS(const struct lfs_config *c);

#endif /* DRV_W25Q_H_ */