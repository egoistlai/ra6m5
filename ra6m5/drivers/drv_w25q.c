/*
 * drv_w25q64.c
 *
 *  Created on: 2023年4月17日
 *      Author: slhuan
 */

#include "drv_w25q.h"
#include <stdio.h>
#include "hal_data.h"
#include "FreeRTOS.h"
#include "lfs.h"

#define SECTOR_SIZE 512
#define FLASH_SECTOR_SIZE 4096 // W25Q 物理扇区 4KB
#define FLASH_TOTAL_SIZE (8 * 1024 * 1024)

static void W25QDrvWaitXfer(void);
static bool g_flash_initialized = false;

void W25QDrvInit(void)
{
    fsp_err_t err = g_qspi0.p_api->open(g_qspi0.p_ctrl, g_qspi0.p_cfg);
    if(FSP_SUCCESS != err)
    {
        printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
        return;
    }
}

unsigned int W25QDrvReadID(void)
{
    uint8_t cmd = 0x9F;
    uint8_t id[3] = {0};
    g_qspi0.p_api->directWrite(g_qspi0.p_ctrl, &cmd, 1, true);
    g_qspi0.p_api->directRead(g_qspi0.p_ctrl, id, 3);
    unsigned int ID = (id[0]<<16) | (id[1]<<8) | (id[2]);
    return ID;
}

static void W25QDrvWaitXfer(void)
{
    spi_flash_status_t status = {.write_in_progress = true};
    while(status.write_in_progress == true)
    {
        fsp_err_t err = g_qspi0.p_api->statusGet(g_qspi0.p_ctrl, &status);
        if(FSP_SUCCESS != err)
        {
            printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
            return;
        }
    }
}

int W25QDrvWrite(unsigned int dwAddr, unsigned char *buf, unsigned int dwSize)
{
    /* 第1步 擦除扇区 */
    unsigned int dwOffsetAddr = 0;
    unsigned int dwStartAddr = dwAddr/4096*4096;
    unsigned int dwSectorCount = (dwSize + dwAddr%4096)/4096 + 1;
    while(dwSectorCount--)
    {
        unsigned int nAddr = dwStartAddr + dwOffsetAddr;
        fsp_err_t  err = g_qspi0.p_api->erase(g_qspi0.p_ctrl, (uint8_t*)(QSPI_DEVICE_START_ADDRESS+nAddr), 4096);
        if(FSP_SUCCESS != err)
        {
            printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
            return -1;
        }
        W25QDrvWaitXfer();
        dwOffsetAddr += 4096;
    }

    /* 第2部 分页写 */
    unsigned int dwPageCount = (dwSize + dwAddr%256)/256 + 1;
    // 如果从起始地址开始偏移dwSize都没有超过一页就从起始地址开始写dwSize字节
    if(dwPageCount == 1)
    {
        fsp_err_t err = g_qspi0.p_api->write(g_qspi0.p_ctrl, buf, (uint8_t*)(QSPI_DEVICE_START_ADDRESS+dwAddr), dwSize);
        if(FSP_SUCCESS != err)
        {
            printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
            return -1;
        }
        W25QDrvWaitXfer();
    }
    else
    {
        unsigned int nAddr = dwAddr;
        // 如果超过了一页则先将起始地址所在页填充满
        unsigned int dwFirstBytes = 256 - dwAddr%256;
        // 计算出写满起始地址所在页后剩余要写的数据个数
        unsigned int dwRestBytes = dwSize - dwFirstBytes;
        // 填充起始地址所在页
        fsp_err_t err = g_qspi0.p_api->write(g_qspi0.p_ctrl, buf, (uint8_t*)(QSPI_DEVICE_START_ADDRESS+nAddr), dwFirstBytes);
        if(FSP_SUCCESS != err)
        {
            printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
            return -1;
        }
        W25QDrvWaitXfer();

        // 将W25Q的地址偏移到下一页的起始地址
        nAddr += dwFirstBytes;
        // 要写入的数据buff地址也要更新
        buf += dwFirstBytes;
        // 开始将剩下的数据写入到W25Q
        while(dwRestBytes != 0)
        {
            unsigned int nBytes = 0;
            // 剩下的数据个数不满一页的话
            // 最后一次就将剩下的数据全部写入到最后要写的这一页
            if(dwRestBytes <= 256)
                nBytes = dwRestBytes;
            else
                nBytes = 256;

            err = g_qspi0.p_api->write(g_qspi0.p_ctrl, buf, (uint8_t*)(QSPI_DEVICE_START_ADDRESS+nAddr), nBytes);
            if(FSP_SUCCESS != err)
            {
                printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
                return -1;
            }
            W25QDrvWaitXfer();

            // W25Q地址和buf地址偏移，剩余个数递减刚才写入的数据个数
            nAddr += nBytes;
            buf += nBytes;
            dwRestBytes -= nBytes;
        }
    }
    // 返回写入数据的个数
    return (int)dwSize;
}

int W25QDrvRead(unsigned int dwAddr, unsigned char *buf, unsigned int dwSize)
{
    unsigned char cmd[4] = {0};

    cmd[0] = 0x03;  // read data command
    cmd[1] = (dwAddr>>16)&0xFF;
    cmd[2] = (dwAddr>>8)&0xFF;
    cmd[3] = (dwAddr)&0xFF;
    
    W25QDrvWaitXfer();
    
    fsp_err_t err = g_qspi0.p_api->directWrite(g_qspi0.p_ctrl, cmd, 4, true);
    if(FSP_SUCCESS != err)
    {
        printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
        return -1;
    }

    err = g_qspi0.p_api->directRead(g_qspi0.p_ctrl, buf, dwSize);
    if(FSP_SUCCESS != err)
    {
        printf("Function:%s\tLine:%d\r\n", __FUNCTION__, __LINE__);
        return -1;
    }

    // 返回独处数据的个数
    return (int)dwSize;
}
/* drv_w25q.c 中新增纯粹的写和擦除接口 */

// 1. 纯粹的写数据 (不带擦除)
int W25QDrvProg_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t addr = block * c->block_size + off;

    // 直接调用 FSP 的 write，不进行任何 erase 操作
    fsp_err_t err = g_qspi0.p_api->write(g_qspi0.p_ctrl, (uint8_t *)buffer, (uint8_t *)(QSPI_DEVICE_START_ADDRESS + addr), size);
    if (FSP_SUCCESS != err)
        return LFS_ERR_IO;

    W25QDrvWaitXfer();
    return LFS_ERR_OK;
}

// 2. 纯粹的擦除接口
int W25QDrvErase_LittleFS(const struct lfs_config *c, lfs_block_t block)
{
    uint32_t addr = block * c->block_size;

    // 擦除一个 4KB 的物理扇区
    fsp_err_t err = g_qspi0.p_api->erase(g_qspi0.p_ctrl, (uint8_t *)(QSPI_DEVICE_START_ADDRESS + addr), c->block_size);
    if (FSP_SUCCESS != err)
        return LFS_ERR_IO;

    W25QDrvWaitXfer();
    return LFS_ERR_OK;
}

// 3. 包装一下你现有的读接口
int W25QDrvRead_LittleFS(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    // 计算绝对物理地址
    uint32_t addr = block * c->block_size + off;

    // 【修改点】：废弃 memcpy，直接调用官方的 W25QDrvRead
    // 官方函数返回读取成功的字节数，如果等于 size 说明成功
    if (W25QDrvRead(addr, (unsigned char *)buffer, size) == (int)size)
    {
        return LFS_ERR_OK;
    }

    return LFS_ERR_IO;
}

// 4. 同步接口 (对于 W25Q 这种，无需额外操作，返回 OK 即可)
int W25QDrvSync_LittleFS(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}