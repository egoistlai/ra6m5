#ifndef DRV_WIFI_H
#define DRV_WIFI_H

#include "hal_data.h"
#include <stdio.h>

/* 定义 Wi-Fi 连接参数 */
typedef enum
{
    STA = 1,
    SoftAP = 2,
    APSTA = 3
} WorkType;

typedef enum
{
    TCP = 0,
    UDP = 1
} NetworkProtocol;

typedef enum
{
    Client = 0,
    Server = 1
} LocalRole;

typedef struct
{
    NetworkProtocol Protocl;
    LocalRole Role;
    char *IP;
    unsigned int RemotePort;
    unsigned int LocalPort;
    unsigned int SocketPort;
} ConnectInfo;

typedef struct
{
    char *ssid;
    char *pwd;
    char *server_ip;
    uint16_t server_port;
} wifi_config_t;

/* 导出函数 */
fsp_err_t drv_wifi_init(void); // 改名以符合 drv_ 风格
int WiFiBtDevSetWorkType(WorkType type);
int WiFiBtDevEnableDHCP(void);
int WiFiBtDevDisableDHCP(const char *ip, const char *netmask, const char *gateway);
int WiFiBtDevNetScan(void);
int WiFiBtDevConnectWiFi(const char *name, const char *password);
int WiFiBtDevDisconnectWiFi(void);
int WiFiBtDevGetLocalIP(void);
int WiFiBtDevConnect(ConnectInfo *info);
int WiFiBtDevDisconnect(ConnectInfo info);
int WiFiBtDevWrite(ConnectInfo info, unsigned char *buf, unsigned int length);

void WiFi_Init(void);
int WiFi_Connect_AP(const char *ssid, const char *pwd);
int WiFi_Connect_TCP(const char *ip, uint16_t port);
int WiFi_Send(uint8_t *data, uint32_t len);
void WiFi_Get_IP(void);
int WiFi_Set_STA_Mode(void);
int WiFi_Enable_DHCP(void);
int WiFi_Set_RPTM(void);


#endif /* DRV_WIFI_H */