#ifndef __ESP_12F_H
#define __ESP_12F_H
#include "main.h"
#include "../pixel_clock_config.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include <time.h>

// ESP8266 初始化
void ESP8266_Init_Setup(void);

/**
 * @brief 连接wifi
 * @param ssid WiFi名
 * @param pwd 密码
 * @return true
 * @return false
 */
bool ESP8266_ConnectWiFi(char *ssid, char *pwd);

/**
 * @brief 连接tcp
 * @param ip_or_domain ip地址
 * @param port         端口
 * @return true
 * @return false
 */
bool ESP8266_Connect_tcp(char *ip_or_domain, char *port);
// --- 存储Wifi列表 ---
#define MAX_WIFI_LIST_NUM 30 // 最大提取数量
#define MAX_SSID_LEN 64      // SSID最大长度

typedef struct
{
    char ssid[MAX_SSID_LEN]; // 存放单个WiFi名称
} WiFi_Info_t;

// 定义结果列表
extern WiFi_Info_t WifiList[];
extern uint8_t WifiListCount; // 实际解析到的数量
// 定义一个结构体用来存天气数据
typedef struct
{
    char name[32];       // 城市名
    char code[5];        // 天气代码
    char weather[32];    // 天气
    char temperature[5]; // 温度
    time_t up_data_time; // 更新时间
} WeatherData_t;

// 定义全局或局部的变量
extern WeatherData_t CurrentWeather;
// 扫描当前可连接wifi
bool ESP8266_scan_wifi(void);
// 断开wifi连接
bool ESP8266_kill_WiFi(void);
// 获取网页信息
bool ESP8266_send_tcp(void);
typedef struct
{
    uint16_t year;  // 年 (例如 2025)
    uint8_t month;  // 月 (1-12)
    uint8_t day;    // 日 (1-31)
    uint8_t hour;   // 时 (0-23)
    uint8_t minute; // 分 (0-59)
    uint8_t second; // 秒 (0-59)
    uint8_t week;   // 星期 (0-6, 0代表周日)
} System_Time_t;    // 存放时间
extern System_Time_t now_time;

/**
 * @brief 将 NTP 获取的时间写入 STM32 内部 RTC
 * @param time: 包含年月日时分秒的结构体
 */
void RTC_Set_Time_From_NTP(System_Time_t *time);
// NTP 与 Unix 时间戳的差值
#define NTP_TIMESTAMP_DELTA 2208988800UL
// 北京时间时区偏移 (8小时)
#define TIMEZONE_OFFSET (8 * 3600)
time_t ESP8266_Get_NTP_Time(void);
/**
 * @brief 基础指令
        AT	测试 AT 启动	返回 OK 表示模块正常
        AT+RST	重启模块	柔性重启
        AT+GMR	查看版本信息	检查固件版本
        ATE0	关闭回显	推荐：发送命令后不重复返回命令本身，减轻单片机解析负担
        ATE1	开启回显	默认状态，调试时好用
        WiFi 连接相关
        指令	说明	示例/备注
        AT+CWMODE=1	设置 Station 模式	1=Station(连接路由), 2=AP(做热点), 3=Station+AP
        AT+CWJAP="YOUR_WIFI_SSID","YOUR_WIFI_PASSWORD"	连接路由	AT+CWJAP="YOUR_WIFI_SSID","YOUR_WIFI_PASSWORD"
        AT+CWQAP	断开连接
        AT+CIFSR	查询本地 IP	连接成功后查看分配到的 IP 地址
        网络通信 (TCP/UDP) 相关
        指令	说明	示例/备注
        AT+CIPSTART="TCP","IP",80	建立 TCP 连接	IP 可以是域名，端口通常是 80 (HTTP)
        AT+CIPCLOSE	关闭连接
        AT+CIPMODE=1	设置透传模式	1=透传(数据纯传输，不解析AT)，0=普通模式
        AT+CIPSEND	普通模式发送	先发 AT+CIPSEND=长度，等收到 > 后发数据
        AT+CIPSEND	透传模式发送	直接发 AT+CIPSEND，收到 > 后，发出的所有数据都会透传
        +++	退出透传	注意：不带回车换行，发送后等待 1秒
 *
 */

#endif
