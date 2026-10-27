#include "ESP_12F.h"
WeatherData_t CurrentWeather;                                                 // 天气数据
bool get_CIPMUX_mode(UART_Rx_Struct *rx_struct, char *mode_str, int max_len); // 读取 CIPMUX
bool Parse_Weather_Data(char *json_str, WeatherData_t *out_data);             // 解析天气数据
static bool ESP8266_Exit_Transparent(void);                                   // 退出透传
time_t ISO8601_To_Timestamp(char *str);
void ESP8266_Clear_Rx(void)
{
    EspRx.RxLen = 0;
    EspRx.RxFlag = 0;
    //memset(EspRx.RxBuffer, 0, RX_BUFFER_SIZE);
    HAL_UART_Receive_DMA(&UART_HandleType3, EspRx.RxBuffer, RX_BUFFER_SIZE); // 重新开 DMA 接收
}

static bool ESP8266_SendCmd(char *cmd, char *expect_str, uint32_t timeout)
{
    ESP8266_Clear_Rx();



    ESP_SendString(cmd);
    uint32_t time_cnt = 0;
    while (time_cnt < timeout)
    {
        if (EspRx.RxFlag == 1) // 等待响应
        {
            if (strstr((const char *)EspRx.RxBuffer, expect_str) != NULL)
            {                // 收到目标字符串
                return true;
            }
            else
            {
                EspRx.RxLen = 0;
                EspRx.RxFlag = 0;
                // 直接覆盖旧数据
                HAL_UART_Receive_DMA(&UART_HandleType3, EspRx.RxBuffer, RX_BUFFER_SIZE);
            }
        }
        vTaskDelay(10);
        time_cnt += 10;
    }

    U1_print("Cmd: %s Fail. Got: \r\n", cmd);
    HAL_UART_Transmit(&UART_HandleType3, EspRx.RxBuffer, strlen(EspRx.RxBuffer), 100);
    return false;
}
bool Send_Cmd_With_Retry(char *cmd, char *expect_str, uint8_t max_retries)
{
    uint8_t attempts = 0;
    if (ESP8266_SendCmd(cmd, expect_str, 500))
    {
        return true;
    }

    while (attempts < max_retries)
    {
        if (ESP8266_SendCmd(cmd, expect_str, 500))
        {
            return true;
        }
        attempts++;
    }

    return false;
}
void ESP8266_Init_Setup(void)
{


    if (!ESP8266_SendCmd("AT\r\n", "OK", 500))
    {
        U1_print("ESP8266 no response, trying to fix...\r\n");

        ESP8266_Exit_Transparent();
        vTaskDelay(500);

        ESP8266_SendCmd("AT+RST\r\n", "OK", 200);
        vTaskDelay(2000);
    }

    if (!Send_Cmd_With_Retry("AT\r\n", "OK", 10))
    {
        U1_print("ESP8266 HW Error!\r\n");
        return;
    }

    if (!Send_Cmd_With_Retry("ATE0\r\n", "OK", 5))
        return;

    Send_Cmd_With_Retry("AT+CWMODE=1\r\n", "OK", 5);

    if (ESP8266_SendCmd("AT+GMR\r\n", "OK", 500))
    {
        U1_print("versions:%s\r\t", EspRx.RxBuffer);
    }

    if (Send_Cmd_With_Retry("AT+CIPMUX=0\r\n", "OK", 5))
    {
        char read_CIPMUX[50];
        if (get_CIPMUX_mode(&EspRx, read_CIPMUX, 50))
        {
            U1_print("Mode Set OK: %s\r\n", read_CIPMUX);
        }
    }
    else
    {
        U1_print("Set CIPMUX Fail\r\n");
    }

    U1_print("ESP8266 Init Finished\r\n");
}

bool ESP8266_ConnectWiFi(char *ssid, char *pwd)
{
    char cmd_buf[128];
    sprintf(cmd_buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);

    if (ESP8266_SendCmd(cmd_buf, "OK", 10000))
    {
        return true;
    }
    return false;
}
bool ESP8266_kill_WiFi(void)
{
    if (ESP8266_SendCmd("AT+CWQAP\r\n", "OK", 10000))
    {
        return true;
    }
    return false;
}
WiFi_Info_t WifiList[MAX_WIFI_LIST_NUM];
uint8_t WifiListCount = 0;
void Parse_WiFi_List(UART_Rx_Struct *uart_data);
bool ESP8266_scan_wifi(void)
{
    if (ESP8266_SendCmd("AT+CWLAP\r\n", "OK", 10000))
    {
        Parse_WiFi_List(&EspRx);
        return true;
    }
    return false;
}
bool ESP8266_Connect_tcp(char *ip_or_domain, char *port)
{
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 500);
    ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", 200);
    char cmd_buf[256];
    vTaskDelay(200);
    sprintf(cmd_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip_or_domain, port);
    if (!ESP8266_SendCmd(cmd_buf, "OK", 10000))
    {
        return false; // 连接失败
    }
    return true;
}
static bool ESP8266_Exit_Transparent(void)
{

    vTaskDelay(100);       // 发送前留空窗
    ESP_SendString("+++"); // 退出透传不要带换行
    vTaskDelay(1000);      // 等模块退出透传

    if (ESP8266_SendCmd("AT\r\n", "OK", 1000))
    {
        ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 500);
        return true;
    }
    return false;
}

void Debug_Hex_Dump(uint8_t *data, uint16_t len)
{
    U1_print("[HEX DUMP] Len: %d\r\n", len);
    if (len > 128)
        len = 128; // 最多打印 128 字节
    for (int i = 0; i < len; i++)
    {
        U1_print("%02X ", data[i]);
        if ((i + 1) % 16 == 0)
            U1_print("\r\n"); // 每 16 字节换行
    }
    U1_print("\r\n[HEX END]\r\n");
}
// 发送天气请求


bool ESP8266_send_tcp(void)
{
    char http_cmd[256];
    char send_len_cmd[32];

    // 心知天气 key
    const char *my_api_key = PIXEL_CLOCK_WEATHER_API_KEY;
    /* 未配置天气服务时不发送空密钥请求。 */
    if (my_api_key[0] == '\0')
    {
        return false;
    }

    // 组 HTTP 请求
    sprintf(http_cmd,
            "GET /v3/weather/now.json?location=ip&key=%s&language=en&unit=c HTTP/1.1\r\n"
            "Host: api.seniverse.com\r\n"
            "Connection: close\r\n"
            "\r\n",
            my_api_key);

    int len = strlen(http_cmd);

    // 发长度
    sprintf(send_len_cmd, "AT+CIPSEND=%d\r\n", len);
    ESP8266_Clear_Rx();
    ESP_SendString(send_len_cmd);

    // 等待 >
    uint32_t time_cnt = 0;
    bool ready_to_send = false;
    while (time_cnt < 2000)
    {
        if (EspRx.RxFlag == 1)
        {
            if (strstr((const char *)EspRx.RxBuffer, ">") != NULL)
            {
                ready_to_send = true;
                break;
            }
            ESP8266_Clear_Rx();
        }
        vTaskDelay(10);
        time_cnt += 10;
    }

    if (!ready_to_send)
    {
        // 调试时可以打开这行
        ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 200);
        return false;
    }

    // 发 HTTP 内容
    ESP8266_Clear_Rx();
    ESP_SendString(http_cmd);

    // 等待返回
    time_cnt = 0;
    bool got_data = false;

    while (time_cnt < 8000)
    {
        if (EspRx.RxFlag == 1)
        {
            if (strstr((const char *)EspRx.RxBuffer, "{") != NULL)
            {
                got_data = true;
                break; 
            }
            else
            {
                ESP8266_Clear_Rx();
            }
        }
        vTaskDelay(20);
        time_cnt += 20;
    }

    if (got_data)
    {
        // 解析天气数据
        if (Parse_Weather_Data((char *)EspRx.RxBuffer, &CurrentWeather))
        {
            ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 200);
            return true;
        }
    }

    // 超时或失败
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 200);
    return false;
}

// 读取 CIPMUX






static bool get_CIPMUX_mode(UART_Rx_Struct *rx_struct, char *mode_str, int max_len)
{
    if (rx_struct == NULL || mode_str == NULL || rx_struct->RxLen == 0)
    {
        return false;
    }
    // 找 +CIPMUX:
    char *start = strstr((char *)rx_struct->RxBuffer, "+CIPMUX:");
    if (start == NULL)
    {
        return false;
    }
    // 跳过前缀
    start += 8;

    // 拷模式值
    int i = 0;
    while (*start >= '0' && *start <= 'z' && i < max_len - 1)
    {
        mode_str[i++] = *start;
        start++;
    }

    mode_str[i] = '\0'; // 补结束符

    return (i > 0) ? true : false;
}

// 解析扫描到的 WiFi 列表




static void Parse_WiFi_List(UART_Rx_Struct *uart_data)
{
    char *pStart = (char *)uart_data->RxBuffer;
    char *pSSID_Start = NULL;
    char *pSSID_End = NULL;

    // 补字符串结束符
    if (uart_data->RxLen < RX_BUFFER_SIZE)
    {
        uart_data->RxBuffer[uart_data->RxLen] = '\0';
    }
    else
    {
        uart_data->RxBuffer[RX_BUFFER_SIZE - 1] = '\0';
    }

    // 清空旧列表
    memset(WifiList, 0, sizeof(WifiList));
    WifiListCount = 0;

    // 逐条解析
    while (WifiListCount < MAX_WIFI_LIST_NUM)
    {
        // 找 +CWLAP
        pStart = strstr(pStart, "+CWLAP:(");

        if (pStart == NULL)
        {
            break;
        }

        // 找 SSID 起始引号
        // 形如 +CWLAP:(enc, "SSID", ...)
        pSSID_Start = strchr(pStart, '"');

        if (pSSID_Start != NULL)
        {
            // 找 SSID 结束引号
            pSSID_End = strchr(pSSID_Start + 1, '"');

            if (pSSID_End != NULL)
            {
                // 算 SSID 长度
                int len = pSSID_End - (pSSID_Start + 1);

                // 长度检查
                if (len > 0 && len < MAX_SSID_LEN)
                {
                    // 拷贝 SSID
                    memcpy(WifiList[WifiListCount].ssid, pSSID_Start + 1, len);
                    // 补结束符
                    WifiList[WifiListCount].ssid[len] = '\0';

                    WifiListCount++; // 数量加 1
                }
            }
        }

        // 挪到下一条
        // 防止卡死
        if (pSSID_End != NULL)
        {
            pStart = pSSID_End + 1;
        }
        else
        {
            pStart++;
        }
    }
}

// 取 JSON 字段






static void copy_json_value(char *src, const char *key, char *dest, int max_len)
{
    // 找 key
    char *start = strstr(src, key);
    if (start)
    {
        start += strlen(key);
        while (*start == ':' || *start == ' ' || *start == '"' || *start == '\t')
        {
            if (*start == '\0')
                return;
            start++;
        }

        int i = 0;
        while (*start != '"' && *start != ',' && *start != '}' && *start != '\0' && i < max_len - 1)
        {
            dest[i++] = *start++;
        }
        dest[i] = '\0'; 
    }
    else
    {
        dest[0] = '\0';
    }
}
// 解析天气数据




bool Parse_Weather_Data(char *json_str, WeatherData_t *out_data)
{
    if (json_str == NULL || out_data == NULL)
        return false;

    copy_json_value(json_str, "\"name\"", out_data->name, sizeof(out_data->name));

    copy_json_value(json_str, "\"code\"", out_data->code, sizeof(out_data->code));

    copy_json_value(json_str, "\"text\"", out_data->weather, sizeof(out_data->weather));

    copy_json_value(json_str, "\"temperature\"", out_data->temperature, sizeof(out_data->temperature));

    char last_update[32];
    copy_json_value(json_str, "\"last_update\"", last_update, sizeof(last_update));

    out_data->up_data_time = ISO8601_To_Timestamp(last_update);

    return true;
}

// 时间戳转日期




void Timestamp_To_Date(time_t timestamp, System_Time_t *out_time)
{
    static const uint8_t month_days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint32_t days;
    uint16_t year = 1970;
    uint8_t i;
    uint16_t days_in_year;

    out_time->second = timestamp % 60;
    timestamp /= 60;
    out_time->minute = timestamp % 60;
    timestamp /= 60;
    out_time->hour = timestamp % 24;

    days = timestamp / 24;

    out_time->week = (days + 4) % 7;
    while (1)
    {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
        {
            days_in_year = 366;
        }
        else
        {
            days_in_year = 365;
        }

        if (days < days_in_year)
        {
            break;
        }
        days -= days_in_year;
        year++;
    }
    out_time->year = year;

    for (i = 1; i <= 12; i++)
    {
        uint8_t dim = month_days[i];

        if (i == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)))
        {
            dim = 29;
        }

        if (days < dim)
        {
            out_time->month = i;
            out_time->day = days + 1;
            break;
        }
        days -= dim;
    }
}
System_Time_t now_time; // 当前时间
// 从 NTP 取时间



time_t ESP8266_Get_NTP_Time(void)
{
    uint8_t ntp_packet[48] = {0};
    time_t timestamp = 0;

    ntp_packet[0] = 0x1B; 

    U1_print("Start NTP...\r\n");

    ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", 1000);
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 1000);

    if (!ESP8266_SendCmd("AT+CIPSTART=\"UDP\",\"203.107.6.88\",123\r\n", "OK", 5000))
    {
        U1_print("NTP Connect Fail\r\n");
        return 0;
    }

    ESP8266_Clear_Rx();
    ESP_SendString("AT+CIPSEND=48\r\n");

    uint32_t time_cnt = 0;
    bool ready_to_send = false;
    while (time_cnt < 2000)
    {
        if (EspRx.RxFlag == 1)
        {
            if (strstr((const char *)EspRx.RxBuffer, ">") != NULL)
            {
                ready_to_send = true;
                break;
            }
            ESP8266_Clear_Rx();
        }
        vTaskDelay(10);
        time_cnt += 10;
    }

    if (!ready_to_send)
    {
        U1_print("NTP > Timeout\r\n");
        ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 200);
        return 0;
    }

    ESP8266_Clear_Rx();
    HAL_UART_Transmit(&UART_HandleType3, ntp_packet, 48, 1000);

    time_cnt = 0;
    bool got_data = false;
    while (time_cnt < 4000)
    {
        if (EspRx.RxFlag == 1)
        {

            char *ipd_ptr = strstr((char *)EspRx.RxBuffer, "+IPD,");
            if (ipd_ptr != NULL)
            {
                char *colon_ptr = strchr(ipd_ptr, ':');
                if (colon_ptr != NULL)
                {
                    uint8_t *data_ptr = (uint8_t *)(colon_ptr + 1);

                    // 取秒数
                    uint32_t high_word = 0;
                    high_word |= ((uint32_t)data_ptr[40] << 24);
                    high_word |= ((uint32_t)data_ptr[41] << 16);
                    high_word |= ((uint32_t)data_ptr[42] << 8);
                    high_word |= ((uint32_t)data_ptr[43]);

                    if (high_word > 0)
                    {
                        timestamp = high_word - 2208988800UL; 
                        timestamp += (8 * 3600);              // 东八区
                        got_data = true;
                        break;
                    }
                }
            }
            ESP8266_Clear_Rx();
        }
        vTaskDelay(10);
        time_cnt += 10;
    }

    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 200);

    if (got_data)
    {
        Timestamp_To_Date(timestamp, &now_time);
        return timestamp;
    }

    U1_print("NTP Timeout\r\n");
    return 0;
}

// 写 RTC



void RTC_Set_Time_From_NTP(System_Time_t *time)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = time->hour;
    sTime.Minutes = time->minute;
    sTime.Seconds = time->second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        U1_print("RTC Set Time Error\r\n");
    }


    if (time->year >= 2000)
    {
        sDate.Year = time->year - 2000;
    }
    else
    {
        sDate.Year = 0; 
    }

    sDate.Month = time->month;
    sDate.Date = time->day;

    if (time->week == 0)
    {
        sDate.WeekDay = RTC_WEEKDAY_SUNDAY; 
    }
    else
    {
        sDate.WeekDay = time->week;
    }

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        U1_print("RTC Set Date Error\r\n");
    }else{

        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
    }

    U1_print("RTC Synced Successfully!\r\n");
}





time_t ISO8601_To_Timestamp(char *str)
{
    struct tm t = {0};
    int year, month, day, hour, minute, second;
    int tz_h = 0, tz_m = 0;
    char tz_sign = '+';

    // 解析年月日时分秒和时区


    int count = sscanf(str, "%d-%d-%dT%d:%d:%d%c%d:%d",
                       &year, &month, &day,
                       &hour, &minute, &second,
                       &tz_sign, &tz_h, &tz_m);

    if (count < 6)
    {
        return 0;
    }

    t.tm_year = year - 1900;
    t.tm_mon = month - 1;    
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = 0; // 不处理夏令时

    // 生成时间戳

    time_t timestamp = mktime(&t);


    long offset_seconds = tz_h * 3600 + tz_m * 60;

    if (tz_sign == '+')
    {
        timestamp -= offset_seconds;
    }
    else if (tz_sign == '-')
    {
        timestamp += offset_seconds;
    }

    return timestamp;
}
