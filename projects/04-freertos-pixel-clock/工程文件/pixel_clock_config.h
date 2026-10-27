#ifndef PIXEL_CLOCK_CONFIG_H
#define PIXEL_CLOCK_CONFIG_H

/* 本地配置不随源码提交；旧编译器可定义 PIXEL_CLOCK_USE_LOCAL_CONFIG。 */
#if defined(PIXEL_CLOCK_USE_LOCAL_CONFIG)
#include "pixel_clock_config.local.h"
#elif defined(__has_include)
#if __has_include("pixel_clock_config.local.h")
#include "pixel_clock_config.local.h"
#endif
#endif

#ifndef PIXEL_CLOCK_WIFI_SSID
#define PIXEL_CLOCK_WIFI_SSID ""
#endif
#ifndef PIXEL_CLOCK_WIFI_PASSWORD
#define PIXEL_CLOCK_WIFI_PASSWORD ""
#endif
#ifndef PIXEL_CLOCK_WEATHER_API_KEY
#define PIXEL_CLOCK_WEATHER_API_KEY ""
#endif

#endif
