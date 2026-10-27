# hardware/LCD

## 当前配置
- 当前 `UI_PORT_USE_MIPI_LCD=1`，默认使用 MIPI ST7701P，旋转 `0U`，LVGL 逻辑分辨率为 `480x800`。
- 可选 SPI ST7789 使用 `lcd_st7789.c/.h`，走 `g_sci_spi2 + g_transfer1(DMAC TX)`，默认逻辑分辨率为横屏 `280x240`。
- MIPI 屏使用 `lcd_mipi_st7701p.c/.h`，走 `g_display0 + g_mipi_dsi0`，由 LVGL 部分刷新到 GLCDC framebuffer。
- `lcd_callbacks.c` 存放 LCD 相关 FSP 回调转发，避免把显示回调继续放在 `timer.c`。
- 在 `ui_port.h` 中通过 `UI_PORT_USE_MIPI_LCD` 切换屏幕，`0` 为 SPI，`1` 为 MIPI。

## 工程接入
- 已在 Keil 工程引用的源文件：`my-code/hardware/LCD/lcd_mipi_st7701p.c`
- 对应头文件：`my-code/hardware/LCD/lcd_mipi_st7701p.h`
- 已在 Keil 工程引用的源文件：`my-code/hardware/LCD/lcd_callbacks.c`
- 已在 Keil 工程引用的源文件：`my-code/hardware/LCD/lcd_st7789.c`
- 对应头文件：`my-code/hardware/LCD/lcd_st7789.h`
- UI 接口位于 [software/ui](../../software/ui/)，LVGL 源码位于 [third_party/lvgl](../../third_party/lvgl/)。当前工程已引用相关源文件，修改显示配置时需同步核对 FSP 和 UI 端口。

## MIPI ST7701P 引脚和配置
- 屏幕：`P040B019-MIPI`，物理 `480x800`，默认 UI 逻辑分辨率 `480x800`，2 lane MIPI，驱动 IC `ST7701P`。
- DSI 输出：当前 FSP 配置为 RGB888 video stream，GLCDC input0 framebuffer 为 32-bit RGB888。
- EoTP：当前 FSP 配置为 enabled。
- 旋转：`ui_port.c` 中 `UI_PORT_MIPI_ROTATION` 默认 `0U`；选择 `90U` 或 `270U` 时逻辑分辨率切换为 `800x480`。
- RST：`P013`，工程符号名 `LCD_RST`。
- BL：`P012`，工程符号名 `LCD_BL`。当前生成配置里该脚是输入，MIPI 驱动初始化时会改成输出后再拉高。
- TE：`P411`，工程符号名 `MIPI_TE`，当前只按 FSP MIPI TE 引脚配置保留，驱动暂不读取。

## SPI ST7789 引脚
- 默认逻辑分辨率：`280x240` 横屏。
- 方向切换：`lcd_st7789.h` 中 `LCD_ST7789_ROTATION` 默认 `90U`，方向相反时改为 `270U`。
- SCL/SCK：P405，SCI2 SCK。
- SDA/MOSI：P406，SCI2 TXD。
- CS：P005。
- DC：P006。
- RES：P402。
- BLK：P403。
