# 第三方组件与声明

仓库根许可证适用于可按该许可发布的自有内容，不替代随源码附带的第三方许可、版权声明及使用限制。保留文件头和原始许可证。

| 组件 | 使用位置与声明 |
| --- | --- |
| STM32 HAL、标准外设库和 CMSIS | 各 STM32 工程中的驱动/库文件头；FOC 的 HAL/CMSIS 目录保留许可 |
| Renesas FSP、CMSIS 和生成配置 | RA8P1、RA6M5 的 `ra/`、`ra_cfg/`、`ra_gen/`；[RA8P1 CMSIS 许可](projects/02-ra8p1-smart-car/ra/arm/CMSIS_6/LICENSE) |
| FreeRTOS | [RA8P1 FreeRTOS 许可](projects/02-ra8p1-smart-car/ra/aws/FreeRTOS/LICENSE.md)，其余副本保留文件头声明 |
| LVGL 及其字体、示例资源 | [LVGL 目录](projects/02-ra8p1-smart-car/my-code/third_party/lvgl/)，内嵌组件适用各自许可 |
| RUHMI 模型生成代码、Ethos-U 和 TensorFlow 相关实现 | [部署源文件目录](projects/02-ra8p1-smart-car/my-code/ai/ruhmi_generated/kws_current/output/build/MCU/compilation/src/)，保留 EdgeCortix、Renesas、Apache/TensorFlow 原文件声明及硬件使用范围文字 |
| MPU6050/DMP、VL53L0X、OLED 字模 | 对应 `Hard Ware/` 或 `HardWare/` 驱动文件头，保留 InvenSense、ST、江协科技等来源文字 |
| FocControl 的 Python、Qt 等依赖 | [上位机第三方说明](projects/01-stm32g474-foc/FocControl/THIRD_PARTY_NOTICES.md) |

训练录音、校准样本、工具安装包和模型缓存不作为此仓库的公开内容。新增或重新分发第三方组件时，应同时保留其来源、版本和许可；本表是声明索引，不对未知来源内容补授许可。
