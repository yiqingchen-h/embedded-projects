#ifndef FOC_MT6835_H
#define FOC_MT6835_H
#include <stdbool.h>
#include <stdint.h>

/* 同一片选内的模式3、MSB优先事务，板卡负责时序与有限超时。 */
typedef bool (*mt6835_transfer_t)(const uint8_t *tx, uint8_t *rx, uint16_t count);

typedef struct {
    /* 21位原始机械角，0～2097151。 */
    uint32_t angle;
    /* bit0超速、bit1弱磁场、bit2欠压；与通信校验分开判断。 */
    uint8_t status;
} mt6835_sample_t;

/* 将传输失败与数据损坏分开，便于定位通信问题。 */
typedef enum { MT6835_OK, MT6835_IO_ERROR, MT6835_CRC_ERROR } mt6835_result_t;

typedef struct {
    /* 0x00D和0x011的原始影子寄存器值；只读，包含需保留的厂商位。 */
    uint8_t hysteresis, bandwidth;
} mt6835_config_t;

/** @brief 在USER_ID易失影子寄存器执行双图样探测并恢复原值，不写EEPROM、不触发标定。
 * @param transfer 板卡事务接口。 @return 双向通信和恢复均成功为true。 */
bool mt6835_probe(mt6835_transfer_t transfer);
/** @brief 双读迟滞和带宽影子寄存器，不写配置或EEPROM。
 * @param config 成功且两次一致时返回两个原始字节，失败不覆盖。
 * @return 已完成探测后，在主循环SPI空闲时读取成功。 */
bool mt6835_read_config(mt6835_config_t *config);
/** @brief 仅修改0x00D低3位的易失迟滞影子，不写EEPROM；失败尝试恢复原字节。
 * @param code 仅4..7，对应0/0.003/0.006/0.011度。
 * @return 写回双读一致且带宽与其余位保持；调用者须停机并保存原配置。 */
bool mt6835_set_hysteresis(uint8_t code);
/** @brief 突发读取角度、状态和CRC；失败不修改输出。
 * @param sample 成功时填充的样本。 @return 传输或CRC检查结果；磁场状态另行判断。 */
mt6835_result_t mt6835_read(mt6835_sample_t *sample);
/** @brief CRC-8/SMBUS，初值0、多项式0x07、不反射。
 * @param data 三个角度与状态字节。 @return 校验值。 */
uint8_t mt6835_crc(const uint8_t data[3]);
#endif
