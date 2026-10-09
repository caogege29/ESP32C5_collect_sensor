#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 2 路压力传感器 ADC 采样。
 * 传感器为电阻式压敏电阻(RP-C18.3), 经 10kΩ 上拉分压后接入 ADC:
 *   传感器1 -> GPIO4(ADC1_CH3), 传感器2 -> GPIO6(ADC1_CH5)。
 */
esp_err_t pressure_init(void);

/**
 * 读取 2 路压力值(单位 g/克)。
 * 换算链路: ADC raw -> 分压电压 -> 传感器电阻 Rp -> 力 F(g)=(A/R_kΩ)^(1/B)。
 * @param press_g[2]  输出两路压力值(g)
 */
esp_err_t pressure_read(float press_g[2]);

#ifdef __cplusplus
}
#endif