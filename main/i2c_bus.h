#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 I2C 主机总线（引脚/频率等参数见 config.h） */
esp_err_t i2c_bus_init(void);

/** 获取 I2C 主机总线句柄 */
i2c_master_bus_handle_t i2c_bus_get_bus(void);

/** 按 7bit 地址获取(懒创建并缓存)总线上的设备句柄; 失败返回 NULL */
i2c_master_dev_handle_t i2c_bus_dev_get(uint8_t addr);

/** 向指定设备写入单个寄存器 */
esp_err_t i2c_bus_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);

/** 从指定设备的寄存器 reg 开始连续读取 len 字节到 buf */
esp_err_t i2c_bus_read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);

/** 扫描总线, 将在线设备的 7bit 地址写入 addrs, 返回在线设备数量 */
int i2c_bus_scan(uint8_t *addrs, int max);

#ifdef __cplusplus
}
#endif
