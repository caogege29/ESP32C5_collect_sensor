#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 一次采集结果: 数据来源(驱动名+地址) + 一帧数据 */
typedef struct {
    const char   *name;  /* 传感器类型名(如 "6-axis IMU") */
    uint8_t       addr;  /* 传感器 I2C 地址 */
    sensor_data_t data;  /* 一帧采集数据 */
} sensor_sample_t;

/**
 * 初始化传感器管理器:
 *   扫描 I2C 总线 -> 遍历驱动注册表 -> 按地址探测识别 -> 登记在线传感器实例。
 */
esp_err_t sensor_manager_init(void);

/** 采集所有在线传感器各一帧, 返回成功采集的数量 */
int sensor_manager_collect(sensor_sample_t *out, int max);

/** 将采集结果格式化为 JSON(含传感器名与地址, 便于 P4 端区分数据来源) */
size_t sensor_sample_to_json(const sensor_sample_t *s, char *buf, size_t len);

#ifdef __cplusplus
}
#endif
