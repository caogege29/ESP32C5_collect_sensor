#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 传感器一帧采集数据: 驱动按自身能力填充对应字段 */
typedef struct {
    bool  has_acc;           /* 三轴加速度, 单位 m/s^2 */
    float acc_mps2[3];
    bool  has_gyr;           /* 三轴角速度, 单位 dps */
    float gyr_dps[3];
    bool  has_mag;           /* 三轴磁场, 单位 uT */
    float mag_ut[3];
    bool  has_temp;          /* 温度, 单位 ℃ */
    float temp_degc;
    bool  has_hum;           /* 湿度, 单位 %RH */
    float hum_pct;
    bool  has_press;         /* 气压, 单位 hPa */
    float press_hpa;

    /*
     * IMU 原始数据(供传输/上位机自行换算):
     * 单位与量程由驱动决定(driver 内部对应的 ±2G acc / ±250dps gyr 需自行换算)。
     */
    bool   has_acc_raw;      /* 是否存在三轴加速度原始 int16 数据 */
    int16_t acc_raw[3];
    bool   has_gyr_raw;      /* 是否存在三轴陀螺仪原始 int16 数据 */
    int16_t gyr_raw[3];
} sensor_data_t;

/*
 * 传感器驱动接口 —— 新增外挂传感器只需:
 *   1. 在 sensors/ 下实现一个 sensor_driver_t;
 *   2. 加入 sensor_manager.c 的 s_sensor_registry 注册表。
 * 框架启动时扫描 I2C 总线, 地址在线则调用 probe 自动识别。
 */
typedef struct {
    const char    *name;      /* 传感器类型名 */
    const uint8_t *addr_list; /* 候选 I2C 地址列表(按优先级) */
    uint8_t        addr_count;
    /* 探测识别: 地址在线时调用; 成功返回 ESP_OK, info 输出芯片识别信息 */
    esp_err_t (*probe)(uint8_t addr, char *info, size_t info_len);
    /* 读取一帧数据 */
    esp_err_t (*read)(uint8_t addr, sensor_data_t *out);
} sensor_driver_t;

/* 6轴IMU 驱动(sensors/sensor_imu_6axis.c) */
extern const sensor_driver_t imu_6axis_driver;

#ifdef __cplusplus
}
#endif
