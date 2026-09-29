/*
 * BMI323 高精度6轴IMU 传感器驱动 (Bosch)
 * 原理图: U3/U7 模块座 (HC-PM254-5.0H-1x6PZ), 引脚见 config.h:
 *   SCL=GPIO3, SDA=GPIO2, INT1=GPIO0, INT2=GPIO1
 *
 * BMI323 关键特性:
 *   - 16 位寄存器架构: 每个寄存器读/写均按 2 字节处理, LSB 在前;
 *   - 连续读时寄存器地址自动递增, 可一次读取多组数据;
 *   - I2C 地址由模块电路固定为 0x4A (见 config.h);
 *   - CHIP_ID(0x00) 固定为 0x43;
 *   - 上电或软复位后 I2C 接口需 dummy read 激活(datasheet 启动序列);
 *   - 传感器默认处于关闭状态, 需写 ACC_CONF/GYR_CONF 使能。
 */
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "i2c_bus.h"
#include "sensor_driver.h"

static const char *TAG = "bmi323";

/* ---------------------------- BMI323 寄存器 ------------------------------- */
#define BMI323_REG_CHIP_ID     0x00   /* 芯片 ID, 读出 0x0043 */
#define BMI323_REG_ERR_REG     0x01   /* 错误状态 */
#define BMI323_REG_STATUS      0x02   /* 状态(数据就绪等) */
#define BMI323_REG_ACC_DATA_X  0x03   /* 加速度计数据, 0x03~0x05 (X/Y/Z) */
#define BMI323_REG_GYR_DATA_X  0x06   /* 陀螺仪数据, 0x06~0x08 (X/Y/Z) */
#define BMI323_REG_TEMPERATURE 0x09   /* 温度数据 */
#define BMI323_REG_ACC_CONF    0x20   /* 加速度计配置: BW/ODR/模式 */
#define BMI323_REG_ACC_RANGE   0x21   /* 加速度计量程 */
#define BMI323_REG_GYR_CONF    0x22   /* 陀螺仪配置: BW/ODR/模式 */
#define BMI323_REG_GYR_RANGE   0x23   /* 陀螺仪量程 */
#define BMI323_REG_CMD         0x7E   /* 命令寄存器 */

/* CMD 命令: 软复位 */
#define BMI323_CMD_SOFT_RESET  0xDE

/* ACC_CONF 字段: [13:10] BW | [9:6] ODR | [1:0] MODE */
#define BMI323_ACC_ODR_100HZ   (7 << 6)   /* ODR=100Hz */
#define BMI323_ACC_MODE_NORMAL 0x02       /* 正常工作模式 */

/* GYR_CONF 字段: [13:10] BW | [9:6] ODR | [1:0] MODE */
#define BMI323_GYR_ODR_100HZ   (7 << 6)   /* ODR=100Hz */
#define BMI323_GYR_MODE_NORMAL 0x02       /* 正常工作模式 */

/* 量程配置 */
#define BMI323_ACC_RANGE_8G    (2 << 12)  /* [13:12]: ±8g */
#define BMI323_GYR_RANGE_2000  (4 << 11)  /* [13:11]: ±2000dps */

/* 灵敏度换算系数 */
#define GRAVITY_MPS2           9.80665f
#define BMI323_ACC_LSB_PER_G   4096.0f    /* ±8g 档: 32768/8 */
#define BMI323_GYR_LSB_PER_DPS 16.384f    /* ±2000dps 档: 32768/2000 */
#define BMI323_TEMP_LSB_DEGC   (1.0f / 511.0f)  /* 温度 1/511 ℃/LSB */
#define BMI323_TEMP_OFFSET     23.0f      /* 温度偏移 */

#define BMI323_CHIP_ID_VALUE   0x43
#define BMI323_RESET_DELAY_MS  20         /* 软复位等待(datasheet ≥1.8ms, 取保险值) */

/* 从字节指针解析小端 16 位有符号整数(寄存器数据为 LSB 在前) */
static int16_t le16(const uint8_t *p)
{
    return (int16_t)((p[1] << 8) | p[0]);
}

/* 候选地址: 本模块固定为 0x4A (见 config.h) */
static const uint8_t s_bmi323_addrs[] = {
    SENSOR_IMU_6AXIS_ADDR,
};

/**
 * 写 BMI323 一个 16 位寄存器: [寄存器地址][LSB][MSB]
 */
static esp_err_t bmi323_write_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * 读 BMI323 一个 16 位寄存器, 返回 LSB 在 buf[0]。
 */
static esp_err_t bmi323_read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf)
{
    return i2c_bus_read_regs(dev, reg, buf, 2);
}

/**
 * 探测识别芯片并完成初始化。
 * 流程遵循 datasheet 启动序列:
 *   软复位 -> 延时 -> dummy read 激活 I2C -> 校验 CHIP_ID -> 配置传感器
 */
static esp_err_t imu_probe(uint8_t addr, char *info, size_t info_len)
{
    i2c_master_dev_handle_t dev = i2c_bus_dev_get(addr);
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint8_t id[2] = { 0 };

    /* 1. 软复位: 处理未知上电状态, 之后 I2C 需 dummy read 才能正常访问 */
    bmi323_write_reg16(dev, BMI323_REG_CMD, BMI323_CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(BMI323_RESET_DELAY_MS));

    /* 2. dummy read 激活 I2C 接口(datasheet 要求) */
    uint8_t dummy = 0;
    (void)i2c_bus_read_regs(dev, BMI323_REG_CHIP_ID, &dummy, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* 3. 校验 CHIP_ID (0x0043, 低字节 0x43) */
    if (bmi323_read_reg16(dev, BMI323_REG_CHIP_ID, id) != ESP_OK || id[0] != BMI323_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "0x%02X: CHIP_ID 校验失败 (读得 0x%02X%02X, 期望 0x0043)",
                 addr, id[1], id[0]);
        return ESP_ERR_NOT_FOUND;
    }

    /* 4. 配置加速度计: 100Hz + 正常模式, 量程 ±8g */
    esp_err_t err = bmi323_write_reg16(dev, BMI323_REG_ACC_CONF,
                                       BMI323_ACC_ODR_100HZ | BMI323_ACC_MODE_NORMAL);
    if (err == ESP_OK) {
        err = bmi323_write_reg16(dev, BMI323_REG_ACC_RANGE, BMI323_ACC_RANGE_8G);
    }
    /* 5. 配置陀螺仪: 100Hz + 正常模式, 量程 ±2000dps */
    if (err == ESP_OK) {
        err = bmi323_write_reg16(dev, BMI323_REG_GYR_CONF,
                                 BMI323_GYR_ODR_100HZ | BMI323_GYR_MODE_NORMAL);
    }
    if (err == ESP_OK) {
        err = bmi323_write_reg16(dev, BMI323_REG_GYR_RANGE, BMI323_GYR_RANGE_2000);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: 传感器配置失败 (err=0x%x)", addr, err);
        return err;
    }

    snprintf(info, info_len, "BMI323 (CHIP_ID=0x%02X)", id[0]);
    ESP_LOGI(TAG, "probe 0x%02X: BMI323 初始化完成 (ACC ±8g 100Hz, GYR ±2000dps 100Hz)",
             addr);
    return ESP_OK;
}

/**
 * 读取一帧 6 轴数据 + 温度:
 *   0x03 起连续 12 字节覆盖 ACC X/Y/Z 与 GYR X/Y/Z 六个 16 位寄存器,
 *   再单独读温度寄存器(0x09), 并按量程换算为物理单位。
 */
static esp_err_t imu_read(uint8_t addr, sensor_data_t *out)
{
    i2c_master_dev_handle_t dev = i2c_bus_dev_get(addr);
    if (dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(out, 0, sizeof(*out));

    /* 加速度 + 角速度: 6 个 16 位寄存器连读 */
    uint8_t raw[12];
    esp_err_t err = i2c_bus_read_regs(dev, BMI323_REG_ACC_DATA_X, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    out->has_acc = true;
    out->acc_mps2[0] = le16(raw + 0) / BMI323_ACC_LSB_PER_G * GRAVITY_MPS2;
    out->acc_mps2[1] = le16(raw + 2) / BMI323_ACC_LSB_PER_G * GRAVITY_MPS2;
    out->acc_mps2[2] = le16(raw + 4) / BMI323_ACC_LSB_PER_G * GRAVITY_MPS2;
    out->has_gyr = true;
    out->gyr_dps[0] = le16(raw + 6) / BMI323_GYR_LSB_PER_DPS;
    out->gyr_dps[1] = le16(raw + 8) / BMI323_GYR_LSB_PER_DPS;
    out->gyr_dps[2] = le16(raw + 10) / BMI323_GYR_LSB_PER_DPS;

    /* 温度: raw/511 + 23 */
    uint8_t traw[2];
    if (bmi323_read_reg16(dev, BMI323_REG_TEMPERATURE, traw) == ESP_OK) {
        out->has_temp  = true;
        out->temp_degc = le16(traw) * BMI323_TEMP_LSB_DEGC + BMI323_TEMP_OFFSET;
    }

    /* 串口打印采集数据(调试用) */
    ESP_LOGI(TAG, "0x%02X ACC[m/s^2] X=%.3f Y=%.3f Z=%.3f | "
                  "GYR[dps] X=%.3f Y=%.3f Z=%.3f | TEMP=%.2f℃",
             addr,
             out->acc_mps2[0], out->acc_mps2[1], out->acc_mps2[2],
             out->gyr_dps[0], out->gyr_dps[1], out->gyr_dps[2],
             out->temp_degc);

    return ESP_OK;
}

/* 导出驱动实例, 供 sensor_manager 的注册表引用 */
const sensor_driver_t imu_6axis_driver = {
    .name       = "BMI323 6-axis IMU",
    .addr_list  = s_bmi323_addrs,
    .addr_count = sizeof(s_bmi323_addrs) / sizeof(s_bmi323_addrs[0]),
    .probe      = imu_probe,
    .read       = imu_read,
};