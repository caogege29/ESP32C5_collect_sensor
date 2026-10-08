/*
 * SEN0692 Gravity 6 轴惯性运动传感器 驱动适配层 (DFRobot Multi-DOF IMU I2C 协议)
 *
 * 原理图: 模块座 (HC-PM254-5.0H-1x6PZ), 引脚见 config.h:
 *   SCL=GPIO3, SDA=GPIO2, INT1=GPIO0, INT2=GPIO1
 *
 * 重要说明(排除数据异常的根因):
 *   该模块(DFRobot SEN0692)板载一颗协议转换 MCU, I2C 地址 0x4A 暴露的是
 *   DFRobot Modbus 简化的寄存器协议, 并【不是】博世 BMI323 原始寄存器。
 *   因此不能使用 Bosch bmi323 裸驱动 —— 直接读 BMI323 寄存器会得到
 *   CHIP_ID 不符(bmi323_init 返回 -3)以及冻结/错位数据。
 *   本文件按 DFRobot_Multi_DOF_IMU 库的 I2C 协议实现:
 *     - 寄存器地址为 16bit, 小端(先低字节后高字节);
 *     - 写: 发 [reg_lo][reg_hi][data...]
 *     - 读: 发 [reg_lo][reg_hi], 等待约 10ms, 再读 len 字节
 *
 * 寄存器(来自库 DFRobot_Multi_DOF_IMU.h, I2C 模式):
 *   0x0001 PID(产品识别码, 仅读)
 *   0x0005 工作模式(写)  normal=0x02
 *   0x0006 加速度量程(写) ±2G=0x00
 *   0x0007 陀螺仪量程(写) ±250dps=0x01
 *   0x0010 ACC/gyr 数据(读 12 字节): accX/Y/Z + gyrX/Y/Z, 各 2 字节小端 int16
 *
 * 物理换算(参考库 lsbToG / lsbToDps):
 *   物理量 = raw * 量程 / 32768   加速度(g→m/s^2 再乘 9.80665)
 */
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "config.h"
#include "i2c_bus.h"
#include "sensor_driver.h"

static const char *TAG = "df_imu6";

/* ---------------- DFRobot I2C 寄存器地址(16bit) ---------------- */
#define REG_PID                   0x0001 /* 产品识别码(只读)           */
#define REG_SENSORS_MODE          0x0005 /* 工作模式(读写)             */
#define REG_ACC_RANGE_CONF        0x0006 /* 加速度量程(读写)           */
#define REG_GYR_RANGE_CONF        0x0007 /* 陀螺仪量程(读写)           */
#define REG_ACC_DATA_X            0x0010 /* 6 轴数据起始(只读, 12B)    */

/* 数值定义(来自库枚举, I2C 模式) */
#define SEN_MODEL_6DOF            0x0006 /* eSensorModel6DOF           */
#define MODE_NORMAL               0x02   /* eNormalMode                */
#define ACC_RANGE_2G              0x00   /* eAccelRange2G              */
#define GYR_RANGE_250DPS          0x01   /* eGyroRange250DPS           */
#define ACC_RANGE_FLOAT           2.0f   /* 与 ACC_RANGE_2G 对应       */
#define GYR_RANGE_FLOAT           250.0f /* 与 GYR_RANGE_250DPS 对应   */

static uint8_t s_dev_addr = 0;   /* 从机地址                       */
static bool    s_inited   = false;

/* ------------------------- 底层 I2C 读/写 ------------------------------- */

/**
 * 按 DFRobot 协议写: [reg_lo][reg_hi][data...]
 */
static esp_err_t df_i2c_write_reg16(uint16_t reg, const uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = i2c_bus_dev_get(s_dev_addr);
    if (dev == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (len + 2 > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[64];
    buf[0] = (uint8_t)(reg & 0xFF);
    buf[1] = (uint8_t)((reg >> 8) & 0xFF);
    memcpy(buf + 2, data, len);
    return i2c_master_transmit(dev, buf, len + 2, I2C_MASTER_TIMEOUT_MS);
}

/**
 * 按 DFRobot 协议读: 发 [reg_lo][reg_hi], 等约 10ms(让 MCU 有时间取数), 再读 len 字节。
 */
static esp_err_t df_i2c_read_reg16(uint16_t reg, uint8_t *buf, size_t len)
{
    i2c_master_dev_handle_t dev = i2c_bus_dev_get(s_dev_addr);
    if (dev == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t addr_buf[2] = {
        (uint8_t)(reg & 0xFF),
        (uint8_t)((reg >> 8) & 0xFF),
    };
    esp_err_t err = i2c_master_transmit(dev, addr_buf, sizeof(addr_buf), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    esp_rom_delay_us(IMU_I2C_READ_DELAY_US); /* 延时由 config.h 配置(默认库值 10ms, 实测可调小提速) */
    return i2c_master_receive(dev, buf, len, I2C_MASTER_TIMEOUT_MS);
}

/* ------------------------- 驱动接口 --------------------------------------- */

/**
 * 探测识别并初始化模块。
 * 返回 ESP_OK 表示识别为 SEN0692(6DOF) 且配置成功。
 */
static esp_err_t imu_probe(uint8_t addr, char *info, size_t info_len)
{
    uint8_t raw[2] = { 0, 0 };

    s_dev_addr = addr;

    /* 读取 PID 判断是否为 6DOF IMU(PID >= eSensorModel6DOF) */
    if (df_i2c_read_reg16(REG_PID, raw, 2) != ESP_OK) {
        ESP_LOGE(TAG, "probe 0x%02X: 读取 PID 失败", addr);
        return ESP_ERR_NOT_FOUND;
    }
    uint16_t pid = (uint16_t)(raw[0] | (raw[1] << 8));
    if (pid < SEN_MODEL_6DOF) {
        ESP_LOGE(TAG, "probe 0x%02X: PID=0x%04X 非 6DOF IMU", addr, pid);
        return ESP_ERR_NOT_FOUND;
    }

    /* 切换到正常模式 */
    uint16_t mode = MODE_NORMAL;
    if (df_i2c_write_reg16(REG_SENSORS_MODE, (const uint8_t *)&mode, 2) != ESP_OK) {
        ESP_LOGE(TAG, "probe 0x%02X: 设置工作模式失败", addr);
        return ESP_FAIL;
    }

    /* 加速度量程 ±2G */
    uint16_t acc = ACC_RANGE_2G;
    if (df_i2c_write_reg16(REG_ACC_RANGE_CONF, (const uint8_t *)&acc, 2) != ESP_OK) {
        ESP_LOGE(TAG, "probe 0x%02X: 设置加速度量程失败", addr);
        return ESP_FAIL;
    }

    /* 陀螺仪量程 ±250dps */
    uint16_t gyr = GYR_RANGE_250DPS;
    if (df_i2c_write_reg16(REG_GYR_RANGE_CONF, (const uint8_t *)&gyr, 2) != ESP_OK) {
        ESP_LOGE(TAG, "probe 0x%02X: 设置陀螺仪量程失败", addr);
        return ESP_FAIL;
    }

    s_inited = true;
    snprintf(info, info_len, "SEN0692 6DOF IMU (PID=0x%04X)", pid);
    ESP_LOGI(TAG, "probe 0x%02X: SEN0692 初始化完成 (PID=0x%04X, ACC ±2g, GYR ±250dps)", addr, pid);
    return ESP_OK;
}

/**
 * 读取一帧 6 轴数据。
 */
static esp_err_t imu_read(uint8_t addr, sensor_data_t *out)
{
    if (!s_inited || addr != s_dev_addr) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(out, 0, sizeof(*out));

    /* 从 REG_ACC_DATA_X 连续读 12 字节(6 个 int16, 小端) */
    uint8_t raw[12];
    if (df_i2c_read_reg16(REG_ACC_DATA_X, raw, sizeof(raw)) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: 读取 6 轴数据失败", addr);
        return ESP_FAIL;
    }

    int16_t av[3], gv[3];
    av[0] = (int16_t)(raw[0]  | (raw[1]  << 8)); /* accX */
    av[1] = (int16_t)(raw[2]  | (raw[3]  << 8)); /* accY */
    av[2] = (int16_t)(raw[4]  | (raw[5]  << 8)); /* accZ */
    gv[0] = (int16_t)(raw[6]  | (raw[7]  << 8)); /* gyrX */
    gv[1] = (int16_t)(raw[8]  | (raw[9]  << 8)); /* gyrY */
    gv[2] = (int16_t)(raw[10] | (raw[11] << 8)); /* gyrZ */

    /* 原始 int16 数据(供 ESP-NOW 传输 / 上位机自行换算) */
    out->has_acc_raw = true;
    out->acc_raw[0] = av[0];
    out->acc_raw[1] = av[1];
    out->acc_raw[2] = av[2];
    out->has_gyr_raw = true;
    out->gyr_raw[0] = gv[0];
    out->gyr_raw[1] = gv[1];
    out->gyr_raw[2] = gv[2];

    /* 加速度: raw -> g -> m/s^2 */
    out->has_acc = true;
    for (int i = 0; i < 3; i++) {
        out->acc_mps2[i] = (float)av[i] * ACC_RANGE_FLOAT / 32768.0f * 9.80665f;
    }

    /* 角速度: raw -> dps */
    out->has_gyr = true;
    for (int i = 0; i < 3; i++) {
        out->gyr_dps[i] = (float)gv[i] * GYR_RANGE_FLOAT / 32768.0f;
    }

    /* 该协议未暴露温度, 置空 */
    out->has_temp = false;

    return ESP_OK;
}

/* 导出驱动实例, 供 sensor_manager 的注册表引用 */
const sensor_driver_t imu_6axis_driver = {
    .name       = "SEN0692 6-axis IMU",
    .addr_list  = &(uint8_t){ SENSOR_IMU_6AXIS_ADDR },
    .addr_count = 1,
    .probe      = imu_probe,
    .read       = imu_read,
};