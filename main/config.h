#pragma once

/* ============================================================================
 * ESP32-C5 传感器采集节点 —— 全局配置（引脚 / 总线 / 地址 / 任务 / WiFi）
 * 接线参考（BMI323 高精度6轴IMU, 模块座 HC-PM254-5.0H-1x6PZ）:
 *   座子1(VCC5V) -> 5V 供电
 *   座子2(GND)   -> GND
 *   座子3(SCL)   -> ESP32-C5 GPIO3
 *   座子4(SDA)   -> ESP32-C5 GPIO2
 *   座子5(INT1') -> ESP32-C5 GPIO0 (中断, 预留)
 *   座子6(INT2') -> ESP32-C5 GPIO1 (中断, 预留)
 * ==========================================================================*/

/* ---------------------------- I2C 总线 ------------------------------------ */
#define I2C_MASTER_SDA_IO          2        /* I2C SDA 引脚 (与芯片 LP_I2C_SDA 默认一致) */
#define I2C_MASTER_SCL_IO          3        /* I2C SCL 引脚 (与芯片 LP_I2C_SCL 默认一致) */
#define I2C_MASTER_PORT            0        /* I2C 控制器编号 */
#define I2C_MASTER_FREQ_HZ         400000   /* 总线频率: 400kHz */
#define I2C_MASTER_TIMEOUT_MS      100      /* 单次传输超时 */
#define I2C_DEV_TABLE_MAX          16       /* 可挂载的最大 I2C 设备数 */

/* ------------------------ 外挂传感器 I2C 地址 ------------------------------ */
/* BMI323 6轴IMU: 本模块实际地址为 0x4A (由模块电路固定) */
#define SENSOR_IMU_6AXIS_ADDR      0x4A

/* ------------------------ BMI323 中断引脚（预留） --------------------------- */
#define SENSOR_IMU_INT1_IO         0        /* BMI323 INT1 -> GPIO0 (数据就绪等) */
#define SENSOR_IMU_INT2_IO         1        /* BMI323 INT2 -> GPIO1 (预留) */

/* ---------------------------- 采集任务 ------------------------------------ */
#define SENSOR_TASK_NAME           "sensor_task"
#define SENSOR_TASK_STACK_SIZE     4096
#define SENSOR_TASK_PRIO           5
#define SENSOR_SAMPLE_INTERVAL_MS  500      /* 采样周期 */
#define SENSOR_MANAGER_MAX_INSTANCES 8      /* 同时在线的最大传感器实例数 */

/* ------------------------ WiFi 传输（预留） -------------------------------- */
/* 后续将采集数据通过 WiFi 发送给 ESP32-P4 主控，接口见 wifi_link.h */
#define WIFI_LINK_ENABLED          0        /* 1: 启用(待实现), 0: 预留 */
#define WIFI_LINK_SSID             "your-ssid"
#define WIFI_LINK_PASS             "your-password"
#define WIFI_LINK_P4_PORT          8080     /* P4 侧 TCP 服务端口(预留) */
