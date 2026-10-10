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
#define I2C_MASTER_FREQ_HZ         100000   /* 总线频率: 100kHz (标准模式, 排查弱上拉时更稳) */
#define I2C_MASTER_TIMEOUT_MS      300      /* 单次传输超时 */
#define I2C_DEV_TABLE_MAX          16       /* 可挂载的最大 I2C 设备数 */

/* ------------------------ 外挂传感器 I2C 地址 ------------------------------ */
/* BMI323 6轴IMU: 本模块实际地址为 0x4A (由模块电路固定) */
#define SENSOR_IMU_6AXIS_ADDR      0x4A

/* ------------------------ BMI323 中断引脚（预留） --------------------------- */
#define SENSOR_IMU_INT1_IO         0        /* BMI323 INT1 -> GPIO0 (数据就绪等) */
#define SENSOR_IMU_INT2_IO         1        /* BMI323 INT2 -> GPIO1 (预留) */

/* ---------------------------- 采集任务 ------------------------------------ */
#define SENSOR_TASK_NAME           "sensor_task"
#define SENSOR_TASK_STACK_SIZE     8192
#define SENSOR_TASK_PRIO           5
#define SENSOR_SAMPLE_INTERVAL_MS  5        /* 采样周期(个位毫秒级) */
#define SENSOR_LOG_INTERVAL_FRAMES 200      /* 每 N 帧打印一次 JSON (5ms*200=1s, 不降发送速率) */
#define SENSOR_MANAGER_MAX_INSTANCES 8      /* 同时在线的最大传感器实例数 */

/* ----------------------- SEN0692 IMU 读时序延时 --------------------------- */
/* DFRobot I2C 读流程: 写寄存器地址后需等待 MCU 取数再读回。
 * 默认 10ms 为库保守值, 偏低主频的模块才需要; 实测稳定可下调到 2~3ms 提速。
 * 单位: 微秒(us)。 */
#define IMU_I2C_READ_DELAY_US       1000   /* 1ms: 提采样率; 若数据异常可调回 3000~10000 */

/* ------------------------ WiFi 数据链路 (STA + UDP) ------------------------- */
/* C5 以普通 WiFi 节点(STA)连接 C6/P4 广播的热点, 然后通过 UDP 把传感器数据
 * 发送到 C6/P4 的固定端口。
 * 启用: WIFI_LINK_ENABLED=1。 */
#define WIFI_LINK_ENABLED           1        /* 1: 启用 WiFi 发送 */
/* C5 节点 ID: 由于会『左』『右』各挂一块 C5 同时向 P4 传输,
 * 每块写入不同 ID 以便 P4 区分数据来源(如 左=0, 右=1)。 */
#define WIFI_LINK_NODE_ID           0        /* 本块 C5 的节点编号(左右各不相同) */
/* 要连接的热点(C6/P4 广播) SSID / 密码 */
#define WIFI_LINK_SSID              "HipExo_C6"         /* P4 侧 AP 热点名 (WIFI_RX_AP_SSID) */
#define WIFI_LINK_PASS              "12345678"          /* 热点密码 (WIFI_RX_AP_PASS) */
/* C6/P4 接收端的 IP 与 UDP 端口 */
#define WIFI_LINK_TARGET_IP         "192.168.4.1"       /* P4 侧 AP 模式默认网关地址 */
#define WIFI_LINK_TARGET_PORT       5005                /* P4 侧 UDP 监听端口 (WIFI_RX_UDP_PORT) */

/* ------------------------ 压力传感器 (2 路 ADC) --------------------------- */
/* 2 路电阻式压力传感器(RP-C18.3, 压阻式, 压力↑→电阻↓), 通过 10kΩ 上拉分压:
 *   V = 3.3V * Rp / (10kΩ + Rp)   ->   Rp = 10k * V / (3.3 - V)
 * C18.3 的 R-F 特性为幂律(见 datasheet 标定曲线, 非反比):
 *   R(kΩ) = A * F(g)^(-B)   , A=153.18, B≈0.699
 *   => 力 F(g) = (A / R_kΩ)^(1/B)
 * 量程 0~6000g(≈0~6kg)。
 * 接线: 传感器1 -> GPIO5(ADC1_CH4) = FORCE_A_3; 传感器2 -> GPIO6(ADC1_CH5) = FORCE_A_2。 */
#define PRESSURE_1_GPIO             5                 /* 压力传感器 1 -> GPIO5 (ADC1_CH4) */
#define PRESSURE_2_GPIO             6                 /* 压力传感器 2 -> GPIO6 (ADC1_CH5) */
#define PRESSURE_ADC_VREF_MV        3300.0f           /* ADC 满量程基准(mV), 可微调校准 */
#define PRESSURE_VCC_MV             3300.0f           /* 分压网络供电 3.3V */
#define PRESSURE_R_REF_OHM          10000.0f          /* 上拉分压电阻 10kΩ */
#define PRESSURE_FORCE_A            153.18f           /* 幂律系数 A: R(kΩ)=A*F(g)^-B */
#define PRESSURE_FORCE_B            0.699f            /* 幂律指数 B (负相关, 标定曲线≈0.6~0.7) */
#define PRESSURE_FORCE_FULL_G       6000.0f           /* 满量程 6000g (用于钳位) */
