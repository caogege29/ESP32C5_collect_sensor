#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "config.h"
#include "i2c_bus.h"
#include "sensor_manager.h"

static const char *TAG = "sensor_mgr";

/*
 * 传感器驱动注册表:
 * 需要外挂更多传感器(温湿度/气压/光照等)时, 实现对应的 sensor_driver_t
 * 并追加到本表即可, 框架会自动扫描地址并识别。
 */
static const sensor_driver_t *const s_sensor_registry[] = {
    &imu_6axis_driver,
};

/* 在线传感器实例: 已识别传感器 = 驱动 + 实际地址 + 识别信息 */
typedef struct {
    const sensor_driver_t *drv;   /* 所属驱动(提供 read 等接口) */
    uint8_t                addr;  /* 绑定的 I2C 地址 */
    char                   info[40]; /* 芯片识别信息(型号/ID) */
} sensor_instance_t;

static sensor_instance_t s_inst[SENSOR_MANAGER_MAX_INSTANCES]; /* 实例数组 */
static int               s_inst_count = 0;                     /* 已识别实例数 */

/**
 * 初始化传感器管理器:
 *   1) 扫描 I2C 总线, 得到所有在线设备地址;
 *   2) 遍历驱动注册表, 地址在线则调用 probe 探测识别;
 *   3) 报告未识别设备, 便于后续扩展驱动。
 */
esp_err_t sensor_manager_init(void)
{
    /* 1. 扫描总线上的在线设备 */
    uint8_t online[117]; /* 7bit 可寻址范围 0x03~0x77 */
    int n = i2c_bus_scan(online, (int)sizeof(online));
    if (n <= 0) {
        ESP_LOGW(TAG, "I2C 总线未发现任何设备, 请检查接线: SDA=GPIO%d SCL=GPIO%d, "
                      "模块通信模式需拨到 I2C", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "I2C 扫描到 %d 个设备:", n);
    ESP_LOG_BUFFER_HEX(TAG, online, n);

    /* 2. 遍历注册表, 按地址探测识别 */
    const size_t drv_num = sizeof(s_sensor_registry) / sizeof(s_sensor_registry[0]);
    for (size_t d = 0; d < drv_num && s_inst_count < SENSOR_MANAGER_MAX_INSTANCES; d++) {
        const sensor_driver_t *drv = s_sensor_registry[d];
        for (int j = 0; j < drv->addr_count; j++) {
            uint8_t addr = drv->addr_list[j];
            /* 在在线设备表中查找该地址 */
            int idx = -1;
            for (int i = 0; i < n; i++) {
                if (online[i] == addr) {
                    idx = i;
                    break;
                }
            }
            if (idx < 0) {
                continue; /* 该地址不在线, 尝试下一个候选地址 */
            }
            sensor_instance_t *inst = &s_inst[s_inst_count];
            inst->drv     = drv;
            inst->addr    = addr;
            inst->info[0] = '\0';
            /* 地址在线, 进一步探测芯片型号(写配置寄存器等) */
            if (drv->probe(addr, inst->info, sizeof(inst->info)) == ESP_OK) {
                ESP_LOGI(TAG, "识别到传感器 [%s] @ 0x%02X : %s",
                         drv->name, addr, inst->info);
                s_inst_count++;
                online[idx] = online[--n]; /* 已识别, 从在线表移出, 避免重复匹配 */
                break;                     /* 同一驱动仅绑定第一个命中的地址 */
            }
        }
    }

    /* 3. 报告未识别设备, 提示可扩展驱动 */
    for (int i = 0; i < n; i++) {
        ESP_LOGW(TAG, "在线设备 0x%02X 未能识别 (无对应驱动), "
                      "可在 s_sensor_registry 注册表扩展", online[i]);
    }
    if (s_inst_count == 0) {
        ESP_LOGW(TAG, "无传感器被识别, 按图检查模块地址拨码: 0x%02X (A0=0, A1=0)",
                 SENSOR_IMU_6AXIS_ADDR);
    }
    return ESP_OK;
}

/**
 * 采集所有在线传感器各一帧数据。
 * @param out  输出数组, 每个元素为一帧采集结果(含来源名+地址)
 * @param max  输出数组容量
 * @return 实际采集成功(读取无错误)的数量
 */
int sensor_manager_collect(sensor_sample_t *out, int max)
{
    int n = 0;
    for (int i = 0; i < s_inst_count && n < max; i++) {
        out[n].name = s_inst[i].drv->name;
        out[n].addr = s_inst[i].addr;
        if (s_inst[i].drv->read(s_inst[i].addr, &out[n].data) == ESP_OK) {
            n++; /* 读取成功才计入 */
        }
    }
    return n;
}

/**
 * 将一帧采集结果序列化为 JSON 字符串。
 * 格式示例:
 *   {"sensor":"6-axis IMU","addr":"0x4A","acc":[...],"gyr":[...],"temp":25.3}
 * 仅输出驱动实际提供的字段(通过 has_xxx 标志判断)。
 * @return 写入的字节数(不含结尾 '\0'); 参数非法时返回 0。
 */
size_t sensor_sample_to_json(const sensor_sample_t *s, char *buf, size_t len)
{
    if (s == NULL || buf == NULL || len == 0) {
        return 0;
    }
    /* 安全追加宏: 仅当缓冲仍有空间时写入, 并累计偏移 off */
#define APPEND(...) do {                                                      \
        if (off < len - 1) {                                                  \
            int r = snprintf(buf + off, len - off, __VA_ARGS__);              \
            if (r > 0) off += (size_t)r;                                      \
        }                                                                     \
    } while (0)

    size_t off = 0;
    /* 头部固定字段: 传感器类型名 + I2C 地址 */
    APPEND("{\"sensor\":\"%s\",\"addr\":\"0x%02X\"", s->name, s->addr);
    const sensor_data_t *d = &s->data;
    /* 按驱动提供的数据类型逐字段追加 */
    if (d->has_acc)  APPEND(",\"acc\":[%.3f,%.3f,%.3f]",
                            d->acc_mps2[0], d->acc_mps2[1], d->acc_mps2[2]);
    if (d->has_gyr)  APPEND(",\"gyr\":[%.3f,%.3f,%.3f]",
                            d->gyr_dps[0], d->gyr_dps[1], d->gyr_dps[2]);
    if (d->has_mag)  APPEND(",\"mag\":[%.2f,%.2f,%.2f]",
                            d->mag_ut[0], d->mag_ut[1], d->mag_ut[2]);
    if (d->has_temp) APPEND(",\"temp\":%.2f", d->temp_degc);
    if (d->has_hum)  APPEND(",\"hum\":%.2f", d->hum_pct);
    if (d->has_press) APPEND(",\"press\":%.2f", d->press_hpa);
    /* 防止越界 */
    if (off > len - 1) {
        off = len - 1;
    }
    APPEND("}");
    return off;
#undef APPEND
}

/**
 * 构建用于 WiFi(UDP) 实时传输的发送帧(JSON 文本)。
 * 帧格式示例:
 *   {"node":0,"sensor":"SEN0692 6-axis IMU","acc":[-0.01,-0.06,-9.35],"gyr":[0.04,-0.02,0.02],"press":[7.35,11.77]}
 * - node  : 本块 C5 的节点 ID, 供 P4 区分『左/右』来源;
 * - acc   : 三轴加速度物理量(m/s^2, ±2G 量程);
 * - gyr   : 三轴角速度物理量(dps, ±250dps 量程);
 * - press : 2 路压力传感器读数(单位 g, 由 ADC+幂律标定换算)。
 */
size_t sensor_build_tx_frame(int node_id, const sensor_sample_t *s,
                             const float press[2], char *buf, size_t len)
{
    if (s == NULL || buf == NULL || len == 0) {
        return 0;
    }

    size_t off = 0;
#define APPEND(...) do {                                                      \
        if (off < len - 1) {                                                  \
            int r = snprintf(buf + off, len - off, __VA_ARGS__);              \
            if (r > 0) off += (size_t)r;                                      \
        }                                                                     \
    } while (0)

    const sensor_data_t *d = &s->data;

    APPEND("{\"node\":%d,\"sensor\":\"%s\",\"addr\":\"0x%02X\"",
           node_id, s->name, s->addr);

    /* IMU 物理量: 加速度 m/s^2(±2G 量程), 角速度 dps(±250dps 量程) */
    if (d->has_acc) {
        APPEND(",\"acc\":[%.3f,%.3f,%.3f]",
               d->acc_mps2[0], d->acc_mps2[1], d->acc_mps2[2]);
    }
    if (d->has_gyr) {
        APPEND(",\"gyr\":[%.3f,%.3f,%.3f]",
               d->gyr_dps[0], d->gyr_dps[1], d->gyr_dps[2]);
    }

    /* 2 路压力传感器读数(单位 g) */
    float p0 = (press != NULL) ? press[0] : 0.0f;
    float p1 = (press != NULL) ? press[1] : 0.0f;
    APPEND(",\"press\":[%.2f,%.2f]", p0, p1);

    if (off > len - 1) {
        off = len - 1;
    }
    APPEND("}");
    return off;
#undef APPEND
}
