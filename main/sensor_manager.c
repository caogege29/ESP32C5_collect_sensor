#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
 * 单次扫描 + 识别:
 *   1) 复位总线(释放可能被从机拉低的 SDA/SCL);
 *   2) 扫描 I2C 总线, 得到所有在线设备地址;
 *   3) 遍历驱动注册表, 地址在线则调用 probe 探测识别;
 *   4) 报告未识别设备, 便于后续扩展驱动。
 * @return 总线上应答的设备数量(用于判断是"总线无设备"还是"设备未识别")
 */
static int sensor_manager_scan(void)
{
    /* 复位总线: 覆盖上一次传输中断/从机挂死导致的整条总线假死 */
    i2c_bus_reset();

    /* 1. 扫描总线上的在线设备 */
    uint8_t online[117]; /* 7bit 可寻址范围 0x03~0x77 */
    int n = i2c_bus_scan(online, (int)sizeof(online));
    int total = n;       /* 记录总线应答总数, 供上层区分故障类型 */
    if (n <= 0) {
        return 0;
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
    return total;
}

/**
 * 初始化传感器管理器: 带延时重试, 覆盖模块上电时序抖动。
 */
esp_err_t sensor_manager_init(void)
{
    const int MAX_RETRY = 3;

    for (int attempt = 0; attempt < MAX_RETRY && s_inst_count == 0; attempt++) {
        if (attempt > 0) {
            /* 等待模块上电稳定后再重扫 */
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (sensor_manager_scan() == 0) {
            /* 总线上一个设备都没应答: 打印 SDA 空闲电平辅助判断硬件,
             * 正常空闲应为 1; 若为 0 说明被拉低(短路/未供电/接线错误) */
            ESP_LOGW(TAG, "I2C 总线未发现任何设备 (第 %d/%d 次): SDA=GPIO%d(空闲电平=%d), "
                          "请检查 SCL=GPIO%d 接线/模块供电/GND/通信模式需拨到 I2C",
                     attempt + 1, MAX_RETRY,
                     I2C_MASTER_SDA_IO, gpio_get_level(I2C_MASTER_SDA_IO),
                     I2C_MASTER_SCL_IO);
        }
    }

    if (s_inst_count == 0) {
        ESP_LOGW(TAG, "无传感器被识别 (已重试 %d 次), 按图检查模块地址拨码: 0x%02X (A0=0, A1=0)",
                 MAX_RETRY, SENSOR_IMU_6AXIS_ADDR);
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
    /* 自愈: 若开机未识别到任何传感器(偶发上电时序/接触导致总线无应答),
     * 运行中每约 1s 重扫一次总线, 识别成功后自动恢复采集, 无需重启设备。 */
    if (s_inst_count == 0) {
        static TickType_t last_rescan = 0;
        TickType_t now = xTaskGetTickCount();
        if (now - last_rescan >= pdMS_TO_TICKS(1000)) {
            last_rescan = now;
            sensor_manager_scan();
        }
    }

    int n = 0;
    for (int i = 0; i < s_inst_count && n < max; i++) {
        out[n].name = s_inst[i].drv->name;
        out[n].addr = s_inst[i].addr;
        esp_err_t err = s_inst[i].drv->read(s_inst[i].addr, &out[n].data);
        if (err == ESP_OK) {
            n++; /* 读取成功才计入 */
        } else {
            /* 读失败节流打印, 避免持续失败时刷屏(每 200 次≈1s 打一次) */
            static uint32_t fail_cnt = 0;
            if ((fail_cnt++ % 200) == 0) {
                ESP_LOGW(TAG, "读取 %s @ 0x%02X 失败: %s",
                         s_inst[i].drv->name, s_inst[i].addr, esp_err_to_name(err));
            }
        }
    }
    return n;
}

int sensor_manager_count(void)
{
    return s_inst_count;
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
 *   {"node":0,"acc":[-0.020,-0.061,-9.353],"gyr":[-0.038,0.023,0.015],"press":[7.35,11.77]}
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

    APPEND("{\"node\":%d", node_id);

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
