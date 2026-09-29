/*
 * ============================================================================
 * ESP32-C5 传感器采集节点 —— 程序入口
 * 工作流程:
 *   I2C 外挂多传感器采集  ->  (预留)WiFi 传输给 ESP32-P4 主控
 * 各子模块:
 *   i2c_bus        I2C 主机总线初始化与读写封装
 *   sensor_manager 传感器驱动注册表 + 按地址自动识别 + 统一采集
 *   wifi_link      预留的 WiFi 数据链路接口(发送给 ESP32-P4)
 * 所有引脚/参数均见 config.h。
 * ============================================================================
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "i2c_bus.h"
#include "sensor_manager.h"
#include "wifi_link.h"

static const char *TAG = "main";

/**
 * 传感器采集任务:
 *   周期采集所有在线传感器 -> 序列化为 JSON -> 打印并(预留)发送到 P4 主控。
 */
static void sensor_task(void *arg)
{
    sensor_sample_t samples[SENSOR_MANAGER_MAX_INSTANCES]; /* 存放一次采集结果 */
    char json[256];                                        /* 单帧 JSON 序列化缓冲 */

    while (1) {
        /* 采集所有在线传感器各一帧, 返回实际采集到的数量 */
        int n = sensor_manager_collect(samples, SENSOR_MANAGER_MAX_INSTANCES);
        for (int i = 0; i < n; i++) {
            /* 将采集结果序列化为 JSON(含传感器名+地址, 便于 P4 区分来源) */
            // if (sensor_sample_to_json(&samples[i], json, sizeof(json)) > 0) {
            //     ESP_LOGI(TAG, "%s", json);
            //     wifi_link_send_data(json); /* 预留: 发送到 ESP32-P4 主控 */
            // }
        }
        /* 按 config.h 中配置的采样周期延时 */
        vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
    }
}

/**
 * 程序入口: 依次初始化 I2C 总线 -> 传感器管理 -> WiFi 链路, 再启动采集任务。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C5 传感器采集节点启动 (SDA=GPIO%d, SCL=GPIO%d)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    /* 初始化 I2C 主机总线(引脚/频率见 config.h) */
    ESP_ERROR_CHECK(i2c_bus_init());
    /* 扫描总线并按地址识别外挂传感器 */
    ESP_ERROR_CHECK(sensor_manager_init());
    /* 初始化预留的 WiFi 数据链路 */
    ESP_ERROR_CHECK(wifi_link_init());

    /* 创建传感器采集任务(参数见 config.h) */
    xTaskCreate(sensor_task, SENSOR_TASK_NAME, SENSOR_TASK_STACK_SIZE,
                NULL, SENSOR_TASK_PRIO, NULL);
}
