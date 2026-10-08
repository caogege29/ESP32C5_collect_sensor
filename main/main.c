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
    TickType_t last_wake = xTaskGetTickCount();            /* 绝对周期调度基准点 */
    uint32_t frame_cnt = 0;                                /* 帧计数(用于节流打印) */

    while (1) {
        /* 采集所有在线传感器各一帧, 返回实际采集到的数量 */
        int n = sensor_manager_collect(samples, SENSOR_MANAGER_MAX_INSTANCES);
        frame_cnt++;
        for (int i = 0; i < n; i++) {
            /* 构建含 node_id + IMU原始数据 + 压力占位的发送帧, 实时发给 P4 */
            if (sensor_build_tx_frame(WIFI_LINK_NODE_ID, &samples[i],
                                      NULL, json, sizeof(json)) > 0) {
                wifi_link_send_data(json);        /* 每帧都实时发送到 ESP32-P4 接收端 */
                /* 打印节流: 每 N 帧(5ms*200=1s)才打一次, 避免高速下 UART 拖慢发送 */
                if ((frame_cnt % SENSOR_LOG_INTERVAL_FRAMES) == 0) {
                    ESP_LOGI(TAG, "%s", json);
                }
                // ESP_LOGI(TAG, "-");
            }
        }
        /* 绝对周期调度: 保证帧间隔真正等于 SENSOR_SAMPLE_INTERVAL_MS */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
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
