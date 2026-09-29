#include "esp_log.h"
#include "config.h"
#include "wifi_link.h"

static const char *TAG = "wifi_link";

/*
 * TODO(WiFi 传输预留): ESP32-C5 -> ESP32-P4 数据通道, 可选方案:
 *   1. WiFi STA + TCP/UDP/MQTT: C5 与 P4 接入同一网络, P4 作服务端监听
 *      WIFI_LINK_P4_PORT 端口;
 *   2. ESP-NOW: 低时延点对点, C5 直接发往 P4 的 MAC 地址;
 *   3. WiFi + ESP-NOW 混合。
 * 启用时将 config.h 中 WIFI_LINK_ENABLED 置 1 并补充实现。
 * 数据格式沿用 sensor_manager 输出的 JSON 字符串。
 */

/**
 * 初始化 WiFi 数据链路(预留)。
 * 待实现: 连接 WiFi 或初始化 ESP-NOW, 建立与 ESP32-P4 主控的连接。
 */
esp_err_t wifi_link_init(void)
{
    ESP_LOGI(TAG, "WiFi 链路为预留接口, 暂未启用 (WIFI_LINK_ENABLED=%d)", WIFI_LINK_ENABLED);
    return ESP_OK;
}

/**
 * 发送一帧采集数据(JSON 字符串)给 ESP32-P4 主控(预留)。
 * @param payload sensor_manager 输出的 JSON 数据帧
 */
esp_err_t wifi_link_send_data(const char *payload)
{
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
#if WIFI_LINK_ENABLED
    /* TODO: 通过已建立的 WiFi/ESP-NOW 通道将 payload 发送到 ESP32-P4 */
    return ESP_OK;
#else
    return ESP_OK; /* 预留: 当前不做实际发送 */
#endif
}
