/*
 * ESP32-C5 传感器数据 WiFi 链路 (STA + UDP)
 *
 * C5 以普通 WiFi 节点(STA)连接 C6/P4 广播的热点, 然后通过 UDP 把传感器
 * 数据(JSON 帧)发送到 C6/P4 的固定端口。ISR/轮询采集逻辑不受影响。
 *
 * 关键点:
 *   1. STA 模式连接 config.h 中 WIFI_LINK_SSID/PASS 指定的热点;
 *   2. 连接成功后创建 UDP socket, 每次 send_data 向 WIFI_LINK_TARGET_IP:PORT 发送;
 *   3. 数据帧由 sensor_build_tx_frame 生成(含 node_id + IMU 原始数据 + 压力占位)。
 */
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "config.h"
#include "wifi_link.h"

static const char *TAG = "wifi_link";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_CONNECT_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group = NULL;

static bool s_udp_ready = false;   /* 连接成功可发送 */
static int  s_sock = -1;           /* UDP socket */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA 启动, 尝试连接热点...");
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* STA 连接失败(如找不到 AP)也会触发 DISCONNECTED, 需按 reason 区分,
         * 避免把"从未连上"误报成"断开重连"。 */
        wifi_event_sta_disconnected_t *d =
            (wifi_event_sta_disconnected_t *)event_data;
        uint8_t reason = (d != NULL) ? d->reason : 0;
        if (reason == WIFI_REASON_NO_AP_FOUND) {
            ESP_LOGW(TAG, "未找到热点 SSID=%s (主设备 AP 未开启?), 继续重试...",
                     WIFI_LINK_SSID);
        } else {
            ESP_LOGW(TAG, "与热点断开 (reason=%d), 尝试重连...", reason);
        }
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_FAIL_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "已获取 IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_link_init(void)
{
#if !WIFI_LINK_ENABLED
    ESP_LOGI(TAG, "WiFi 发送未启用 (WIFI_LINK_ENABLED=0)");
    return ESP_OK;
#endif

    /* 1. NVS(WiFi 子系统需要) */
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    /* 2. 网络栈 + 事件循环 + 事件组 */
    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    assert(sta);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* 3. WiFi STA 启动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wcfg = { 0 };
    strlcpy((char *)wcfg.sta.ssid, WIFI_LINK_SSID, sizeof(wcfg.sta.ssid));
    strlcpy((char *)wcfg.sta.password, WIFI_LINK_PASS, sizeof(wcfg.sta.password));
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 4. 等待连接成功(或超时失败) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_CONNECT_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi 连接超时/失败, 仍在后台重试. "
                      "检查热点 SSID=%s", WIFI_LINK_SSID);
    }
    ESP_LOGI(TAG, "WiFi 已启动, 目标 %s:%d, node_id=%d",
             WIFI_LINK_TARGET_IP, WIFI_LINK_TARGET_PORT, WIFI_LINK_NODE_ID);
    return ESP_OK;
}

esp_err_t wifi_link_send_data(const char *payload)
{
#if !WIFI_LINK_ENABLED
    (void)payload;
    return ESP_OK;
#endif
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t len = strlen(payload);

    /* UDP 数据报无需连接, 首次使用前懒创建 socket */
    if (s_sock < 0) {
        s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_sock < 0) {
            ESP_LOGE(TAG, "UDP socket 创建失败, errno %d", errno);
            return ESP_FAIL;
        }
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(WIFI_LINK_TARGET_PORT);
    dest.sin_addr.s_addr = inet_addr(WIFI_LINK_TARGET_IP);

    int n = sendto(s_sock, payload, len, 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    if (n < 0) {
        // ESP_LOGW(TAG, "UDP 发送失败, errno %d (是否已连上热点?)", errno);
        return ESP_FAIL;
    }
    s_udp_ready = true;
    return ESP_OK;
}