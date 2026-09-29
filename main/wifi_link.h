#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 WiFi 数据链路(预留) */
esp_err_t wifi_link_init(void);

/**
 * 发送一帧采集数据(JSON 字符串)给 ESP32-P4 主控(预留)。
 * payload 格式: {"sensor":"6-axis IMU","addr":"0x4A","acc":[...],"gyr":[...]}
 */
esp_err_t wifi_link_send_data(const char *payload);

#ifdef __cplusplus
}
#endif
