#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 WiFi 数据链路: 以 STA 连接热点并通过 UDP 发送数据到 C6/P4 */
esp_err_t wifi_link_init(void);

/**
 * 发送一帧采集数据(JSON 文本)给 C6/P4 接收端(UDP)。
 * payload 格式: {"node":0,"sensor":"6-axis IMU","addr":"0x4A","acc_raw":[...],"gyr_raw":[...],"press":...}
 * 注意: 需先成功连接热点; 连接建立前该调用会尝试创建 socket 并发送(可能失败)。
 */
esp_err_t wifi_link_send_data(const char *payload);

#ifdef __cplusplus
}
#endif
