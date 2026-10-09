#include <math.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "config.h"
#include "pressure_sensor.h"

static const char *TAG = "pressure";

/*
 * GPIO -> ADC 通道映射(ESP32-C5):
 *   GPIO4 = ADC1_CH3
 *   GPIO6 = ADC1_CH5
 */
#define PRESS_CH0        ADC_CHANNEL_3   /* 压力1 -> GPIO4 */
#define PRESS_CH1        ADC_CHANNEL_5   /* 压力2 -> GPIO6 */

static adc_oneshot_unit_handle_t s_adc1 = NULL;

esp_err_t pressure_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&ucfg, &s_adc1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit 失败: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t cc = {
        .atten    = ADC_ATTEN_DB_12,   /* 覆盖 ~0~3.3V, 匹配分压网络满摆幅 */
        .bitwidth = ADC_BITWIDTH_12,   /* 12bit, 0~4095 */
    };
    err = adc_oneshot_config_channel(s_adc1, PRESS_CH0, &cc);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_oneshot_config_channel(s_adc1, PRESS_CH1, &cc);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "压力 ADC 初始化完成 (GPIO%d=CH3, GPIO%d=CH5, 量程 0~%.0fg)",
             PRESSURE_1_GPIO, PRESSURE_2_GPIO, PRESSURE_FORCE_FULL_G);
    return ESP_OK;
}

/*
 * raw -> 电压 -> 传感器电阻 Rp -> 力 F(g)
 *   电压   V = Vcc * Rp / (R_ref + Rp)
 *   反推   Rp = R_ref * V / (Vcc - V)
 *   幂律   R(kΩ) = A * F(g)^(-B)   =>   F(g) = (A / R_kΩ)^(1/B)
 *   (C18.3 压阻式, R-F 为幂律关系, 非反比)
 */
static float raw_to_force_g(int raw)
{
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 4095) {
        raw = 4095;
    }

    float v_mv = PRESSURE_ADC_VREF_MV * (float)raw / 4095.0f;

    /* 电压接近 VCC(无压)时力趋于 0; 分母加下限防除零 */
    float denom = PRESSURE_VCC_MV - v_mv;
    if (denom < 1.0f) {
        denom = 1.0f;
    }
    float rp_ohm = PRESSURE_R_REF_OHM * v_mv / denom;
    if (rp_ohm < 1.0f) {
        rp_ohm = 1.0f;
    }

    /* 电阻转 kΩ, 代入幂律反解求力 */
    float r_kohm = rp_ohm / 1000.0f;
    float ratio = PRESSURE_FORCE_A / r_kohm;
    if (ratio <= 0.0f) {
        ratio = 1e-6f;
    }
    float f_g = powf(ratio, 1.0f / PRESSURE_FORCE_B);

    if (f_g < 0.0f) {
        f_g = 0.0f;
    }
    if (f_g > PRESSURE_FORCE_FULL_G) {
        f_g = PRESSURE_FORCE_FULL_G;
    }
    return f_g;
}

esp_err_t pressure_read(float press_g[2])
{
    if (press_g == NULL || s_adc1 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw0 = 0, raw1 = 0;
    esp_err_t err = adc_oneshot_read(s_adc1, PRESS_CH0, &raw0);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_oneshot_read(s_adc1, PRESS_CH1, &raw1);
    if (err != ESP_OK) {
        return err;
    }

    press_g[0] = raw_to_force_g(raw0);
    press_g[1] = raw_to_force_g(raw1);
    return ESP_OK;
}