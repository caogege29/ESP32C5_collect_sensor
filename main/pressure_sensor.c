#include <math.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "config.h"
#include "pressure_sensor.h"

static const char *TAG = "pressure";

/*
 * GPIO -> ADC 通道映射(ESP32-C5):
 *   GPIO5 = ADC1_CH4
 *   GPIO6 = ADC1_CH5
 */
#define PRESS_CH0        ADC_CHANNEL_4   /* 压力1 -> GPIO5 */
#define PRESS_CH1        ADC_CHANNEL_5   /* 压力2 -> GPIO6 */

static adc_oneshot_unit_handle_t s_adc1 = NULL;

/* 两路通道与各自的校准句柄。
 * 用 raw*3300/4095 的线性估算在 12dB 衰减下误差很大, 且各通道偏差不同,
 * 会导致"静止时两路算出的力不一样"。改用每通道独立的曲线拟合校准。 */
static const adc_channel_t s_ch[2] = { PRESS_CH0, PRESS_CH1 };
static adc_cali_handle_t   s_cali[2] = { NULL, NULL };

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

    for (int i = 0; i < 2; i++) {
        err = adc_oneshot_config_channel(s_adc1, s_ch[i], &cc);
        if (err != ESP_OK) {
            return err;
        }
        /* 每通道独立校准: C5 支持通道补偿, 消除两路 ADC 的系统偏差 */
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .chan     = s_ch[i],
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t cerr = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali[i]);
        if (cerr != ESP_OK) {
            ESP_LOGW(TAG, "ADC 通道 %d 校准初始化失败 (%s), 回退线性估算",
                     i, esp_err_to_name(cerr));
            s_cali[i] = NULL;
        }
    }

    ESP_LOGI(TAG, "压力 ADC 初始化完成 (GPIO%d=CH4, GPIO%d=CH5, 校准=%s, 量程 0~%.0fg)",
             PRESSURE_1_GPIO, PRESSURE_2_GPIO,
             (s_cali[0] != NULL && s_cali[1] != NULL) ? "曲线拟合" : "关闭",
             PRESSURE_FORCE_FULL_G);
    return ESP_OK;
}

/*
 * 电压(mV, 已校准) -> 传感器电阻 Rp -> 力 F(g)
 *   电压   V = Vcc * Rp / (R_ref + Rp)
 *   反推   Rp = R_ref * V / (Vcc - V)
 *   幂律   R(kΩ) = A * F(g)^(-B)   =>   F(g) = (A / R_kΩ)^(1/B)
 *   (C18.3 压阻式, R-F 为幂律关系, 非反比)
 */
static float mv_to_force_g(int mv)
{
    if (mv < 0) {
        mv = 0;
    }

    float v_mv = (float)mv;

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

    int raw[2] = { 0, 0 };
    int mv[2]  = { 0, 0 };
    for (int i = 0; i < 2; i++) {
        esp_err_t err = adc_oneshot_read(s_adc1, s_ch[i], &raw[i]);
        if (err != ESP_OK) {
            return err;
        }
        /* 原始值 -> 电压(mV): 优先用校准, 否则退回线性估算 */
        if (s_cali[i] != NULL) {
            adc_cali_raw_to_voltage(s_cali[i], raw[i], &mv[i]);
        } else {
            mv[i] = (int)(PRESSURE_ADC_VREF_MV * (float)raw[i] / 4095.0f);
        }
        press_g[i] = mv_to_force_g(mv[i]);
    }

    /* 诊断(节流: 每 200 次≈1s): 打印实测 raw/mV/力, 便于与万用表对比 */
    // static uint32_t log_cnt = 0;
    // if ((log_cnt++ % 200) == 0) {
    //     ESP_LOGI(TAG, "CH4(GPIO%d): raw=%4d %4dmV -> %6.2fg | CH5(GPIO%d): raw=%4d %4dmV -> %6.2fg",
    //              PRESSURE_1_GPIO, raw[0], mv[0], press_g[0],
    //              PRESSURE_2_GPIO, raw[1], mv[1], press_g[1]);
    // }
    return ESP_OK;
}