#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sgm41510.h"

#define I2C_MASTER_SDA_IO  (18)
#define I2C_MASTER_SCL_IO  (19)

esp_adc_cal_characteristics_t adc_char;
sgm41510_dev_t sgm;

static int sgm_adc_callback() {
    int64_t adc_average = 0;
    for (int i = 0; i < 128; i++) {
        adc_average += adc1_get_raw(ADC1_CHANNEL_5);
    }
    adc_average /= 128;

    int v = (int)esp_adc_cal_raw_to_voltage(adc_average, &adc_char);
    ESP_LOGI("MAIN", "ADC = %d, ADC_Average = %lld", v, adc_average);
    return v;
}

_Noreturn
void app_main(void)
{
    esp_err_t err = ESP_OK;

    // 配置 ADC，通道5 对应 GPIO33
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

    // 读取 ADC-Voltage 特征曲线
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_char);

    // 初始化 SGM41510
    err = sgm41510_init(&sgm, &(sgm41510_config_t) {
        .i2c_port = I2C_NUM_0,
        .sda = I2C_MASTER_SDA_IO,
        .scl = I2C_MASTER_SCL_IO,
        .monitor = {
            .enable = true,
            .adc_callback = sgm_adc_callback
        }
    });
    ESP_ERROR_CHECK(err);

    // 读取数据
    struct monitor_data {
        sgm41510_charge_status_t status;
        float vin;
        float iin;
        float vbat;
        float ibat;
        float vout;
        float icharge;
    } monitor_data = {0};

    while (1) {
        sgm41510_enable_battery_monitor(&sgm);

        sgm41510_get_charge_status(&sgm, &monitor_data.status);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_IN_VOLTAGE, &monitor_data.vin);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_INPUT_CURRENT, &monitor_data.iin);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_BAT_VOLTAGE, &monitor_data.vbat);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_BAT_CURRENT, &monitor_data.ibat);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_OUT_VOLTAGE, &monitor_data.vout);
        sgm41510_battery_monitor(&sgm, SGM_MONITOR_CHARGE_CURRENT, &monitor_data.icharge);

        ESP_LOGI("MAIN", "+Status = %s", sgm41510_charge_status_to_str(monitor_data.status));
        ESP_LOGI("MAIN", "    |- V_in = %.2f", monitor_data.vin);
        ESP_LOGI("MAIN", "    |- I_in = %.2f", monitor_data.iin);
        ESP_LOGI("MAIN", "    |- V_bat = %.2f", monitor_data.vbat);
        ESP_LOGI("MAIN", "    |- I_bat = %.2f", monitor_data.ibat);
        ESP_LOGI("MAIN", "    |- V_out = %.2f", monitor_data.vout);
        ESP_LOGI("MAIN", "    |- I_charge = %.2f", monitor_data.icharge);


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
