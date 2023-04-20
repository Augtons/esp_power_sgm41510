#ifndef _SGM41510_H_
#define _SGM41510_H_

#include "esp_err.h"
#include "sgm41510_types.h"

#define SGM41510_I2C_CLK_FREQ           (CONFIG_SGM41510_I2C_CLK_FREQ)
#define SGM41510_I2C_SLAVE_ADDR         (CONFIG_SGM41510_I2C_SLAVE_ADDR)

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sgm41510_init(sgm41510_dev_t* out_dev, const sgm41510_config_t* config);
esp_err_t sgm41510_deinit(sgm41510_dev_t* dev);

esp_err_t sgm41510_read_reg(const sgm41510_dev_t* dev, uint8_t reg_addr, uint8_t* databuf, uint8_t len);
esp_err_t sgm41510_write_reg(const sgm41510_dev_t* dev, uint8_t reg_addr, uint8_t* databuf, uint8_t len);

esp_err_t sgm41510_get_charge_status(const sgm41510_dev_t* dev, sgm41510_charge_status_t* out_status);
const char* sgm41510_charge_status_to_str(sgm41510_charge_status_t status);

void sgm41510_set_battery_monitor(sgm41510_dev_t* dev, sgm41510_adc_callback callback);
esp_err_t sgm41510_enable_or_disable_battery_monitor(sgm41510_dev_t* dev, bool enable);

inline esp_err_t sgm41510_enable_battery_monitor(sgm41510_dev_t* dev) {
    return sgm41510_enable_or_disable_battery_monitor(dev, 1);
}
inline esp_err_t sgm41510_disable_battery_monitor(sgm41510_dev_t* dev) {
    return sgm41510_enable_or_disable_battery_monitor(dev, 0);
}

esp_err_t sgm41510_battery_monitor(const sgm41510_dev_t* dev, sgm41510_monitor_mode_t mode, float* value);

#ifdef __cplusplus
};
#endif

#endif //_SGM41510_H_