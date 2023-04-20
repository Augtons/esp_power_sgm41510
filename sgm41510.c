#include <stdio.h>
#include "sgm41510.h"
#include "esp_log.h"

/**
 * @brief 初始化I2C
 * @param port
 * @param sda
 * @param scl
 * @return
 */
static esp_err_t init_i2c(i2c_port_t port, int sda, int scl) {
    esp_err_t err = ESP_OK;

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        goto finally;
    }

    i2c_config_t conf = {
        .mode 			  = I2C_MODE_MASTER,
        .sda_io_num 	  = sda,
        .sda_pullup_en 	  = GPIO_PULLUP_ENABLE,
        .scl_io_num 	  = scl,
        .scl_pullup_en 	  = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SGM41510_I2C_CLK_FREQ,
        .clk_flags = 0,//此句可以注释掉
    };
    err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        goto finally;
    }
finally:
    return err;
}

/**
 * @brief 通过I2C往寄存器里写内容
 * @param i2c_num
 * @param device_address
 * @param reg_addr
 * @param write_buffer
 * @param write_size
 * @param ticks_to_wait
 * @return
 */
static esp_err_t i2c_master_write_to_reg(i2c_port_t i2c_num, uint8_t device_address,
                                     uint8_t reg_addr,
                                     const uint8_t* write_buffer, size_t write_size,
                                     TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, reg_addr, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write(handle, write_buffer, write_size, true);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);

end:
    i2c_cmd_link_delete(handle);
    return err;
}

esp_err_t sgm41510_init(sgm41510_dev_t* out_dev, const sgm41510_config_t* config) {
    esp_err_t err = ESP_OK;

    if (out_dev->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    err = init_i2c(config->i2c_port, config->sda, config->scl);
    if (err != ESP_OK) {
        goto finally;
    }

    out_dev->inited = true;
    out_dev->config = *config;

    if (out_dev->config.monitor.adc_callback == NULL) {
        out_dev->config.monitor.enable = false;
    }

    if (out_dev->config.monitor.enable) {
        sgm41510_enable_battery_monitor(out_dev);
    }

finally:
    return err;
}

esp_err_t sgm41510_deinit(sgm41510_dev_t* dev) {
    esp_err_t err = i2c_driver_delete(dev->config.i2c_port);
    if (err != ESP_OK) {
        goto finally;
    }
    dev->inited = false;
finally:
    return err;
}


esp_err_t sgm41510_read_reg(const sgm41510_dev_t* dev, uint8_t reg_addr, uint8_t* databuf, uint8_t len) {
    esp_err_t err = ESP_OK;

    if (!dev->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    err = i2c_master_write_read_device(dev->config.i2c_port, SGM41510_I2C_SLAVE_ADDR,
                                       &reg_addr, 1, databuf, len, pdMS_TO_TICKS(250));
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t sgm41510_write_reg(const sgm41510_dev_t* dev, uint8_t reg_addr, uint8_t* databuf, uint8_t len) {
    esp_err_t err = ESP_OK;

    if (!dev->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    err = i2c_master_write_to_reg(dev->config.i2c_port, SGM41510_I2C_SLAVE_ADDR,
                                  reg_addr, databuf, len, pdMS_TO_TICKS(250));
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t sgm41510_get_charge_status(const sgm41510_dev_t* dev, sgm41510_charge_status_t* out_status) {
    esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = sgm41510_read_reg(dev, 0x0B, &data, 1);
    if (err != ESP_OK) {
        goto finally;
    }

    data >>= 3;
    data &= 0b11;

    *out_status = (sgm41510_charge_status_t)data;

finally:
    return err;
}

const char* sgm41510_charge_status_to_str(sgm41510_charge_status_t status) {
    switch (status) {
        case SGM_STAT_DISABLED:      return "Disabled";
        case SGM_STAT_PRE_CHARGE:    return "Pre charge";
        case SGM_STAT_FAST_CHARGING: return "Fast charging";
        case SGM_STAT_FINISHED:      return "Charge Finished";
    }
    return "Unknown";
}

void sgm41510_set_battery_monitor(sgm41510_dev_t* dev, sgm41510_adc_callback callback) {
    dev->config.monitor.adc_callback = callback;
}

esp_err_t sgm41510_enable_or_disable_battery_monitor(sgm41510_dev_t* dev, bool enable) {
    esp_err_t err = ESP_OK;
    dev->config.monitor.enable = enable;

    uint8_t data = 0;
    err = sgm41510_read_reg(dev, 0x02, &data, 1);
    if (err != ESP_OK) { return err; }

    if (enable) {
        data |= 0x80;
    } else {
        data &= ~0x80;
    }
    err = sgm41510_write_reg(dev, 0x02, &data, 1);
    return err;
}

esp_err_t sgm41510_battery_monitor(const sgm41510_dev_t* dev, sgm41510_monitor_mode_t mode, float* value) {
    esp_err_t err = ESP_OK;
    uint8_t data = 0;

    // 判断是否启用了监视器功能
    if (!dev->config.monitor.enable || !dev->config.monitor.adc_callback) {
        return ESP_ERR_INVALID_STATE;
    }

    // 读取先前的数据
    err = sgm41510_read_reg(dev, 0x15, &data, 1);
    if (err != ESP_OK) {
        goto finally;
    }

    // 擦除原模式，写入新模式
    data &= ~0b11100000;
    data |= (uint8_t)mode << 5;

    err = sgm41510_write_reg(dev, 0x15, &data, 1);
    if (err != ESP_OK) {
        goto finally;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // 读取ADC (mV)
    int cm_mv = dev->config.monitor.adc_callback();
    if (cm_mv < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // 计算
    switch (mode) {
        case SGM_MONITOR_CHARGE_CURRENT:
        case SGM_MONITOR_INPUT_CURRENT:
            *value = (float)cm_mv * 40000.0F / CONFIG_SGM41510_CM_R;
            break;

        case SGM_MONITOR_BAT_CURRENT:
            *value = (float)cm_mv * 40000.0F / CONFIG_SGM41510_CM_R - 6400.0F;
            break;

        case SGM_MONITOR_BAT_VOLTAGE:
        case SGM_MONITOR_OUT_VOLTAGE:
            *value = (float)cm_mv * 2;
            break;

        case SGM_MONITOR_IN_VOLTAGE:
            *value = (float)cm_mv * 10;
            break;
    }

finally:
    return err;
}