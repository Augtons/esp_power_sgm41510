//
// Created by augtons on 23-4-20.
//

#ifndef _SGM41510_TYPES_H
#define _SGM41510_TYPES_H

#include "sdkconfig.h"
#include "esp_wifi_types.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*sgm41510_adc_callback)();

typedef struct {
    i2c_port_t              i2c_port;
    gpio_num_t              sda;
    gpio_num_t              scl;
    struct {
        bool                    enable;
        sgm41510_adc_callback   adc_callback;
    } monitor;

} sgm41510_config_t;

typedef struct {
    sgm41510_config_t       config;
    bool                    inited;
} sgm41510_dev_t;

typedef enum {
    SGM_STAT_DISABLED      = 0b00,
    SGM_STAT_PRE_CHARGE    = 0b01,
    SGM_STAT_FAST_CHARGING = 0b10,
    SGM_STAT_FINISHED      = 0b11,
} sgm41510_charge_status_t;

typedef enum {
    SGM_MONITOR_CHARGE_CURRENT  = 0b001,
    SGM_MONITOR_BAT_CURRENT     = 0b011,
    SGM_MONITOR_INPUT_CURRENT   = 0b100,
    SGM_MONITOR_BAT_VOLTAGE     = 0b101,
    SGM_MONITOR_OUT_VOLTAGE     = 0b110,
    SGM_MONITOR_IN_VOLTAGE      = 0b111,
} sgm41510_monitor_mode_t;

#ifdef __cplusplus
};
#endif

#endif //_SGM41510_TYPES_H
