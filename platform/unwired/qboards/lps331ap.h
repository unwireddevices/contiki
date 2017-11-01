/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_lps331ap LPS331AP Pressure Sensor Driver
 * @ingroup     drivers_sensors
 * @brief       Device driver for the LPS331AP pressure sensor
 * @{
 *
 * @file
 * @brief       Device driver interface for the LPS331AP pressure sensor
 *
 * @note This driver uses the sensors I2C interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef LPS331AP_H
#define LPS331AP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board-i2c.h"

 /**
  * @brief The sensors default I2C address
  */
#define LPS331AP_DEFAULT_ADDRESS        0x5c

typedef enum {
    LPS331AP_RATE_1HZ = 1,      /**< sample with 1Hz */
    LPS331AP_RATE_7HZ = 5,      /**< sample with 7Hz */
    LPS331AP_RATE_12HZ5 = 6,    /**< sample with 12.5Hz */
    LPS331AP_RATE_25HZ = 7      /**< sample with 25Hz */
} lps331ap_rate_t;

/**
 * @brief   Struct holding all parameters needed for device initialization
 */

/**
 * @brief Initialize a given LPS331AP pressure sensor
 *
 * @param[out] dev      device descriptor of the sensor
 * @param[in]  i2c      I2C bus the sensor is connected to
 * @param[in]  address  the sensor's address on the I2C bus
 * @param[in]  rate     internal sampling rate of the sensor
 *
 * @return              0 on success
 * @return              -1 on error
 */
int lps331ap_init(lps331ap_rate_t rate);

/**
 * @brief Read a temperature value from the given sensor, returned in m°C
 *
 * @param[in] dev       device descriptor of sensor to read from
 *
 * @return              temperature value in m°C
 */
double lps331ap_read_temp();

/**
 * @brief Read a pressure value from the given sensor, returned in mbar
 *
 * @param[in] dev       device descriptor of sensor to read from
 *
 * @return              pressure value in mbar
 */
int lps331ap_read_pres();

/**
 * @brief Enable the given sensor
 *
 * @param[in] dev       device descriptor of sensor to enable
 *
 * @return              0 on success
 * @return              -1 on error
 */
int lps331ap_enable();

/**
 * @brief Disable the given sensor
 *
 * @param[in] dev       device descriptor of sensor to disable
 *
 * @return              0 on success
 * @return              -1 on error
 */
int lps331ap_disable();

#ifdef __cplusplus
}
#endif

#endif /* LPS331AP_H */
/** @} */
