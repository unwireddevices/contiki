/*
 * Copyright (c) 2014, OpenMote Technologies, S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup openmote-sht21-sensor
 * @{
 *
 * \file
 * Driver for the SHT21 temperature and relative humidity sensor
 *
 * \author
 * Pere Tuset <peretuset@openmote.com>
 */
/*---------------------------------------------------------------------------*/
#include "board-i2c.h"
#include "sht21.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
/**
 * \name SHT21 address
 */
#define SHT21_ADDRESS                   (0x40)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SHT21 register addresses and values
 * @{
 */
#define SHT21_USER_REG_READ             (0xE7)
#define SHT21_USER_REG_WRITE            (0xE6)
#define SHT21_USER_REG_RESERVED_BITS    (0x38)

#define SHT21_TEMPERATURE_HM_CMD        (0xE3)
#define SHT21_HUMIDITY_HM_CMD           (0xE5)
#define SHT21_TEMPERATURE_NHM_CMD       (0xF3)
#define SHT21_HUMIDITY_NHM_CMD          (0xF5)
#define SHT21_RESET_CMD                 (0xFE)

#define SHT21_STATUS_MASK               (0xFC)

#define SHT21_RESOLUTION_12b_14b        ((0 << 7) | (0 << 0))
#define SHT21_RESOLUTION_8b_12b         ((0 << 7) | (1 << 0))
#define SHT21_RESOLUTION_10b_13b        ((1 << 7) | (0 << 0))
#define SHT21_RESOLUTION_11b_11b        ((1 << 7) | (1 << 0))
#define SHT21_BATTERY_ABOVE_2V25        (0 << 6)
#define SHT21_BATTERY_BELOW_2V25        (1 << 6)
#define SHT21_ONCHIP_HEATER_ENABLE      (1 << 2)
#define SHT21_ONCHIP_HEATER_DISABLE     (0 << 2)
#define SHT21_OTP_RELOAD_ENABLE         (0 << 1)
#define SHT21_OTP_RELOAD_DISABLE        (1 << 1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SHT21 configuration values
 * @{
 */
#define SHT21_DEFAULT_CONFIG            (SHT21_RESOLUTION_12b_14b | \
                                         SHT21_ONCHIP_HEATER_DISABLE | \
                                         SHT21_BATTERY_ABOVE_2V25 | \
                                         SHT21_OTP_RELOAD_DISABLE)

#define SHT21_USER_CONFIG               (SHT21_RESOLUTION_8b_12b | \
                                         SHT21_ONCHIP_HEATER_DISABLE | \
                                         SHT21_BATTERY_ABOVE_2V25 | \
                                         SHT21_OTP_RELOAD_DISABLE)
/** @} */
/*---------------------------------------------------------------------------*/
void
sht21_init(void)
{
  uint8_t config[2];

  /* Setup the configuration vector, the first position holds address */
  /* and the second position holds the actual configuration */
  config[0] = SHT21_USER_REG_WRITE;
  config[1] = 0;

  PRINTF("SHT: Init I2C.\n");
  board_i2c_select(BOARD_I2C_INTERFACE_0, SHT21_ADDRESS);

  PRINTF("SHT: Send read config cmd.\n");
  /* Read the current configuration according to the datasheet (pag. 9, fig. 18) */
  board_i2c_write_single(SHT21_USER_REG_READ);
  PRINTF("SHT: Read config.\n");
  board_i2c_read(&config[1], 1);

  /* Clean all the configuration bits except those reserved */
  config[1] &= SHT21_USER_REG_RESERVED_BITS;

  /* Set the configuration bits without changing those reserved */
  config[1] |= SHT21_USER_CONFIG;

  PRINTF("SHT: Send proper config.\n");
  board_i2c_write(config, sizeof(config));
}
/*---------------------------------------------------------------------------*/
void
sht21_reset(void)
{
  /* Send a soft-reset command according to the datasheet (pag. 9, fig. 17) */
  board_i2c_write_single(SHT21_RESET_CMD);
}
/*---------------------------------------------------------------------------*/
bool
sht21_is_present(void)
{
  uint8_t is_present;

  /* Read the current configuration according to the datasheet (pag. 9, fig. 18) */
  PRINTF("SHT: Send read config cmd.\n");
  board_i2c_write_single(SHT21_USER_REG_READ);
  board_i2c_read(&is_present, 1);
  PRINTF("SHT: readed 0x%x.\n", is_present);
  /* Clear the reserved bits according to the datasheet (pag. 9, tab. 8) */
  is_present &= ~SHT21_USER_REG_RESERVED_BITS;

  return (is_present == SHT21_USER_CONFIG) || (is_present == SHT21_DEFAULT_CONFIG);
}
/*---------------------------------------------------------------------------*/
uint16_t
sht21_read_temperature(void)
{
  uint8_t sht21_temperature[2];
  uint16_t temperature;

  /* Read the current temperature according to the datasheet (pag. 8, fig. 15) */
  board_i2c_write_single(SHT21_TEMPERATURE_HM_CMD);
  board_i2c_read(sht21_temperature, sizeof(sht21_temperature));

  temperature = (sht21_temperature[0] << 8) | (sht21_temperature[1] & SHT21_STATUS_MASK);

  return temperature;
}
/*---------------------------------------------------------------------------*/
float
sht21_convert_temperature(uint16_t temperature)
{
  float result;

  result = -46.85;
  result += 175.72 * (float)temperature / 65536.0;

  return result;
}
/*---------------------------------------------------------------------------*/
uint16_t
sht21_read_humidity(void)
{
  uint8_t sht21_humidity[2];
  uint16_t humidity;

  /* Read the current humidity according to the datasheet (pag. 8, fig. 15) */
  board_i2c_write_single(SHT21_HUMIDITY_HM_CMD);
  board_i2c_read(sht21_humidity, sizeof(sht21_humidity));

  humidity = (sht21_humidity[0] << 8) | (sht21_humidity[1] & SHT21_STATUS_MASK);

  return humidity;
}
/*---------------------------------------------------------------------------*/
float
sht21_convert_humidity(uint16_t humidity)
{
  float result;

  result = -6.0;
  result += 125.0 * (float)humidity / 65536.0;

  return result;
}
/*---------------------------------------------------------------------------*/
void
sht21_close()
{
  board_i2c_shutdown();
}
/*---------------------------------------------------------------------------*/
/** @} */
