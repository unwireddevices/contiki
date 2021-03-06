/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup cc26xx-srf-tag
 * @{
 *
 * \defgroup srf06-cc13xx-peripherals Peripherals for the SmartRF06EB + CC1310EM
 *
 * Defines related to the SmartRF06 Evaluation Board with a CC1310EM
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx/CC26xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * SmartRF06 Evaluation Board with a CC1310EM
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name Pinout
 *
 */

/* Without shell */
#define BOARD_IOID_UART_COORDINATOR_TX   IOID_0
#define BOARD_IOID_UART_COORDINATOR_RX   IOID_1
#define BOARD_IOID_UART_SHELL_RX         IOID_8 // IOID_UNUSED
#define BOARD_IOID_UART_SHELL_TX         IOID_9 // IOID_UNUSED
#define BOARD_IOID_LED                   IOID_2
#define BOARD_IOID_PWM                   IOID_3
#define BOARD_IOID_BOOT_BUTTON           IOID_4
#define BOARD_IOID_RELAY                 IOID_6
#define BOARD_IOID_BOOT_MODE             IOID_7
#define BOARD_IOID_FLASH_SPI_CS          IOID_10
#define BOARD_IOID_FLASH_SPI_MISO        IOID_11
#define BOARD_IOID_FLASH_SPI_CLK         IOID_12
#define BOARD_IOID_FLASH_SPI_MOSI        IOID_13

/* With shell */
// #define BOARD_IOID_UART_COORDINATOR_TX   IOID_9
// #define BOARD_IOID_UART_COORDINATOR_RX   IOID_8
// #define BOARD_IOID_UART_SHELL_RX         IOID_1
// #define BOARD_IOID_UART_SHELL_TX         IOID_0
// #define BOARD_IOID_LED                   IOID_2
// #define BOARD_IOID_PWM                   IOID_3
// #define BOARD_IOID_BOOT_BUTTON           IOID_4
// #define BOARD_IOID_RELAY                 IOID_6
// #define BOARD_IOID_BOOT_MODE             IOID_7
// #define BOARD_IOID_FLASH_SPI_CS          IOID_10
// #define BOARD_IOID_FLASH_SPI_MISO        IOID_11
// #define BOARD_IOID_FLASH_SPI_CLK         IOID_12
// #define BOARD_IOID_FLASH_SPI_MOSI        IOID_13

/* UMDK-RF */
// #define BOARD_IOID_BOOT_BUTTON          IOID_1 //+
// #define BOARD_IOID_UART_SHELL_RX        IOID_2 //+
// #define BOARD_IOID_UART_SHELL_TX        IOID_3 //+
// #define BOARD_IOID_LED                  IOID_22 //+
// #define BOARD_IOID_PWM                  IOID_7 //+
// #define BOARD_IOID_RELAY                IOID_6 //+
// #define BOARD_IOID_BOOT_MODE            IOID_23 //+
// #define BOARD_IOID_UART_COORDINATOR_TX  IOID_27 //+
// #define BOARD_IOID_UART_COORDINATOR_RX  IOID_26 //+
// #define BOARD_IOID_FLASH_SPI_CS         IOID_4 //+
// #define BOARD_IOID_FLASH_SPI_MISO       IOID_5 //+
// #define BOARD_IOID_FLASH_SPI_MOSI       IOID_24 //+
// #define BOARD_IOID_FLASH_SPI_CLK        IOID_25 //+

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED configurations
 *
 */
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LED_A
#undef LED_B
#undef LED_C
#undef LED_D
#undef LED_E
#undef LEDS_CONF_ALL

#define LED_A           1
#define LED_B           2
#define LED_C           4
#define LED_D           8
#define LED_E           16

#define LEDS_CONF_ALL 31

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings

 */
#define BOARD_IOID_LED_A          BOARD_IOID_LED //led on radio-board
#define BOARD_IOID_LED_B          IOID_25 //on UMDK-BUTTON
#define BOARD_IOID_LED_C          IOID_26 //on UMDK-BUTTON
#define BOARD_IOID_LED_D          IOID_28 //on UMDK-BUTTON
#define BOARD_IOID_LED_E          IOID_28 //on UMDK-BUTTON
#define BOARD_LED_A               (1 << BOARD_IOID_LED_A)
#define BOARD_LED_B               (1 << BOARD_IOID_LED_B)
#define BOARD_LED_C               (1 << BOARD_IOID_LED_C)
#define BOARD_LED_D               (1 << BOARD_IOID_LED_D)
#define BOARD_LED_E               (1 << BOARD_IOID_LED_E)
#define BOARD_LED_ALL             (BOARD_LED_A | BOARD_LED_B | BOARD_LED_C | \
                                   BOARD_LED_D | BOARD_LED_E)

/*---------------------------------------------------------------------------*/
/**
 * \name Relay IOID mappings
 */
#define BOARD_IOID_RELAY_1          IOID_17
#define BOARD_IOID_RELAY_2          IOID_16
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Dimmer IOID mappings
 */
#define BOARD_IOID_DIMMER_1         IOID_16
#define BOARD_IOID_DIMMER_2         IOID_17
#define ZERO_CROSS_SYNC_IOID        IOID_27

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name 1-10V IOID mappings
 */
#define BOARD_IOID_1_10V_1         IOID_16

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 */

#define BOARD_IOID_UART_TX        BOARD_IOID_UART_SHELL_TX
#define BOARD_IOID_UART_RX        BOARD_IOID_UART_SHELL_RX
#define BOARD_IOID_ALT_UART_TX    BOARD_IOID_UART_COORDINATOR_TX
#define BOARD_IOID_ALT_UART_RX    BOARD_IOID_UART_COORDINATOR_RX
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 */
#define BOARD_IOID_KEY_A            IOID_4 //on UMDK-BUTTON
#define BOARD_IOID_KEY_B            IOID_5 //on UMDK-BUTTON
#define BOARD_IOID_KEY_C            IOID_6 //on UMDK-BUTTON
#define BOARD_IOID_KEY_D            IOID_7 //on UMDK-BUTTON
#define BOARD_IOID_KEY_E            BOARD_IOID_BOOT_BUTTON //generic connect/prog
#define BOARD_KEY_A                 (1 << BOARD_IOID_KEY_A)
#define BOARD_KEY_B                 (1 << BOARD_IOID_KEY_B)
#define BOARD_KEY_C                 (1 << BOARD_IOID_KEY_C)
#define BOARD_KEY_D                 (1 << BOARD_IOID_KEY_D)
#define BOARD_KEY_E                 (1 << BOARD_IOID_KEY_E)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name 3.3V domain IOID mapping
 *
 */
#define BOARD_IOID_3V3_EN         IOID_13
#define BOARD_3V3_EN              (1 << BOARD_IOID_3V3_EN)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI IOID mapping
 *
 */
// #define BOARD_IOID_SPI_SCK        IOID_10
// #define BOARD_IOID_SPI_CLK_FLASH  BOARD_IOID_SPI_SCK
// #define BOARD_IOID_SPI_MOSI       IOID_9
// #define BOARD_IOID_SPI_MISO       IOID_8
// #define BOARD_IOID_FLASH_CS       IOID_14
// #define BOARD_SPI_SCK             (1 << BOARD_IOID_SPI_SCK)
// #define BOARD_SPI_MOSI            (1 << BOARD_IOID_SPI_MOSI)
// #define BOARD_SPI_MISO            (1 << BOARD_IOID_SPI_MISO)





#define BOARD_IOID_SPI_SCK        BOARD_IOID_FLASH_SPI_CLK
#define BOARD_IOID_SPI_CLK_FLASH  BOARD_IOID_SPI_SCK
#define BOARD_IOID_SPI_MOSI       BOARD_IOID_FLASH_SPI_MOSI
#define BOARD_IOID_SPI_MISO       BOARD_IOID_FLASH_SPI_MISO
#define BOARD_IOID_FLASH_CS       BOARD_IOID_FLASH_SPI_CS
#define BOARD_FLASH_CS            (1 << BOARD_IOID_FLASH_CS)
#define BOARD_SPI_SCK             (1 << BOARD_IOID_SPI_SCK)
#define BOARD_SPI_MOSI            (1 << BOARD_IOID_SPI_MOSI)
#define BOARD_SPI_MISO            (1 << BOARD_IOID_SPI_MISO)

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 */
#define BOARD_IOID_SDA            IOID_30 /**< Interface 0 SDA: All sensors bar MPU */
#define BOARD_IOID_SCL            IOID_29 /**< Interface 0 SCL: All sensors bar MPU */
#define BOARD_IOID_SDA_HP         IOID_30 /**< Interface 1 SDA: MPU */
#define BOARD_IOID_SCL_HP         IOID_29 /**< Interface 1 SCL: MPU */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LCD IOID mapping
 *
 */
#define BOARD_IOID_LCD_MODE       IOID_UNUSED
#define BOARD_IOID_LCD_RST        IOID_UNUSED
#define BOARD_IOID_LCD_CS         IOID_UNUSED
#define BOARD_IOID_LCD_SCK        BOARD_IOID_SPI_SCK
#define BOARD_IOID_LCD_MOSI       BOARD_IOID_SPI_MOSI
#define BOARD_LCD_MODE            (1 << BOARD_IOID_LCD_MODE)
#define BOARD_LCD_RST             (1 << BOARD_IOID_LCD_RST)
#define BOARD_LCD_CS              (1 << BOARD_IOID_LCD_CS)
#define BOARD_LCD_SCK             BOARD_SPI_SCK
#define BOARD_LCD_MOSI            BOARD_SPI_MOSI
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SD Card IOID mapping
 *
 */
#define BOARD_IOID_SDCARD_CS      IOID_UNUSED
#define BOARD_SDCARD_CS           (1 << BOARD_IOID_SDCARD_CS)
#define BOARD_IOID_SDCARD_SCK     BOARD_IOID_SPI_SCK
#define BOARD_SDCARD_SCK          BOARD_SPI_SCK
#define BOARD_IOID_SDCARD_MOSI    BOARD_IOID_SPI_MOSI
#define BOARD_SDCARD_MOSI         BOARD_SPI_MOSI
#define BOARD_IOID_SDCARD_MISO    BOARD_IOID_SPI_MISO
#define BOARD_SDCARD_MISO         BOARD_SPI_MISO
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ALS IOID mapping
 *
 */
#define BOARD_IOID_ALS_PWR        IOID_UNUSED
#define BOARD_IOID_ALS_OUT        IOID_UNUSED
#define BOARD_ALS_PWR             (1 << BOARD_IOID_ALS_PWR)
#define BOARD_ALS_OUT             (1 << BOARD_IOID_ALS_OUT)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ACC IOID mapping
 *
 */
#define BOARD_IOID_ACC_PWR        IOID_UNUSED
#define BOARD_IOID_ACC_INT        IOID_UNUSED
#define BOARD_IOID_ACC_INT1       IOID_UNUSED
#define BOARD_IOID_ACC_INT2       IOID_UNUSED
#define BOARD_IOID_ACC_CS         IOID_UNUSED
#define BOARD_ACC_PWR             (1 << BOARD_IOID_ACC_PWR)
#define BOARD_ACC_INT             (1 << BOARD_IOID_ACC_INT)
#define BOARD_ACC_INT1            (1 << BOARD_IOID_ACC_INT1)
#define BOARD_ACC_INT2            (1 << BOARD_IOID_ACC_INT2)
#define BOARD_ACC_CS              (1 << BOARD_IOID_ACC_CS)
#define BOARD_IOID_ACC_SCK        BOARD_IOID_SPI_SCK
#define BOARD_ACC_SCK             BOARD_SPI_SCK
#define BOARD_IOID_ACC_MOSI       BOARD_IOID_SPI_MOSI
#define BOARD_ACC_MOSI            BOARD_SPI_MOSI
#define BOARD_IOID_ACC_MISO       BOARD_IOID_SPI_MISO
#define BOARD_ACC_MISO            BOARD_SPI_MISO
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Bootloader settings
 */
#ifndef SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE
// #define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE 0x00       // Disable ROM boot loader
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE    0xC5       // Enable ROM boot loader
#endif

#ifndef SET_CCFG_BL_CONFIG_BL_LEVEL
#define SET_CCFG_BL_CONFIG_BL_LEVEL             0x0        // Active low to open boot loader backdoor
// #define SET_CCFG_BL_CONFIG_BL_LEVEL          0x1        // Active high to open boot loader backdoor
#endif

#ifndef SET_CCFG_BL_CONFIG_BL_PIN_NUMBER
//#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER      0xFF       // DIO number for boot loader backdoor
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER        BOARD_IOID_BOOT_BUTTON       // DIO number for boot loader backdoor
#endif

#ifndef SET_CCFG_BL_CONFIG_BL_ENABLE
#define SET_CCFG_BL_CONFIG_BL_ENABLE            0xC5       // Enabled boot loader backdoor
//#define SET_CCFG_BL_CONFIG_BL_ENABLE          0xFF       // Disabled boot loader backdoor
#endif
/** @} */
/*---------------------------------------------------------------------------*/

/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "Unwired Devices softel_lighting_board/cc1310 5x5"
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
