/*
 * Copyright (c) 2016, Unwired Devices LLC - http://www.unwireddevices.com/
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Unwired Devices nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/*
 * \file
 *         System functions for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "dev/watchdog.h"

#include <string.h>
#include <stdio.h>

#include "ud_binary_protocol.h"
#include "xxf_types_helper.h"

#include "batmon-sensor.h"
#include "core/lib/sensors.h"

#include "net/rpl/rpl.h"
#include "net/link-stats.h"

#include <ctype.h> // for str2xxx
#include <errno.h> // for str2xxx
#include <limits.h> // for str2xxx

#include "system-common.h"
#include "ota-main.h"
#include "ota-common.h"

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/
/*Иннициализация UART*/
void on_uart(uint32_t rx_dio, uint32_t tx_dio, uint32_t baud_rate)
{
   if(baud_rate == 9600) //Не обновляется скорость UART
   {
	  (*(unsigned long*)(0x40001024)) = 312;
	  (*(unsigned long*)(0x40001028)) = 32;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   else 
   {
	  (*(unsigned long*)(0x40001024)) = 26;
	  (*(unsigned long*)(0x40001028)) = 3;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   
   ti_lib_ioc_io_port_pull_set(IOID_26, IOC_IOPULL_UP);
   ti_lib_ioc_pin_type_gpio_output(tx_dio);
   ti_lib_gpio_set_dio(tx_dio);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_ioc_pin_type_uart(UART0_BASE, rx_dio, tx_dio, IOID_UNUSED, IOID_UNUSED);
   ti_lib_uart_config_set_exp_clk(UART0_BASE, ti_lib_sys_ctrl_clock_get(), baud_rate, 
                  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   ti_lib_uart_fifo_level_set(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
   ti_lib_uart_fifo_enable(UART0_BASE);
   ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_enable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
}

/*---------------------------------------------------------------------------*/
/*Деиннициализация UART*/
void off_uart(uint32_t rx_dio, uint32_t tx_dio)
{
   ti_lib_ioc_port_configure_set(tx_dio, IOC_PORT_GPIO, IOC_STD_OUTPUT);
   ti_lib_ioc_port_configure_set(rx_dio, IOC_PORT_GPIO, IOC_STD_INPUT);
   ti_lib_gpio_set_output_enable_dio(tx_dio, GPIO_OUTPUT_ENABLE);
   ti_lib_gpio_set_dio(tx_dio);

   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_disable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_fifo_disable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_disable(UART0_BASE);
}

/*---------------------------------------------------------------------------*/
/**/
void flash_damp_hex(uint8_t mode)
{
	const uint32_t start_adress = (ota_images[1-1] << 12);
	const uint32_t read_length = 0x400;
	uint8_t flash_read_data_buffer[read_length];

	printf("SPIFLASH DAMP: \n");
	for (uint8_t page = 0; page < 100; page++ )
	{
		watchdog_periodic();
		ext_flash_open();
		bool eeprom_access = ext_flash_read(start_adress + (read_length * page), read_length, flash_read_data_buffer);
		ext_flash_close();

		if(!eeprom_access)
		{
			printf("SPIFLASH: Error - Could not read EEPROM\n");
		}
		else
		{
			if (mode == HEXVIEW_MODE)
				hexview_print(read_length, flash_read_data_buffer, start_adress+(read_length*page));
			if (mode == HEXRAW_MODE)
				hexraw_print(read_length, flash_read_data_buffer);
		}
	}
	printf("\nSPIFLASH DAMP END \n");
}

/*---------------------------------------------------------------------------*/
/*Выводит содержимое памяти по адресу*/
void hexraw_print(uint32_t flash_length, uint8_t *flash_read_data_buffer)
{
	for (uint32_t i = 0; i < flash_length; i++)
	{
		printf(" %"PRIXX8, flash_read_data_buffer[i]);
	}
}

/*---------------------------------------------------------------------------*/
/*Выводит содержимое памяти по адресу*/
void hexview_print(uint32_t flash_length, uint8_t *flash_read_data_buffer, uint32_t offset)
{
	for (uint32_t i = 0; i < flash_length; i = i + 16)
	{
		printf("0x%"PRIXX32": ", i + offset);
		for (int i2 = 0; i2 < 16; i2++)
		{
			printf("%"PRIXX8" ", flash_read_data_buffer[i2+i]);
			if (i2 == 7) 
				printf(" ");
		}
		printf("\n");
	}
}

/*---------------------------------------------------------------------------*/
/*Считает CRC16 со стартовым значением 0x0000*/
uint16_t crc16_arc(uint8_t *data, uint16_t len)
{
	uint16_t crc = 0x0000;
	uint16_t j;
	int i;
	// Note: 0xA001 is the reflection of 0x8005
	for (j = len; j > 0; j--)
	{
		crc ^= *data++;
		for (i = 0; i < 8; i++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}
	
	return (crc);
}

/*---------------------------------------------------------------------------*/
/*Считает CRC16 со стартовым значением 0xFFFF*/
uint16_t crc16_modbus(uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t j;
	int i;
	// Note: 0xA001 is the reflection of 0x8005
	for (j = len; j > 0; j--)
	{
		crc ^= *data++;
		for (i = 0; i < 8; i++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}
	
	return (crc);
}

/*---------------------------------------------------------------------------*/
/*Возвращает напряжение*/
uint8_t get_voltage()
{
	return (uint8_t)((((batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT) * 125) >> 5) - 2000) / 50);
}

/*---------------------------------------------------------------------------*/
/*Возвращает темперетуру*/
uint8_t get_temperature()
{
	uint32_t temp_offset = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP) + 50;
	uint8_t temp_offset_u8 = (uint8_t)temp_offset;
	
	return temp_offset_u8;
}

/*---------------------------------------------------------------------------*/
/*Возвращает RSSI*/
uint8_t get_parent_rssi()
{
	const rpl_dag_t *dag = NULL;
	const struct link_stats *stat_parent = NULL;
	dag = rpl_get_any_dag();
	if (dag != NULL)
	{
		stat_parent = rpl_get_parent_link_stats(dag->preferred_parent);
		if (stat_parent != NULL)
		{
			int16_t rssi_int = ((stat_parent->rssi) + 200);
			uint16_t rssi_uint = (uint16_t)rssi_int;
			uint8_t rssi_uint_8t = (uint8_t)rssi_uint;
			return rssi_uint_8t;
		}
	}
	
	return 0xFF;
}

/*---------------------------------------------------------------------------*/
/**/
str2int_errno_t hex_str2uint16(uint16_t *out, char *s) 
{
	char *end;
	if (s[0] == '\0' || isspace((unsigned char) s[0]))
		return STR2INT_INCONVERTIBLE;
	
	errno = 0;
	long l = strtol(s, &end, 16);
   
	/* Both checks are needed because INT_MAX == LONG_MAX is possible. */
	if (l > 0xFFFF || (errno == ERANGE && l == LONG_MAX))
		return STR2INT_OVERFLOW;
   
	if (l < 0 || (errno == ERANGE && l == LONG_MIN))
		return STR2INT_UNDERFLOW;
   
	if (*end != '\0')
		return STR2INT_INCONVERTIBLE;
   
	*out = (uint16_t)l;
	return STR2INT_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/**/
str2int_errno_t hex_str2uint8(uint8_t *out, char *s) 
{
	char *end;
	if (s[0] == '\0' || isspace((unsigned char) s[0]))
		return STR2INT_INCONVERTIBLE;
	
	errno = 0;
	long l = strtol(s, &end, 16);

	// Both checks are needed because INT_MAX == LONG_MAX is possible. //
	if (l > 255 || (errno == ERANGE && l == LONG_MAX))
		return STR2INT_OVERFLOW;
	
	if (l < 0 || (errno == ERANGE && l == LONG_MIN))
		return STR2INT_UNDERFLOW;
	
	if (*end != '\0')
		return STR2INT_INCONVERTIBLE;
	
	*out = (uint8_t)l;
	return STR2INT_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/**/
str2int_errno_t dec_str2uint8(uint8_t *out, char *s) 
{
	char *end;
	if (s[0] == '\0' || isspace((unsigned char) s[0]))
		return STR2INT_INCONVERTIBLE;
	
	errno = 0;
	long l = strtol(s, &end, 10);
	
	/* Both checks are needed because INT_MAX == LONG_MAX is possible. */
	if (l > 255 || (errno == ERANGE && l == LONG_MAX))
		return STR2INT_OVERFLOW;
	
	if (l < 0 || (errno == ERANGE && l == LONG_MIN))
		return STR2INT_UNDERFLOW;
	
	if (*end != '\0')
		return STR2INT_INCONVERTIBLE;
	
	*out = (uint8_t)l;
	return STR2INT_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/**/
str2int_errno_t dec_str2uint32(uint32_t *out, char *s) 
{
	char *end;
	if (s[0] == '\0' || isspace((unsigned char) s[0]))
		return STR2INT_INCONVERTIBLE;

	errno = 0;
	long l = strtol(s, &end, 10);
	
	/* Both checks are needed because INT_MAX == LONG_MAX is possible. */
	if (l > 4294967294 || (errno == ERANGE && l == LONG_MAX))
		return STR2INT_OVERFLOW;
	
	if (l < 0 || (errno == ERANGE && l == LONG_MIN))
		return STR2INT_UNDERFLOW;
	
	if (*end != '\0')
		return STR2INT_INCONVERTIBLE;
	
	*out = (uint32_t)l;
	return STR2INT_SUCCESS;
}

/*---------------------------------------------------------------------------*/