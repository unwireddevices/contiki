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
/**
 * \file
 *         Header file for system functions
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 */
/*---------------------------------------------------------------------------*/
#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "protocol.h"

/*---------------------------------------------------------------------------*/

#define HEXVIEW_MODE                            0x00
#define HEXRAW_MODE                             0x01

typedef enum 
{
	UMDK_OPT3001_CMD_SET_PERIOD = 0,
	UMDK_OPT3001_CMD_POLL = 1,
} umdk_opt3001_cmd_t;

typedef enum str2int_errno_t
{
	STR2INT_SUCCESS,
	STR2INT_OVERFLOW,
	STR2INT_UNDERFLOW,
	STR2INT_INCONVERTIBLE
} str2int_errno_t;

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF FUNCTIONS*/
void on_uart(uint32_t rx_dio, uint32_t tx_dio, uint32_t baud_rate);
void off_uart(uint32_t rx_dio, uint32_t tx_dio);
void hexraw_print(uint32_t flash_length, uint8_t *flash_read_data_buffer);
void hexview_print(uint32_t flash_length, uint8_t *flash_read_data_buffer, uint32_t offset);
void flash_damp_hex(uint8_t mode);
uint16_t crc16_arc(uint8_t *data, uint16_t len);
uint16_t crc16_modbus(uint8_t *data, uint16_t len);
bool is_array_zero(uint8_t *array_of_zeros);
uint8_t get_voltage();
uint8_t get_parent_rssi();
uint8_t get_temperature();

str2int_errno_t hex_str2uint16(uint16_t *out, char *s);
str2int_errno_t hex_str2uint16(uint16_t *out, char *s);
str2int_errno_t dec_str2uint8(uint8_t *out, char *s);
str2int_errno_t dec_str2uint32(uint32_t *out, char *s);
str2int_errno_t hex_str2uint8(uint8_t *out, char *s);

/*---------------------------------------------------------------------------*/
#endif	/* #ifndef SYSTEM_COMMON_H */