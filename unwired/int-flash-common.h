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
 *         Header file for internal flash read-write functions
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 */
/*---------------------------------------------------------------------------*/
#ifndef INT_FLASH_COMMON_H
#define INT_FLASH_COMMON_H

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ecc.h"

#define USER_FLASH_LENGTH                       100

#define START_USER_FLASH                        0x1E000
#define END_USER_FLASH                          USER_FLASH_LENGTH + START_USER_FLASH

#define START_ON_LAST_STATE                     0x01
#define START_ON_ON_STATE                       0x02
#define START_ON_OFF_STATE                      0x03

#define POWER_1_DIO                             BOARD_IOID_RELAY_1
#define POWER_1_CH_LAST_STATE_OFFSET            0x01
#define POWER_1_CH_START_STATE_OFFSET           0x02

#define POWER_2_DIO                             BOARD_IOID_RELAY_2
#define POWER_2_CH_LAST_STATE_OFFSET            0x03
#define POWER_2_CH_START_STATE_OFFSET           0x04

#define OTA_STATUS_FLAG                         0x05

#define BLANK_FLASH_VALUE                       0xFF

/*---------------------------------------------------------------------------*/

typedef  struct {
	uint16_t panid; 				
    uint8_t channel; 						
	uint8_t aes_key[16];					
	uint8_t panid_configured;
	uint8_t channel_configured;	
	uint8_t aes_key_configured;	
	uint8_t is_backup_golden_image;	
} eeprom_t;

/*---------------------------------------------------------------------------*/
uint32_t write_eeprom(uint8_t *pui8DataBuffer, uint32_t ui32Count);
void read_eeprom(uint8_t *pui8DataBuffer, uint32_t ui32Count);

/*Обновляет ключ шифрования и перезагружает*/
void aes128_key_update(const uint8_t *aes_key_new);

// /*Возвращает указатель на массив в котором хранится ключ шифрования*/
// uint8_t *get_aes128_key(void);

/*Обновляет channel в EEPROM*/
void channel_update(uint8_t channel_new);

/*Обновляет PANID в EEPROM*/
void panid_update(uint16_t panid_new);

void user_flash_update_byte(uint8_t offset, uint8_t data);
uint8_t user_flash_read_byte(uint8_t offset);
void flash_read(uint8_t *pui8DataBuffer, uint32_t ui32Address, uint32_t ui32Count);
uint32_t flash_write(uint8_t *pui8DataBuffer, uint32_t ui32Address, uint32_t ui32Count);

/*---------------------------------------------------------------------------*/
#endif  /* INT_FLASH_COMMON_H */