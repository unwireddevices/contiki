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
 *         OTA common funtions and defines
 * \author
 *         Manchenko Oleg man4enkoos@gmail.com
 *
 */
/*---------------------------------------------------------------------------*/

#include "string.h"
#include <stdbool.h>
#include "ota-common.h"
#include "dev/watchdog.h"

#define EEPROM_FLAG_ADDR			(0x00070000)
#define MAGIC_BYTES_ADDR			(EEPROM_FLAG_ADDR)
#define MAGIC_BYTES					(0xBABECAFE)
#define MAGIC_BYTES_LENGTH			(sizeof(uint32_t))
#define FLAG_FW_ADDR				(EEPROM_FLAG_ADDR + MAGIC_BYTES_LENGTH)
#define FLAG_FW_LENGTH				(sizeof(uint8_t))

/*---------------------------------------------------------------------------*/

bool write_fw_flag(uint8_t flag_value)
{
	/* Проверяем установлена ли флешка */
	bool eeprom_access = ext_flash_open();
	if(eeprom_access)
	{
		/* ERASE FLASH */
		ext_flash_erase(EEPROM_FLAG_ADDR, (MAGIC_BYTES_LENGTH + FLAG_FW_LENGTH));
		watchdog_periodic();

		/* Записываем флаг в EEPROM */
		ext_flash_write(FLAG_FW_ADDR, FLAG_FW_LENGTH, (uint8_t*)&flag_value);

		/* MAGIC BYTES */
		uint32_t magic_bytes = MAGIC_BYTES;
    	ext_flash_write(MAGIC_BYTES_ADDR, MAGIC_BYTES_LENGTH, (uint8_t*)&magic_bytes);
		ext_flash_close();

		return FLAG_OK_WRITE;
	}
	else
	{
		// print_uart("Could not access EEPROM\n");
		ext_flash_close();

		return FLAG_ERROR_WRITE;
	}
}

/*---------------------------------------------------------------------------*/

uint8_t read_fw_flag(void)
{
	/* Проверяем установлена ли флешка */
	bool eeprom_access = ext_flash_open();
	if(eeprom_access)
	{
		/* Проверяет есть ли сохранённый флаг */
		uint32_t magic_bytes; 

		/* Считываем магические байты */
		ext_flash_read(MAGIC_BYTES_ADDR, MAGIC_BYTES_LENGTH, (uint8_t*)&magic_bytes);

		if(magic_bytes != MAGIC_BYTES)
		{
			ext_flash_close();
			write_fw_flag(FW_FLAG_NON_UPDATE);

			return FW_FLAG_NON_UPDATE;
		}

		/* Считываем флаг */
		uint8_t flag_value;
		ext_flash_read(FLAG_FW_ADDR, FLAG_FW_LENGTH, (uint8_t*)&flag_value);
		ext_flash_close();

		return flag_value;
	}
	else
	{
		// print_uart("Could not access EEPROM\n");
		ext_flash_close();

		return FW_FLAG_NON_UPDATE;
	}   
}

/*---------------------------------------------------------------------------*/
