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
 *         Crypto functions for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"

#include <string.h>
#include <stdio.h>
#include "ti-lib.h"

#include "crypto-common.h"

#include "xxf_types_helper.h"

#include <cpu/cc26xx-cc13xx/lib/cc13xxware/driverlib/rom_crypto.h>
#include <cpu/cc26xx-cc13xx/lib/cc13xxware/driverlib/crypto.h>

/*---------------------------------------------------------------------------*/

uint16_t iterator_to_byte(uint16_t iterator)
{
	if(iterator <= 16)
		return 16;
	if((iterator > 16) && (iterator <= 32))
		return 32;
	if((iterator > 32) && (iterator <= 48))
		return 48;
	if((iterator > 48) && (iterator <= 64))
		return 64;
	if((iterator > 64) && (iterator <= 80))
		return 80;
	if((iterator > 80) && (iterator <= 96))
		return 96;
	if((iterator > 96) && (iterator <= 112))
		return 112;
	if((iterator > 112) && (iterator <= 128))
		return 128;
	if((iterator > 128) && (iterator <= 144))
		return 144;
	if((iterator > 144) && (iterator <= 160))
		return 160;
	if((iterator > 160) && (iterator <= 176))
		return 176;
	if((iterator > 176) && (iterator <= 192))
		return 192;
	if((iterator > 192) && (iterator <= 208))
		return 208;
	if((iterator > 208) && (iterator <= 224))
		return 224;
	if((iterator > 224) && (iterator <= 240))
		return 240;
	if((iterator > 240) && (iterator <= 256))
		return 256;
	if((iterator > 256) && (iterator <= 272))
		return 272;
	if((iterator > 272) && (iterator <= 288))
		return 288;
	return 0;
}

/*---------------------------------------------------------------------------*/

enum crypto
{
	encrypt = 0x01,
	decrypt = 0x00,
	interrupts_enabled = 0x01,
	interrupts_disabled = 0x00,
};

/*---------------------------------------------------------------------------*/

void aes_cbc_nonce_gen(uint32_t *nonce)
{
	for (uint8_t i=0; i < 8; i++ )
		((uint16_t *)nonce )[i] = random_rand();
}

/*---------------------------------------------------------------------------*/

void periph_crypto_run()
{
	ti_lib_int_master_disable();
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_load_set();
	while(!ti_lib_prcm_load_get());
	ti_lib_int_master_enable();
}

/*---------------------------------------------------------------------------*/

void periph_crypto_stop()
{
	ti_lib_int_master_disable();
	ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_peripheral_sleep_disable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_peripheral_deep_sleep_disable(PRCM_PERIPH_CRYPTO);
	ti_lib_prcm_load_set();
	while(!ti_lib_prcm_load_get());
	ti_lib_int_master_enable();
}

/*---------------------------------------------------------------------------*/


void aes_cbc_encrypt(uint32_t *aes_key, uint32_t *nonce, uint32_t *input_data, uint32_t *output_data, uint32_t data_lenth)
{
	uint8_t key_index = CRYPTO_KEY_AREA_0;
	//aes_cbc_nonce_gen(nonce);

	periph_crypto_run();
	ti_lib_crypto_aes_load_key(aes_key, key_index);
	ti_lib_crypto_aes_cbc(input_data, output_data, data_lenth, nonce, key_index, encrypt, interrupts_disabled);
	while (ti_lib_crypto_aes_cbc_status() != AES_SUCCESS);
	ti_lib_crypto_aes_cbc_finish();
	periph_crypto_stop();
}

/*---------------------------------------------------------------------------*/

void aes_cbc_decrypt(uint32_t *aes_key, uint32_t *nonce, uint32_t *input_data, uint32_t *output_data, uint32_t data_lenth)
{
	uint8_t key_index = CRYPTO_KEY_AREA_0;

	periph_crypto_run();
	ti_lib_crypto_aes_load_key(aes_key, key_index);
	ti_lib_crypto_aes_cbc(input_data, output_data, data_lenth, nonce, key_index, decrypt, interrupts_disabled);
	while (ti_lib_crypto_aes_cbc_status() != AES_SUCCESS);
	ti_lib_crypto_aes_cbc_finish();
	periph_crypto_stop();
}

/*---------------------------------------------------------------------------*/


void aes_ecb_encrypt(uint32_t *aes_key, uint32_t *input_data, uint32_t *output_data)
{
	uint8_t key_index = CRYPTO_KEY_AREA_0;

	periph_crypto_run();
	ti_lib_crypto_aes_load_key(aes_key, key_index);
	ti_lib_crypto_aes_ecb(input_data, output_data, key_index, encrypt, interrupts_disabled);
	while (ti_lib_crypto_aes_ecb_status() != AES_SUCCESS);
	ti_lib_crypto_aes_ecb_finish();
	periph_crypto_stop();
}

/*---------------------------------------------------------------------------*/

void aes_ecb_decrypt(uint32_t *aes_key, uint32_t *input_data, uint32_t *output_data)
{
	uint8_t key_index = CRYPTO_KEY_AREA_0;

	periph_crypto_run();
	ti_lib_crypto_aes_load_key(aes_key, key_index);
	ti_lib_crypto_aes_ecb(input_data, output_data, key_index, decrypt, interrupts_disabled);
	while (ti_lib_crypto_aes_ecb_status() != AES_SUCCESS);
	ti_lib_crypto_aes_ecb_finish();
	periph_crypto_stop();
}

/*---------------------------------------------------------------------------*/

