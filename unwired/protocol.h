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
*         Protocol
* \author
*         Manchenko Oleg man4enkoos@gmail.com
*/
/*---------------------------------------------------------------------------*/

#define HEADER_LENGTH 		8
#define JOIN_STAGE_1_LENGTH 4
#define JOIN_STAGE_2_LENGTH 2
#define JOIN_STAGE_3_LENGTH 6
#define JOIN_STAGE_4_LENGTH 16





/*---------------------------------------------------------------------------*/

// typedef union u8_u16_t
// {
	// uint16_t u16;
	// uint8_t u8[2];
// } u8_u16_t;

// typedef union u8_u32_t
// {
	// uint32_t u32;
	// uint8_t u8[4];
// } u8_u32_t;

/*---------------------------------------------------------------------------*/

typedef struct {		
	uint8_t ciphertext[16];
} crypto_1_block_t;

typedef struct {		
	uint8_t ciphertext[32];
} crypto_2_block_t;

typedef struct {		
	uint8_t ciphertext[48];
} crypto_3_block_t;

typedef struct {		
	uint8_t ciphertext[64];
} crypto_4_block_t;

typedef struct {		
	uint8_t ciphertext[80];
} crypto_5_block_t;

typedef struct {		
	uint8_t ciphertext[96];
} crypto_6_block_t;

typedef struct {		
	uint8_t ciphertext[112];
} crypto_7_block_t;

/*---------------------------------------------------------------------------*/

typedef struct {
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
} header_up_t;

typedef struct {		
    u8_u16_t counter;
	uint8_t length;
} header_down_t;

typedef struct {		
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
	u8_u16_t counter;
	uint8_t length;
} header_t;

typedef struct {		
	u8_u32_t serial;
} join_stage_1_t;

typedef struct {		
	u8_u16_t nonce;
} join_stage_2_t;

typedef struct {		
	u8_u16_t serial;
	u8_u16_t nonce;
} join_stage_3_t;

typedef struct {		
	uint8_t array_of_zeros[16];
} join_stage_4_t;



/*---------------------------------------------------------------------------*/












