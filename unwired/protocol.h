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
#define UDP_DATA_PORT				4004

#define UDBP_PROTOCOL_VERSION		1

#define HEADER_OFFSET 				0
#define HEADER_DOWN_OFFSET 			6
#define PAYLOAD_OFFSET 				9
#define HEADER_DOWN_LENGTH_OFFSET 	2

#define SERIAL_LENGTH 				4
#define NONCE_LENGTH				2
#define STATUS_CODE_LENGTH			1
#define ARRAY_OF_ZEROS_LENGTH		16
#define HEADER_UP_LENGTH			6
#define HEADER_DOWN_LENGTH			3

#define CRYPTO_1_BLOCK_LENGTH 		16	
#define CRYPTO_2_BLOCK_LENGTH 		32
#define CRYPTO_3_BLOCK_LENGTH 		48
#define CRYPTO_4_BLOCK_LENGTH 		64
#define CRYPTO_5_BLOCK_LENGTH 		80
#define CRYPTO_6_BLOCK_LENGTH 		96
#define CRYPTO_7_BLOCK_LENGTH 		112

#define JOIN_STAGE_1_PAYLOAD_LENGTH SERIAL_LENGTH
#define JOIN_STAGE_2_PAYLOAD_LENGTH CRYPTO_1_BLOCK_LENGTH
#define JOIN_STAGE_3_PAYLOAD_LENGTH SERIAL_LENGTH + CRYPTO_1_BLOCK_LENGTH
#define JOIN_STAGE_4_PAYLOAD_LENGTH CRYPTO_1_BLOCK_LENGTH
#define PING_PAYLOAD_LENGTH 		CRYPTO_1_BLOCK_LENGTH
#define PONG_PAYLOAD_LENGTH 		STATUS_CODE_LENGTH + CRYPTO_1_BLOCK_LENGTH

#define HEADER_LENGTH 				9
#define JOIN_STAGE_1_LENGTH 		SERIAL_LENGTH
#define JOIN_STAGE_2_LENGTH 		NONCE_LENGTH
#define JOIN_STAGE_3_LENGTH 		SERIAL_LENGTH + NONCE_LENGTH
#define JOIN_STAGE_4_LENGTH 		ARRAY_OF_ZEROS_LENGTH
#define PING_LENGTH 				ARRAY_OF_ZEROS_LENGTH
#define PONG_LENGTH 				STATUS_CODE_LENGTH + ARRAY_OF_ZEROS_LENGTH

#define OFFSET_0_BYTE 				0
#define OFFSET_1_BYTE 				1
#define OFFSET_2_BYTE 				2
#define OFFSET_3_BYTE 				3
#define OFFSET_4_BYTE 				4
#define OFFSET_5_BYTE 				5
#define OFFSET_6_BYTE 				6
#define OFFSET_7_BYTE 				7
#define OFFSET_8_BYTE 				8
#define OFFSET_9_BYTE 				9

#define STATUS_OK	 				0x00
#define STATUS_ERROR	 			0x01

/*---------------------------------------------------------------------------*/
/* Data types */
// #define DATA_TYPE_RESERVED_1				0x01 //Не используется
// #define DATA_TYPE_SENSOR_DATA			0x02 //Данные с датчиков устройства
// #define DATA_TYPE_RESERVED_2				0x03 //Не используется
// #define DATA_TYPE_ACK					0x04 //Подтверждение доставки пакета
// #define DATA_TYPE_COMMAND				0x05 //Команды возможностям устройства
// #define DATA_TYPE_STATUS					0x06 //Пакет со статусными данными
// #define DATA_TYPE_GET_STATUS				0x07 //Запрос статуса(не реализовано)
// #define DATA_TYPE_SETTINGS				0x08 //Команда настройки параметров
// #define DATA_TYPE_MESSAGE				0x09 //Сообщения
// #define DATA_TYPE_SET_TIME				0x0A //Команда установки времени
// #define DATA_TYPE_SET_SCHEDULE			0x0B //Команда установки расписания(не реализовано)
// #define DATA_TYPE_FIRMWARE				0x0C //Данные для OTA
// #define DATA_TYPE_UART					0x0D //Команда с данными UART
// #define DATA_TYPE_FIRMWARE_CMD			0x0E //Команды OTA
// #define DATA_TYPE_RESERVED_3				0x0F //Не используется
#define DATA_TYPE_JOIN_STAGE_1				0x10 //Нода посылает запрос координатору
#define DATA_TYPE_JOIN_STAGE_2				0x11 //Координатор отправляет ecb_encrypt(nonce=rand())
#define DATA_TYPE_JOIN_STAGE_3				0x12 //Нода удостоверяет, что она знает ключ отправляя cbc_encrypt(nonce)
#define DATA_TYPE_JOIN_STAGE_4				0x13 //Координатор отвечает ноде что она имеет право быть в сети
#define PING								0x14 //Ping
#define PONG								0x15 //Pong
#define UART_FROM_AIR_TO_TX					0x20 //Пакет с UART
#define UART_FROM_RX_TO_AIR					0x21 //Пакет с UART

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
	u8_u32_t serial;
	crypto_1_block_t crypto_1_block;
} join_stage_3_t;

typedef struct {		
	uint8_t array_of_zeros[16];
} join_stage_4_t;

typedef struct {		
	uint8_t array_of_zeros[16];
} ping_t;

typedef struct {		
	uint8_t status_code;
	uint8_t array_of_zeros[16];
} pong_t;

/*---------------------------------------------------------------------------*/












