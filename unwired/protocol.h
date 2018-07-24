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
#include "net/ip/uip.h"
/*---------------------------------------------------------------------------*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define pack 		__attribute__((packed))
/*---------------------------------------------------------------------------*/
/**/
typedef union u8_u16_t {
	uint16_t u16;
	uint8_t u8[2];
} u8_u16_t;

typedef union u8_i16_t {
	int16_t i16;
	uint8_t u8[2];
} u8_i16_t;

typedef union u8_u32_t {
	uint32_t u32;
	uint8_t u8[4];
} u8_u32_t;

/*---------------------------------------------------------------------------*/
/**/
typedef enum 
{
	UNWDS_GPIO_MODULE_ID = 1,
	UNWDS_4BTN_MODULE_ID = 2,
	UNWDS_GPS_MODULE_ID = 3,
	UNWDS_LSM6DS3_MODULE_ID = 4,
	UNWDS_LM75_MODULE_ID = 5,
	UNWDS_LMT01_MODULE_ID = 6,
	UNWDS_UART_MODULE_ID = 7,
	UNWDS_SHT21_MODULE_ID = 8,
	UNWDS_PIR_MODULE_ID = 9,
	UNWDS_ADC_MODULE_ID = 10,
	UNWDS_LPS331_MODULE_ID = 11,
	UNWDS_COUNTER_MODULE_ID = 12,
	UNWDS_RSSIECHO_MODULE_ID = 13,
	UNWDS_6FET_MODULE_ID = 14,
	UNWDS_LIT_MODULE_ID = 15,
	UNWDS_DALI_MODULE_ID = 16,
	UNWDS_BME280_MODULE_ID = 17,
	UNWDS_MHZ19_MODULE_ID = 18,
	UNWDS_RANGE_MODULE_ID = 19,
	UNWDS_ADXL345_MODULE_ID = 20,
	/* Proprietary 50 to 99 */
	UNWDS_M200_MODULE_ID = 50,
	UNWDS_PULSE_MODULE_ID = 51,
	UNWDS_IBUTTON_MODULE_ID = 52,
	UNWDS_SWITCH_MODULE_ID = 53,
	UNWDS_M230_MODULE_ID = 54,
	UNWDS_IEC61107_MODULE_ID = 55,
	/* Customer 100 to 125*/
	UNWDS_CUSTOMER_MODULE_ID = 100,
	UNWDS_FIREBUTTON_MODULE_ID = 101,
	/* System module 126 */
	UNWDS_CONFIG_MODULE_ID = 126,
	UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 127,
} UNWDS_MODULE_IDS_t;

/*---------------------------------------------------------------------------*/
/*CR->ROOT*/
//0xFD 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x02 0x12 0x4B 0x00 0x0C 0x46 0x8D 0x03 0x0F 0x26 0x00 //test lum
typedef struct {		
	uip_ip6addr_t dest_addr; 
	uint8_t device_id; 
	uint8_t data_type; 
	uint8_t payload_len;
} pack uart_header_t;

/*---------------------------------------------------------------------------*/
/*HEADER*/
typedef struct {
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
} pack header_up_t;

typedef struct {		
    u8_u16_t counter;
	u8_u16_t crc;
	uint8_t length;
} pack header_down_t;

typedef struct {		
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
	u8_u16_t counter;
	u8_u16_t crc;
	uint8_t length;
} pack header_t; 

/*---------------------------------------------------------------------------*/

typedef struct {		
	uint8_t ciphertext[16];
} pack crypto_1_block_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-4BTN*/

/*Command*/
#define BUTTON_STATUS		0x00 /*Пакет статусом нажатой кнопки*/

/*Struct*/
typedef struct {		
	uint8_t button_status;
} pack button_status_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-6FET*/

/*Command*/
#define PWM_SETTINGS		0x00 /*Пакет с настройкоами ШИМ канала*/

/*Struct*/
typedef struct {
	uint32_t frequency;
	uint8_t channel;
	uint8_t duty;
} pack pwm_settings_t;

/*-----------------------------------*/
/*Command*/
#define PWM_POWER			0x01 /*Команда включения/выключения канала ШИМ'а*/

/*Struct*/
typedef struct {		
	uint8_t pwm_power;
} pack pwm_power_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-LIT*/

/*Command*/
#define LIT_MEASURE			0x00 /*Команда замера освещенности*/

/*Struct*/
typedef struct {		
	uint32_t lit_measure;
} pack lit_measure_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-6LOWPAN_SYSTEM*/

/*Command*/
#define DATA_TYPE_JOIN_STAGE_1		0x00 /*Нода посылает запрос координатору*/

/*Struct*/
typedef struct {		
	uint8_t module_id;
} pack join_stage_1_t;

/*-----------------------------------*/
/*Command*/
#define DATA_TYPE_JOIN_STAGE_2		0x01 /*Координатор отправляет ecb_encrypt(nonce=rand())*/

/*Struct*/
typedef struct {		
	u8_u16_t nonce;
} pack join_stage_2_t;

/*-----------------------------------*/
/*Command*/
#define DATA_TYPE_JOIN_STAGE_3		0x02 /*Нода удостоверяет, что она знает ключ отправляя cbc_encrypt(nonce)*/

/*Struct*/
typedef struct {		
	u8_u16_t nonce;
} pack join_stage_3_t;


/*-----------------------------------*/
/*Command*/
#define DATA_TYPE_JOIN_STAGE_4		0x03 /*Координатор отвечает ноде что она имеет право быть в сети*/

/*Struct*/
typedef struct {		
	uint8_t status_code;
} pack join_stage_4_t;

/*-----------------------------------*/
/*Command*/
#define PING						0x04 /*Ping*/

/*Struct*/
typedef struct {		
	uint8_t array_of_zeros[16];
} pack ping_t;

/*-----------------------------------*/
/*Command*/
#define PONG						0x05 /*Pong*/

/*Struct*/
typedef struct {		
	uint8_t status_code;
} pack pong_t;

/*---------------------------------------------------------------------------*/
#define UDP_DATA_PORT					4004

#define UDBP_PROTOCOL_VERSION			1

#define STATUS_CODE_LENGTH				1
#define HEADER_UP_LENGTH				sizeof(header_up_t)
#define HEADER_DOWN_LENGTH				sizeof(header_down_t)

#define HEADER_OFFSET 					0
#define HEADER_DOWN_OFFSET 				HEADER_UP_LENGTH
#define PAYLOAD_OFFSET 					HEADER_UP_LENGTH + HEADER_DOWN_LENGTH

#define CRYPTO_1_BLOCK_LENGTH 			sizeof(crypto_1_block_t)

#define JOIN_STAGE_1_PAYLOAD_LENGTH 	JOIN_STAGE_1_LENGTH
#define JOIN_STAGE_2_PAYLOAD_LENGTH 	CRYPTO_1_BLOCK_LENGTH
#define JOIN_STAGE_3_PAYLOAD_LENGTH 	CRYPTO_1_BLOCK_LENGTH
#define JOIN_STAGE_4_PAYLOAD_LENGTH 	CRYPTO_1_BLOCK_LENGTH
#define PING_PAYLOAD_LENGTH 			CRYPTO_1_BLOCK_LENGTH
#define PONG_PAYLOAD_LENGTH 			CRYPTO_1_BLOCK_LENGTH
#define BUTTON_STATUS_PAYLOAD_LENGTH	CRYPTO_1_BLOCK_LENGTH
#define PWM_SETTINGS_PAYLOAD_LENGTH		CRYPTO_1_BLOCK_LENGTH
#define PWM_POWER_PAYLOAD_LENGTH		CRYPTO_1_BLOCK_LENGTH
#define LIT_MEASURE_PAYLOAD_LENGTH		CRYPTO_1_BLOCK_LENGTH

#define HEADER_LENGTH 					HEADER_UP_LENGTH + HEADER_DOWN_LENGTH
#define JOIN_STAGE_1_LENGTH 			sizeof(join_stage_1_t)
#define JOIN_STAGE_2_LENGTH 			sizeof(join_stage_2_t)
#define JOIN_STAGE_3_LENGTH 			sizeof(join_stage_3_t)
#define JOIN_STAGE_4_LENGTH 			sizeof(join_stage_4_t)
#define PING_LENGTH 					sizeof(ping_t)
#define PONG_LENGTH 					sizeof(pong_t)
#define BUTTON_STATUS_LENGTH 			sizeof(button_status_t)
#define PWM_SETTINGS_LENGTH 			sizeof(pwm_settings_t)
#define PWM_POWER_LENGTH 				sizeof(pwm_power_t)
#define LIT_MEASURE_LENGTH				sizeof(lit_measure_t)

#define OFFSET_0_BYTE 					0
#define OFFSET_1_BYTE 					1
#define OFFSET_2_BYTE 					2
#define OFFSET_3_BYTE 					3
#define OFFSET_4_BYTE 					4
#define OFFSET_5_BYTE 					5
#define OFFSET_6_BYTE 					6
#define OFFSET_7_BYTE 					7
#define OFFSET_8_BYTE 					8
#define OFFSET_9_BYTE 					9

#define STATUS_OK	 					0x01
#define STATUS_ERROR	 				0x00

/*---------------------------------------------------------------------------*/
#endif
/*---------------------------------------------------------------------------*/




