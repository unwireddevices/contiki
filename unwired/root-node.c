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
*         RPL-root service for Unwired Devices mesh 
* \author
*         Manchenko Oleg man4enkoos@gmail.com
*/
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"

#include "dev/leds.h"
#include "cc26xx/board.h"

#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ip/uip-debug.h"
#include "simple-udp.h"
#include "net/rpl/rpl.h"

#include <stdio.h>
#include <string.h>

#include "button-sensor.h"
#include "board-peripherals.h"
#include "system-common.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"

#include "protocol.h"
#include "xxf_types_helper.h"
#include "dev/watchdog.h"
#include "root-node.h"
#include "crypto-common.h"
#include "rtc-common.h"
#include "int-flash-common.h"

#include "dev/serial-line.h"
#include "../cpu/cc26xx-cc13xx/dev/cc26xx-uart.h"
#include "../../core/dev/serial-line.h"

#include "uart/root.h"

#define DIO_MASK				0x3F
#define STATUS_MASK				0xC0
#define CLICK					0x80
#define LONG_CLICK				0xC0

#define IOC_OUTPUT_PULL_UP	(IOC_CURRENT_2MA	| IOC_STRENGTH_AUTO	| \
							IOC_IOPULL_UP		| IOC_SLEW_DISABLE	| \
							IOC_HYST_DISABLE	| IOC_NO_EDGE		| \
							IOC_INT_DISABLE		| IOC_IOMODE_NORMAL	| \
							IOC_NO_WAKE_UP		| IOC_INPUT_DISABLE	)	

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

#define WAIT_RESPONSE 			3 	//Максимальное время ожидания ответа от счетчика в секундах

/*---------------------------------------------------------------------------*/
/*ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ*/

uint8_t aes_key[16];					/*Ключ шифрования*/
static uint8_t nonce_key[16];			/*Сессионный ключ*/

eeprom_t eeprom_settings;				/*Настройки из EEPROM*/

static uip_ipaddr_t null_addr;			/*Сравниваем с этим адресом (аналог nullptr)*/

/*Счетчик пакетов*/
static volatile union 
{ 
	uint16_t u16; 
	uint8_t u8[2]; 
} packet_counter_root;

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ФУНКЦИЙ*/

/*Вторая стадия авторизации*/
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

/*Четвертая стадия авторизации*/
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

/*Pong*/
static void pong_sender(const uip_ip6addr_t *dest_addr, ping_t *ping_pack);

/*Обработчик пакета с нажатой кнопкой*/
static void button_status_handler(const uip_ip6addr_t *dest_addr, button_status_t *button_status_pack);

/*Обработчик пакета с измерением освещенности*/
static void lit_measure_handler(const uip_ip6addr_t *dest_addr, lit_measure_t *lit_measure_pack);

/**/
static void send_pack_from_cr(uint8_t* data);

/**/
static void print_cr(const uip_ip6addr_t *dest_addr, uint8_t* data, uint8_t length);


/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ПРОЦЕССОВ*/

/*Процесс инициализации настроек из EEPROM*/
PROCESS(settings_root_init, "Initializing settings of ROOT");

/*Процесс управления ROOT'ом*/
PROCESS(main_root_process, "main root process");

/*---------------------------------------------------------------------------*/
/*Обработчик принятых пакетов*/
void udp_data_receiver(struct simple_udp_connection *connection,
                       const uip_ipaddr_t *sender_addr, //
                       uint16_t sender_port,
                       const uip_ipaddr_t *receiver_addr, //
                       uint16_t receiver_port,
                       const uint8_t *data, //
                       uint16_t datalen)
{
	led_on(LED_A); 								/*Включаем светодиод*/
	
	/*Отражаем структуру на массив*/ 
	header_up_t *header_up_pack = (header_up_t*)&data[HEADER_OFFSET];
	
	/*Для отладки. Выводит тип принятого сообщения*/ 
	// printf("Recive pack: %x\n", header_up_pack->data_type);
	
	/*Вывод информационного сообщения в консоль*/
	// printf("Packet crypto received(%"PRIu8"): ", datalen);
	// for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
		// printf("%"PRIXX8, data[i]);
	// printf("\n");
	

	/*Проверяем версию протокола*/ 
	if(header_up_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{
		/*Проверяем ID модуля и тип пакета*/ 
		if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == DATA_TYPE_JOIN_STAGE_1))
		{
			/*Вторая стадия авторизации*/		
			/*Выдача сообщения CR*/			
			join_stage_2_sender((uip_ip6addr_t*)sender_addr, data, datalen);
			led_off(LED_A);		/*Выключаем светодиод*/
			return;
		}
		
		else if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == DATA_TYPE_JOIN_STAGE_3))
		{
			/*Четвертая стадия авторизации*/
			join_stage_4_sender((uip_ip6addr_t*)sender_addr, data, datalen);
			led_off(LED_A);		/*Выключаем светодиод*/
			return;
		}
		
		else
		{
			/*Получаем nonce*/
			u8_u16_t nonce;
			nonce.u16 = get_nonce((uip_ip6addr_t*)sender_addr);	
			
			/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
			for(int i = 0; i < 16; i += 2)
			{
				nonce_key[i] = nonce.u8[1];	
				nonce_key[i+1] = nonce.u8[0];	
			}
			
			/*Расшифровываем данные*/
			aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], iterator_to_byte(datalen - HEADER_UP_LENGTH));
			
			/*Вывод информационного сообщения в консоль*/
			// printf("Packet no crypto received(%"PRIu8"): ", datalen);
			// for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
				// printf("%"PRIXX8, data[i]);
			// printf("\n");
			
			/*Отражаем структуру на массив*/ 
			header_down_t *header_down_pack = (header_down_t*)&data[HEADER_DOWN_OFFSET];
						
			/*CRC16 проверка*/ 
			if(header_down_pack->crc.u16 != crc16_arc((uint8_t*)&data[PAYLOAD_OFFSET], header_down_pack->length))
			{
				printf("CRC16 error!\n");
				led_off(LED_A);
				return;
			}
			
			/*Защита от атаки повтором*/
			if(!valid_counter((uip_ip6addr_t*)sender_addr, header_down_pack->counter.u16))
			{
				printf("Counter error!\n");
				led_off(LED_A);
				return;
			}
			
			/*Выдача сообщения CR*/
			print_cr((uip_ip6addr_t*)sender_addr, (uint8_t*)data, (HEADER_LENGTH + header_down_pack->length));
				
			/*Проверяем ID модуля*/ 
			if(header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
			{
				if(header_up_pack->data_type == PING)
				{
					/*Pong*/
					pong_sender((uip_ip6addr_t*)sender_addr, (ping_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else
				{	
					printf("Unknown command for system!\n");
					led_off(LED_A);
					return;
				}
			}
			
			if(header_up_pack->device_id == UNWDS_4BTN_MODULE_ID)
			{
				if(header_up_pack->data_type == BUTTON_STATUS)
				{
					button_status_handler((uip_ip6addr_t*)sender_addr, (button_status_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else
				{	
					printf("Unknown command for UMDK-4BTN!\n");
					led_off(LED_A);
					return;
				}
			}
		
			else if(header_up_pack->device_id == UNWDS_LIT_MODULE_ID)
			{
				if(header_up_pack->data_type == LIT_MEASURE)
				{
					lit_measure_handler((uip_ip6addr_t*)sender_addr, (lit_measure_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else
				{	
					printf("Unknown command for UMDK-LIT!\n");
					led_off(LED_A);
					return;
				}
			}
			
			else
			{	
				printf("Unknown module!\n");
				led_off(LED_A);
				return;
			}
		}	
	}
	/*Выключаем светодиод*/
	led_off(LED_A);
}	

/*---------------------------------------------------------------------------*/
/*Вторая стадия авторизации*/
/*Генерирует сессионный ключ (nonce) и отправляет его зашифрованым AES128-ECB, добавляет маршрут в таблицу*/
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{	
	/*Выдача сообщения CR*/
	print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_1_LENGTH));
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = DATA_TYPE_JOIN_STAGE_2;			/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_root.u16;			/*Счетчик пакетов*/ 
	header_pack->length = JOIN_STAGE_2_LENGTH;					/*Размер пакета (незашифрованного)*/
	
	/*Payload*/ 
	join_stage_2_pack->nonce.u16 = random_rand();				/*Генерируем сессионный ключ*/ 
	
	/*Добавляем маршрут*/ 
	add_route ( (uip_ip6addr_t*)dest_addr,						/*Address*/ 
				join_stage_2_pack->nonce.u16);					/*Nonce*/ 
	
	/*Дозаполняем блок для шифрования нулями*/ 
	for(uint8_t i = JOIN_STAGE_2_LENGTH; i < (JOIN_STAGE_2_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/*CRC16*/ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&join_stage_2_pack, sizeof(join_stage_2_pack));
	
	/*Зашифровываем блок*/ 
	aes_ecb_encrypt((uint32_t*)aes_key, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]));
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH), dest_addr);
	packet_counter_root.u16++;	/*Инкрементируем счетчик пакетов*/ 
}

/*---------------------------------------------------------------------------*/
/*Четвертая стадия авторизации*/
/*Принимает nonce зашифрованный AES128-CBC. Если сходится с тем что он сгенерировал, то авторизация прошла успешно, настройки шифрования верные. Отправляем пакет с нулями что бы DAG мог убедиться в этом*/
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{	
	/*Получаем nonce*/
	u8_u16_t nonce;
	nonce.u16 = get_nonce((uip_ip6addr_t*)dest_addr);	
	
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = nonce.u8[1];	
		nonce_key[i+1] = nonce.u8[0];	
	}

	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);
	
	/*Выдача сообщения CR*/
	print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_3_LENGTH));
	
	/*Отражаем структуры на массивы*/ 
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&data[PAYLOAD_OFFSET];
	
	/*Заполняем payload*/
	join_stage_4_t join_stage_4_pack;				/*Создаем структуру*/
	
	/*Проверяем одинаковые ли у нас настройки шифрования*/ 
	if((get_nonce((uip_ip6addr_t*)dest_addr) + 1) != join_stage_3_pack->nonce.u16)
	{
		join_stage_4_pack.status_code = false;
		
		/*Вывод сообщения об не успешной авторизации*/
		printf("[ROOT Node] Authorization error node: ");
		uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
		printf("\n");
	}
	
	/*Если nonce'ы совпадают, то авторизация прошла успешно, шифрование настроенно правильно*/ 
	else
	{
		join_stage_4_pack.status_code = true;		/*Статус код*/
		unlock_addr((uip_ip6addr_t*)dest_addr);		/*Разрешаем обрабатывать пакеты принятые с авторизированного устройства*/
		
		/*Вывод сообщения об успешной авторизации*/
		printf("[ROOT Node] Authorized node: ");
		uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
		printf("\n");
		
	}
			
	/*Отправляем пакет*/
	pack_sender(dest_addr, 						/*Адрес модуля UMDK-6FET*/
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID, /*Индентификатор модуля*/
				DATA_TYPE_JOIN_STAGE_4, 		/*Команда 4 стадии авторизации*/
				(uint8_t*)&join_stage_4_pack, 	/*Payload*/
				sizeof(join_stage_4_pack));		/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Pong*/
static void pong_sender(const uip_ip6addr_t *dest_addr, ping_t *ping_pack)
{
	/*Заполняем payload*/
	pong_t pong_pack;							/*Создаем структуру*/
	
	pong_pack.status_code = STATUS_OK;			/*Статус код*/
	
	/*Проверяем массив. Если все нули, то настройки шифрования верны*/ 
	if(ping_pack->nonce.u16 == get_nonce((uip_ip6addr_t*)dest_addr))
	{
		pong_pack.status_code = STATUS_OK;
	}
	
	else
	{
		pong_pack.status_code = STATUS_ERROR;
	}
		
	/*Отправляем пакет*/
	pack_sender(dest_addr, 						/*Адрес модуля UMDK-6FET*/
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID, /*Индентификатор модуля*/
				PONG, 							/*Команда ответа на PING*/
				(uint8_t*)&pong_pack, 			/*Payload*/
				sizeof(pong_pack));				/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Обработчик нажатой кнопки*/
static void button_status_handler(const uip_ip6addr_t *dest_addr, button_status_t *button_status_pack)
{
	uint8_t status = button_status_pack->button_status & STATUS_MASK;
	uint8_t dio = button_status_pack->button_status & DIO_MASK;

	/*Вывод сообщения об успешной авторизации*/
	printf("[");
	uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
	printf("] ");
	
	if((dio == BOARD_IOID_KEY_A) && (status == CLICK))
		printf("Button A click\n");
	else if((dio == BOARD_IOID_KEY_A) && (status == LONG_CLICK))
		printf("Button A long click\n");
	
	else if((dio == BOARD_IOID_KEY_B) && (status == CLICK))
		printf("Button B click\n");
	else if((dio == BOARD_IOID_KEY_B) && (status == LONG_CLICK))
		printf("Button B long click\n");
	
	else if((dio == BOARD_IOID_KEY_C) && (status == CLICK))
		printf("Button C click\n");
	else if((dio == BOARD_IOID_KEY_C) && (status == LONG_CLICK))
		printf("Button C long click\n");
	
	else if((dio == BOARD_IOID_KEY_D) && (status == CLICK))
		printf("Button D click\n");
	else if((dio == BOARD_IOID_KEY_D) && (status == LONG_CLICK))
		printf("Button D long click\n");
	
	else if((dio == BOARD_IOID_KEY_E) && (status == CLICK))
		printf("Button E click\n");
	else if((dio == BOARD_IOID_KEY_E) && (status == LONG_CLICK))
		printf("Button E long click\n");
	
	else
		printf("Button error\n");
}

/*---------------------------------------------------------------------------*/
/*Обработчик пакета с измерением освещенности*/
static void lit_measure_handler(const uip_ip6addr_t *dest_addr, lit_measure_t *lit_measure_pack)
{
	/*Вывод сообщения об успешной авторизации*/
	printf("[");
	uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
	printf("] Luminocity: %lu lux\n", lit_measure_pack->lit_measure);
}

/*---------------------------------------------------------------------------*/
/*Отправка настроек канала ШИМ'а*/
void pwm_settings_sender(const uip_ip6addr_t *dest_addr, uint8_t channel, uint32_t frequency, uint8_t duty)
{	
	/*Заполняем payload*/
	pwm_settings_t pwm_settings_pack;				/*Создаем структуру*/
	
	pwm_settings_pack.channel = channel;			/*Номер канала*/
	pwm_settings_pack.frequency = frequency;		/*Частота*/
	pwm_settings_pack.duty = duty;					/*Коэффицент заполненния*/
			
	/*Отправляем пакет*/
	pack_sender(dest_addr, 							/*Адрес модуля UMDK-6FET*/
				UNWDS_6FET_MODULE_ID, 				/*Индентификатор модуля UMDK-6FET*/
				PWM_SETTINGS, 						/*Команда настройки канала ШИМ'а*/
				(uint8_t*)&pwm_settings_pack, 		/*Payload*/
				sizeof(pwm_settings_pack));			/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Отправка команды включения/выключения канала ШИМ'а*/
void pwm_power_channel_sender(const uip_ip6addr_t *dest_addr, uint8_t channel, uint8_t pwm_power_channel)
{
	/*Заполняем payload*/
	pwm_power_t pwm_power_pack;					/*Создаем структуру*/
	
	pwm_power_pack.pwm_power = channel;			/*Номер канала*/
	
	if(pwm_power_channel)						/*Если включить, то устанавливаем старший бит в единицу*/
		pwm_power_pack.pwm_power |= 0x80;			
			
	/*Отправляем пакет*/
	pack_sender(dest_addr, 						/*Адрес модуля UMDK-6FET*/
				UNWDS_6FET_MODULE_ID, 			/*Индентификатор модуля UMDK-6FET*/
				PWM_POWER, 						/*Команда включения канала ШИМ'а*/
				(uint8_t*)&pwm_power_pack, 		/*Payload*/
				sizeof(pwm_power_pack));		/*Размер payload'а*/
}
/*---------------------------------------------------------------------------*/
/*Совершить замер освещенности*/
void lit_measurement_sender(const uip_ip6addr_t *dest_addr)
{
	/*Отправляем пакет c запросом освещенности*/
	pack_sender(dest_addr, 				/*Адрес модуля UMDK-6FET*/
				UNWDS_LIT_MODULE_ID, 	/*Индентификатор модуля UMDK-LIT*/
				LIT_MEASURE, 			/*Команда измерения освещенности*/
				NULL, 					/*Payload'а нет*/
				0);						/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Конструктор пакета*/
void pack_sender(const uip_ip6addr_t *dest_addr, 
				uint8_t device_id, 
				uint8_t data_type, 
				uint8_t *payload, 
				uint8_t payload_len)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	/*Проверка на то что передан не нулевой адрес буфера*/
	if ((payload == NULL) && (payload_len != 0))
		return;
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t crypto_length = iterator_to_byte(HEADER_DOWN_LENGTH + payload_len);
	uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	/*Получаем nonce*/
	u8_u16_t nonce;
	nonce.u16 = get_nonce((uip_ip6addr_t*)dest_addr);	
	
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = nonce.u8[1];	
		nonce_key[i+1] = nonce.u8[0];	
	}
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = device_id;							/*ID устройства*/
	header_pack->data_type = data_type;							/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_root.u16;			/*Счетчик пакетов*/ 
	header_pack->length = payload_len;							/*Размер пакета (незашифрованного)*/
	
	/*Payload*/ 	
	/*Pаполняем пакет, зашифровываем и отправляем его DAG'у. */ 
	for(uint8_t i = 0; i < (crypto_length - HEADER_DOWN_LENGTH); i++)
	{
		if(i < payload_len)
			udp_buffer[PAYLOAD_OFFSET + i] = payload[i];
		else
			udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	}
	
	/*CRC16*/ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], header_pack->length);
	
	/*Для отладки. Выводит содержимое пакета*/ 
	printf("Pack: \n");
	hexraw_print(16, (uint8_t*)dest_addr);
	hexraw_print(1, (uint8_t*)&(header_pack->device_id));
	hexraw_print(1, (uint8_t*)&(header_pack->data_type));
	hexraw_print(1, (uint8_t*)&(header_pack->length));	
	hexraw_print(payload_len, (uint8_t*)payload);
	printf("\n");
	
	// printf("Pack:\n");
	// hexraw_print((HEADER_UP_LENGTH + crypto_length), (uint8_t*)udp_buffer);
	// printf("\n");
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), dest_addr);
	packet_counter_root.u16++;		/*Инкрементируем счетчик пакетов*/
}

/*---------------------------------------------------------------------------*/
/**/
static void send_pack_from_cr(uint8_t* data)
{
	/*Отражаем структуры на массивы*/ 
	uart_header_t *uart_header_pack = (uart_header_t*)&data[1];
	
	hexraw_print(data[0], (uint8_t*)&data[1]);
	printf("\n");
	
	/*Отправляем пакет*/ 
	pack_sender(&(uart_header_pack->dest_addr), 
				uart_header_pack->device_id, 
				uart_header_pack->data_type, 
				(uint8_t*)&data[20], 
				uart_header_pack->payload_len);
}

/*---------------------------------------------------------------------------*/
/**/
static void print_cr(const uip_ip6addr_t *dest_addr, uint8_t* data, uint8_t length)
{
	/*Ожидаем завершения передачи*/
	while(ti_lib_uart_busy(UART0_BASE));

	/*Отсоединяем пин от UART'а*/ 
	ti_lib_gpio_set_dio(BOARD_IOID_UART_TX);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_UART_TX, GPIO_OUTPUT_ENABLE);
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_TX, IOC_PORT_GPIO, IOC_OUTPUT_PULL_UP);

	/*Присоединяем пин к UART'у*/ 
	ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);

	/*Выводим адрес отправителя*/
	for (uint8_t i = 0; i < sizeof(uip_ip6addr_t); i++)	
	{
		while(!ti_lib_uart_char_put_non_blocking(UART0_BASE, dest_addr->u8[i]));
	}
	
	/*Выводим принятый пакет*/
	for (uint8_t i = 0; i < length; i++)	
	{
		while(!ti_lib_uart_char_put_non_blocking(UART0_BASE, data[i]));
	}
	
	/*Ожидаем завершения передачи*/
	while(ti_lib_uart_busy(UART0_BASE));
	
	/*Отсоединяем пин от UART'а*/ 
	ti_lib_gpio_set_dio(BOARD_IOID_ALT_UART_TX);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_ALT_UART_TX, GPIO_OUTPUT_ENABLE);
	ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_TX, IOC_PORT_GPIO, IOC_OUTPUT_PULL_UP);

	/*Присоединяем пин к UART'у*/ 
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
}

/*---------------------------------------------------------------------------*/
/*Иннициализация RPL*/
void rpl_initialize()
{
	/*Устанавливаем режим работы MASH*/
	rpl_set_mode(RPL_MODE_MESH);

	static uip_ipaddr_t ipaddr;
	
	/*Инициализируем нулевой адрес*/
	uip_ip6addr(&null_addr, 0, 0, 0, 0, 0, 0, 0, 0);

	/* Fill in the address with zeros and the local prefix */
	uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

	/* Generate an address based on the chip ID */
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);

	/* Adding autoconfigured address as the device address */
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

	/* make local address as rpl-root */
	rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
	rpl_dag_t *dag = rpl_get_any_dag();

	uip_ipaddr_t prefix;
	uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
	rpl_set_prefix(dag, &prefix, 64);

	printf("[ROOT Node] Created a new RPL DAG, i'm root!\n");
}

/*---------------------------------------------------------------------------*/
/*Иннициализация ноды*/
void root_node_initialize()
{
	/*Устанавливаем обработчик входящих UDP данных*/
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_data_receiver);

	/*Мигнуть светодиодом*/
	led_blink(LED_A);
	led_blink(LED_A);

	/*Запускаем главный процес*/
	process_start(&main_root_process, NULL);
}

/*---------------------------------------------------------------------------*/
/*Процесс инициализации настроек из EEPROM*/
PROCESS_THREAD(settings_root_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	/*Считываем данные из EEPROM в структуру*/
	read_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
	
	/*Если настроек нет, то устанавливаем эти*/
	if(eeprom_settings.aes_key_configured == true)
	{
		if((eeprom_settings.channel != 26) && (eeprom_settings.panid != 0xAABB))
		{
			eeprom_settings.channel = 26;
			eeprom_settings.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
		}
		
		printf("AES-128 key not declared\n");
		while(eeprom_settings.aes_key_configured)
		{
			PROCESS_YIELD();
		}
	}
	
	/*Если ключа шифрования нет, то информируем об этом*/
	else
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_settings.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");		
	}
	
	radio_value_t channel;										/*Выделяем память под переменную channel*/
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);	/*Считываем в channel текущий канал*/
	
	/*Если текущий канал отличается от настроенного в EEPROM, изменяем канал на тот что из EEPROM*/
	if(channel != eeprom_settings.channel)
	{
		/*Устанавливаем канал*/
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_settings.channel);
		
		/*Если мы чип CC26XX, то выводим частоту*/
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_settings.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_settings.channel, freq_mhz);
		}

		/*Если мы чип CC13XX, то выводим частоту*/
		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_settings.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_settings.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;								/*Выделяем память под panid*/
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);	/*Считываем в PAN ID*/
		
		/*Если текущий PAN ID отличается от настроенного в EEPROM, изменяем PAN ID на тот что из EEPROM*/
		if(panid != eeprom_settings.panid)
		{
			/*Устанавливаем PAN ID*/
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_settings.panid);
			
			/*Выводим PAN ID*/
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_settings.panid);
		}
	}
	
	/*Передаем управление rpl_root_process*/
	process_post(&rpl_root_process, PROCESS_EVENT_CONTINUE, NULL);
	
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс управления ROOT'ом*/
PROCESS_THREAD(main_root_process, ev, data)
{
	PROCESS_BEGIN();

	packet_counter_root.u16 = 0;						/*Обнуляем счетчик пакетов*/
	PROCESS_PAUSE();									/*Небольшая задержка*/

	/*Цикл который ожидает события uart_event_message*/
	while(1) 							
	{
		PROCESS_YIELD(); 
		if(ev == uart_event_message) 
		{
			/**/ 
			printf("uart_event_message: ");
			send_pack_from_cr(data);
		} 
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
