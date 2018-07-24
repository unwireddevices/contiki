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
*         Header file for DAG-node service
* \author
*         Manchenko Oleg man4enkoos@gmail.com
*/
/*---------------------------------------------------------------------------*/
#define DPRINT printf(">dag_node.c:%"PRIu16"\n", __LINE__);watchdog_periodic();

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "clock.h"

#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ip/uip-debug.h"
#include "net/ip/simple-udp.h"
#include "net/link-stats.h"

#include "dev/leds.h"
#include "sys/clock.h"
#include "shell.h"
#include "serial-shell.h"
#include "dev/serial-line.h" //
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "radio_power.h"
#include "dev/watchdog.h"
#include "dev/cc26xx-uart.h"
#include "board-peripherals.h"
#include "board.h"
#include "ti-lib.h"

#include "ota-main.h"
#include "ota-common.h"
#include "crypto-common.h"
#include "rtc-common.h"
#include "system-common.h"
#include "int-flash-common.h"

#include "xxf_types_helper.h"
#include "protocol.h"

#include "dag_node.h"
#include "uart/uart.h"

#include "../cpu/cc26xx-cc13xx/dev/pwm.h" /*PWM*/
#include "../platform/unwired/udboards/opt3001.h" /*LIT*/

#define MAINTENANCE_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define SHORT_STATUS_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define LONG_STATUS_INTERVAL			(20 * 60 * CLOCK_SECOND)
#define ROOT_FIND_INTERVAL				(2 * CLOCK_SECOND)
#define ROOT_FIND_LIMIT_TIME			(2 * 60 * CLOCK_SECOND)

#define FALSE							0x00
#define TRUE							0x01

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/
/*ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ*/

simple_udp_connection_t udp_connection;	/*Структура UDP подключения*/

volatile uint8_t led_mode;

volatile uint8_t non_answered_ping = 0;	/*Количество неотвеченых пингов*/

static struct etimer maintenance_timer;	/*Таймер*/

/*Счетчик пакетов*/
static volatile union 
{ 
	uint16_t u16;
	uint8_t u8[2];
} packet_counter_root;					

uint8_t aes_key[16];					/*Ключ шифрования для ECB*/
static uint8_t nonce_key[16];			/*Сессионный ключ*/

eeprom_t eeprom_settings;				/*Структура с значениями из EEPROM*/

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ФУНКЦИЙ*/

/*Обработчик принятых пакетов*/
static void udp_receiver(struct simple_udp_connection *c,
						const uip_ipaddr_t *sender_addr,
						uint16_t sender_port,
						const uip_ipaddr_t *receiver_addr,
						uint16_t receiver_port,
						const uint8_t *data, 
						uint16_t datalen);
						
/*Первая стадия авторизации*/
static void join_stage_1_sender(const uip_ipaddr_t *dest_addr);

/*Третья стадия авторизации*/
static void join_stage_3_sender(const uip_ipaddr_t *dest_addr,
								const uint8_t *data,
								uint16_t datalen);

/*Обработчик четвертой стадии авторизации*/
static void join_stage_4_handler(const uip_ipaddr_t *sender_addr,
								const uint8_t *data,
								uint16_t datalen);

/*Ping*/
static void ping_sender(void);

/*Pong*/
static void pong_handler(const uip_ipaddr_t *sender_addr,
						pong_t *pong_pack);
								
/*Инициализация с заданными настройками канала ШИМ'а*/
static void dag_pwm_settings(const uip_ipaddr_t *sender_addr,
							pwm_settings_t *pwm_settings_pack);

/*Включение/выключение канала ШИМ'а*/
static void dag_pwm_power (	const uip_ipaddr_t *sender_addr,
							pwm_power_t *pwm_power_pack);
							
/*Совершить замер освещенности*/
static void dag_lit_measure_sender();

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ПРОЦЕССОВ*/

/*Процесс опроса ROOT'а на достижимость*/
PROCESS(ping_process, "Ping process");

/*Процесс инициализации настроек из EEPROM*/
PROCESS(settings_dag_init, "Initializing settings of DAG");

/*Запуск инициализации ноды (точка входа)*/
PROCESS(dag_node_process, "DAG-node process");

/*Процесс отслеживает нажатие кнопки. При нажатии происходит перезагрузка*/
PROCESS(dag_node_button_process, "DAG-node button process");

/*Процесс поиска ROOT'а*/
PROCESS(root_find_process, "Root find process");

/*Процесс управления нодой*/
PROCESS(maintenance_process, "Maintenance process");

/*Процесс упарвления светодиодами*/
PROCESS(led_process, "Led process");

/*---------------------------------------------------------------------------*/
/*Обработчик принятых пакетов*/
static void udp_receiver(struct simple_udp_connection *c,
						const uip_ipaddr_t *sender_addr,
						uint16_t sender_port,
						const uip_ipaddr_t *receiver_addr,
						uint16_t receiver_port,
						const uint8_t *data, 
						uint16_t datalen)
{
	/*Отражаем структуру на массив*/ 
	header_up_t *header_up_pack = (header_up_t*)&data[HEADER_OFFSET];

	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
	{
		printf("[DAG Node]: UDP crypto packet received(%"PRIu8"): ", datalen);
		for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
			printf("%"PRIXX8, data[i]);
		printf("\n");
	}
	
	/*Проверяем версию протокола*/ 
	if(header_up_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{

		/*Проверяем ID модуля и тип пакета*/ 
		if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == DATA_TYPE_JOIN_STAGE_2))
		{
			/*Третья стадия авторизации*/
			join_stage_3_sender(sender_addr, data, datalen);
		}
		
		else if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == DATA_TYPE_JOIN_STAGE_4))
		{
			/*Обработчик четвертой стадии авторизации*/
			join_stage_4_handler(sender_addr, data, datalen);
		}
		
		else 
		{	
			/*Расшифровываем данные*/
			aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], iterator_to_byte(datalen - HEADER_UP_LENGTH));

			/*Вывод информационного сообщения в консоль*/
			if(uart_status() == 0)
			{
				printf("DAG Node: UDP no crypto packet received(%"PRIu8"): ", datalen);
				for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
					printf("%"PRIXX8, data[i]);
				printf("\n");
			}
			
			/*Отражаем структуру на массив*/ 
			header_down_t *header_down_pack = (header_down_t*)&data[HEADER_DOWN_OFFSET];
			
			/*Защита от атаки повтором*/
			/*Проверяем счетчик пакетов на валидность данного пакета*/
			if(packet_counter_root.u16 >= header_down_pack->counter.u16)
			{	
				printf("COUNTER ERROR!!!\n");
				return;
			}
			
			/*CRC16 проверка*/ 
			if(header_down_pack->crc.u16 != crc16_arc((uint8_t*)&data[PAYLOAD_OFFSET], header_down_pack->length))
			{
				printf("CRC16 ERROR!!!\n");
				return;
			}
			
			/*Обновляем значение счетчика ROOT'а*/
			packet_counter_root.u16 = header_down_pack->counter.u16;	
			
			/*Проверяем ID модуля*/ 
			/*UNWDS-6LOWPAN_SYSTEM*/
			if(header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
			{
				if(header_up_pack->data_type == PONG)
				{
					pong_handler(sender_addr, (pong_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}

				else
				{
					return;
				}
					
			}
			
			/*UMDK-6FET*/ 
			if(header_up_pack->device_id == UNWDS_6FET_MODULE_ID)
			{
				if(header_up_pack->data_type == PWM_SETTINGS)
				{
					dag_pwm_settings(sender_addr, (pwm_settings_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else if(header_up_pack->data_type == PWM_POWER)
				{
					dag_pwm_power(sender_addr, (pwm_power_t*)&data[PAYLOAD_OFFSET]);
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else
				{
					return;
				}
					
			}
		
			/*UMDK-LIT*/ 
			else if(header_up_pack->device_id == UNWDS_LIT_MODULE_ID)
			{
				if(header_up_pack->data_type == LIT_MEASURE)
				{
					dag_lit_measure_sender();
					led_off(LED_A);		/*Выключаем светодиод*/
					return;
				}
				
				else
				{
					return;
				}	
			}

			else
			{
				return;
			}
		}
		
	}
	
	else
	{
		/*Вывод информационного сообщения в консоль*/
		if(uart_status() == 0)
			printf("[DAG Node] Incompatible protocol version: %"PRIXX8"\n", header_up_pack->protocol_version);
		return;
	}

	/*Мигаем светодиодом*/
	led_mode_set(LED_FLASH);
}

/*---------------------------------------------------------------------------*/
/**/
void pack_sender(uint8_t device_id, 
				uint8_t data_type, 
				uint8_t *payload, 
				uint8_t payload_len)
{	
	/*Проверка на то что передан не нулевой адрес буфера*/
	if ((payload == NULL) && (payload_len != 0))
		return;
		
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t crypto_length = iterator_to_byte(HEADER_DOWN_LENGTH + payload_len);
	uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = device_id;							/*ID устройства*/
	header_pack->data_type = data_type;							/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_node.u16;			/*Счетчик пакетов*/ 
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
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), (uip_ipaddr_t*)&root_addr);
	packet_counter_node.u16++;		/*Инкрементируем счетчик пакетов*/
}

/*---------------------------------------------------------------------------*/
/*Первая стадия авторизации*/
/*Передаём свой серийный номер*/
static void join_stage_1_sender(const uip_ipaddr_t *dest_addr)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;

	uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&addr, dest_addr);		/*Копируем адрес*/
	
	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
	{
		printf("[DAG Node] Send join packet to DAG-root node: ");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}

	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/	
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH];
	
	/*Отражаем структуры на массив*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = DATA_TYPE_JOIN_STAGE_1;			/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_node.u16;			/*Счетчик пакетов*/ 
	header_pack->length = JOIN_STAGE_1_LENGTH;					/*Размер пакета*/

	/*Payload*/
	join_stage_1_pack->module_id = UNWDS_MODULE_ID;
	
	/*CRC16*/ 
	header_pack->crc.u16 = 0;
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH), &addr);
}

/*---------------------------------------------------------------------------*/
/*Третья стадия авторизации*/
/*Получаем nonce и отправляем его root'у зашифрованым AES128-CBC*/
static void join_stage_3_sender(const uip_ipaddr_t *dest_addr,
								const uint8_t *data,
								uint16_t datalen)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&addr, dest_addr);		/*Копируем адрес*/
	
	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
	{
		printf("[DAG Node] Send join packet stage 3 to DAG-root node:");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH];	
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&data[PAYLOAD_OFFSET];
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = DATA_TYPE_JOIN_STAGE_3;			/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_node.u16;			/*Счетчик пакетов*/ 
	header_pack->length = JOIN_STAGE_3_LENGTH;					/*Размер пакета*/
	
	/*Payload*/ 
	/*Расшифровываем блок*/ 
	aes_ecb_decrypt((uint32_t*)aes_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET]);
	
	//CRC16
	
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = join_stage_2_pack->nonce.u8[1];	
		nonce_key[i+1] = join_stage_2_pack->nonce.u8[0];	
	}
	
	/*Отправляем ROOT'у nonce на еденицу больше для того что бы он был уверен что у нас одинаковое шифрование*/ 
	join_stage_3_pack->nonce.u16 = join_stage_2_pack->nonce.u16 + 1;
	
	/*Дозаполняем блок для шифрования нулями*/ 
	for(uint8_t i = JOIN_STAGE_3_LENGTH; i < (JOIN_STAGE_3_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/*CRC16*/ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&join_stage_3_pack, sizeof(join_stage_3_pack));
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&udp_buffer[HEADER_DOWN_OFFSET], (uint32_t*)&udp_buffer[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);		
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_DOWN_OFFSET + JOIN_STAGE_3_PAYLOAD_LENGTH), &addr);
}
/*---------------------------------------------------------------------------*/
/*Обработчик четвертой стадии авторизации*/
/*Сравниваем данные пришедшие от root'а для того что бы удостоверится в том что используются правильные настройки шифрования*/
static void join_stage_4_handler(const uip_ipaddr_t *sender_addr,
								 const uint8_t *data,
								 uint16_t datalen)
{	
	/*Проверка на то что передан существующий адрес*/
	if (sender_addr == NULL)
		return;

	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&data[HEADER_OFFSET];
	join_stage_4_t *join_stage_4_pack = (join_stage_4_t*)&data[PAYLOAD_OFFSET];
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);
	
	/*Проверяем массив. Если все нули, то авториция прошла успешно*/
	if(join_stage_4_pack->status_code)
	{
		packet_counter_root.u16 = header_pack->counter.u16;				/*Сохраняем счетчик пакетов ROOT'а*/ 
		uip_ipaddr_copy(&root_addr, sender_addr); 						/*Копируем адрес ROOT'а с которым авторизировались*/ 
		packet_counter_node.u16 = 1;									/*Инициализируем счетчик пакетов*/
		etimer_set(&maintenance_timer, 0);								/*Устанавливаем таймер*/ 
		process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);	/*Передаем управление dag_node_process*/ 
		process_start(&ping_process, NULL);
		return;
	}
	
	/*Выводим: Ошибка авторизации*/
	printf("[DAG Node] Authorisation Error\n"); 
}

/*---------------------------------------------------------------------------*/
/*Ping*/
static void ping_sender(void)
{
	/*Заполняем payload*/
	ping_t ping_pack;							/*Создаем структуру*/
	
	ping_pack.nonce.u8[1] = nonce_key[0];		/*Заполняем nonce*/
	ping_pack.nonce.u8[0] = nonce_key[1];		/*Заполняем nonce*/
	
	/*Отправляем пакет*/	
	pack_sender(UNWDS_6LOWPAN_SYSTEM_MODULE_ID,
				PING, 							/*Команда ping*/
				(uint8_t*)&ping_pack, 			/*Payload*/
				sizeof(ping_pack));				/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Pong*/
static void pong_handler(const uip_ipaddr_t *sender_addr,
						pong_t *pong_pack)
{	
	if(pong_pack->status_code)
	{
		non_answered_ping = 0;
		return;
	}
	
	else
	{
		/*Выводим: Ошибка авторизации*/
		printf("[DAG Node] Crypto Error. Reboot\n");
		watchdog_reboot();
		
		// node_mode = MODE_JOIN_PROGRESS; 	/*Установка режима работы устройства*/
		// process_exit(&maintenance_process);
		// process_start(&maintenance_process, NULL);
	}
}

/*---------------------------------------------------------------------------*/
/*Инициализация с заданными настройками канала ШИМ'а*/
static void dag_pwm_settings(const uip_ipaddr_t *sender_addr,
							pwm_settings_t *pwm_settings_pack)
{	
	if(pwm_settings_pack->channel == 0)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_5);
	}
	
	else if(pwm_settings_pack->channel == 1)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_6);
	}
	else if(pwm_settings_pack->channel == 2)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_7);
	}
	else if(pwm_settings_pack->channel == 3)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_24);
	}
	else if(pwm_settings_pack->channel == 4)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_25);
	}
	else if(pwm_settings_pack->channel == 5)
	{
		pwm_config(pwm_settings_pack->channel, pwm_settings_pack->frequency, pwm_settings_pack->duty, IOID_26);
	}
}

/*---------------------------------------------------------------------------*/
/*Включение/выключение канала ШИМ'а*/
static void dag_pwm_power (	const uip_ipaddr_t *sender_addr,
							pwm_power_t *pwm_power_pack)
{
	uint8_t pwm_channel = pwm_power_pack->pwm_power & 0x7F;
	uint8_t pwm_power = ((pwm_power_pack->pwm_power & 0x80) >> 7); 
	
	if(pwm_power)
	{
		pwm_start(pwm_channel);
	}
	
	else
	{
		pwm_stop(pwm_channel);
	}
}

/*---------------------------------------------------------------------------*/
/*Совершить замер освещенности*/
static void dag_lit_measure_sender()
{
	/*Заполняем payload*/
	lit_measure_t lit_measure_pack;						/*Создаем структуру*/
	
	lit_measure_pack.lit_measure = opt3001_measure();	/*Измеряем освещенность*/
	
	/*Вывод информационного сообщения в консоль*/
	printf("[DAG Node] Send LIT measure packet to DAG-root node\n");
	printf("[UMDK-LIT] Luminocity: %lu lux\n", lit_measure_pack.lit_measure);
	
	/*Отправляем пакет*/	
	pack_sender(UNWDS_LIT_MODULE_ID,
				LIT_MEASURE, 							/*Команда включения канала ШИМ'а*/
				(uint8_t*)&lit_measure_pack, 			/*Payload*/
				sizeof(lit_measure_pack));				/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Функция отправки состояния кнопок*/
void button_status_sender ( uint8_t button_number,
							uint8_t click_type)
{
	/*Заполняем payload*/
	button_status_t button_status_pack;				/*Создаем структуру*/
	
	button_status_pack.button_status = button_number;

	if(click_type == CLICK)
		button_status_pack.button_status |= CLICK;

	else if (click_type == LONG_CLICK)
		button_status_pack.button_status |= LONG_CLICK;
	
	/*Вывод информационного сообщения в консоль*/
	// printf("[DAG Node] Send button status packet to DAG-root node\n");
	
	/*Отправляем пакет*/
	pack_sender(UNWDS_4BTN_MODULE_ID,
				BUTTON_STATUS, 						/*Команда включения канала ШИМ'а*/
				(uint8_t*)&button_status_pack, 		/*Payload*/
				sizeof(button_status_pack));		/*Размер payload'а*/
}

/*---------------------------------------------------------------------------*/
/*Функция управления светодиодами*/
void led_mode_set(uint8_t mode)
{
	led_mode = mode;
	if (led_mode == LED_OFF)
		led_off(LED_A);

	if (led_mode == LED_ON)
		led_on(LED_A);

	if ((led_mode == LED_SLOW_BLINK) || (led_mode == LED_FAST_BLINK) || (led_mode == LED_FLASH))
		process_start(&led_process, NULL);
	else
		process_exit(&led_process);
}

/*---------------------------------------------------------------------------*/
/*Процесс опроса ROOT'а на достижимость*/
PROCESS_THREAD(ping_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;
	
	static struct etimer ping_timer;							/*Создаём таймер для по истечении которого будет ROOT будет пинговаться*/
	
	while (1)
	{
		etimer_set(&ping_timer, (CLOCK_SECOND * 10));			/*Устанавливаем таймер на 10 минут*/
		
		if(non_answered_ping > 3)								/*Перезагрузить если больше трех неотвеченных пингов*/
		{
			printf("Ping error!");
			watchdog_reboot();
		}
		
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ping_timer));	/*Засыпаем до срабатывания таймера*/
		
		non_answered_ping++;									/*Увеличиваем на еденицу. При ответе в pong_handler() должно обнулиться*/		
		ping_sender();											/*Отправляем ping*/
	}
	
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс инициализации настроек из EEPROM*/
PROCESS_THREAD(settings_dag_init, ev, data)
{
	PROCESS_BEGIN();
	
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	read_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
	
	if(eeprom_settings.aes_key_configured == true)
	{
		if((eeprom_settings.channel != 26) && (eeprom_settings.panid != 0xAABB))
		{
			eeprom_settings.channel = 26;
			eeprom_settings.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
		}
	}
	
	if(!eeprom_settings.aes_key_configured) 
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_settings.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");
		;
	}
	else
	{
		printf("AES-128 key not declared\n***PLEASE SET AES KEY***\n");
		led_mode_set(LED_FAST_BLINK);	/*Мигаем светодиодом*/
		while(eeprom_settings.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	radio_value_t channel = 0;
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	if(channel != eeprom_settings.channel)
	{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_settings.channel);
		
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_settings.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_settings.channel, freq_mhz);
		}

		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_settings.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_settings.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		if(panid != eeprom_settings.panid)
		{
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_settings.panid);
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_settings.panid);
		}
	}
	
	process_post(&main_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс упарвления светодиодами*/
PROCESS_THREAD(led_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;
	
	static struct etimer led_mode_timer;

	while((led_mode == LED_SLOW_BLINK) || (led_mode == LED_FAST_BLINK) || (led_mode == LED_FLASH))
	{
		if(led_mode == LED_FAST_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/10);

		if(led_mode == LED_SLOW_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/2);

		if(led_mode == LED_FLASH)
			etimer_set(&led_mode_timer, 1);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&led_mode_timer));

		led_on(LED_A);

		if(led_mode == LED_FAST_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/32);

		if(led_mode == LED_SLOW_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/32);

		if(led_mode == LED_FLASH)
			etimer_set(&led_mode_timer, CLOCK_SECOND/16);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&led_mode_timer) );

		led_off(LED_A);

		if(led_mode == LED_FLASH)
			led_mode = LED_OFF;
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс отслеживает нажатие кнопки. При нажатии происходит перезагрузка*/
PROCESS_THREAD(dag_node_button_process, ev, data)
{
	PROCESS_BEGIN();
	
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	PROCESS_PAUSE();

	while (1)
	{
		PROCESS_YIELD();

		if (ev == sensors_event)
		{
			if (data == &button_e_sensor_long_click)
			{
				/*Включаем светодиод*/
				led_mode_set(LED_ON);	
				
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("[SYSTEM] Button E long click, reboot\n");
				
				watchdog_reboot();
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс управления нодой*/
PROCESS_THREAD(maintenance_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;

	PROCESS_PAUSE();

	while(1)
	{
		if(node_mode == MODE_NEED_REBOOT)
		{
			static struct etimer maintenance_reboot_timer;
			etimer_set(&maintenance_reboot_timer, (5 * CLOCK_SECOND));
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&maintenance_reboot_timer) );
			watchdog_reboot();
		}

		if(node_mode == MODE_NORMAL)
		{
			led_mode_set(LED_OFF);	/*Выключаем светодиод*/

			if(process_is_running(&root_find_process) == 1)
				process_exit(&root_find_process);
		}

		if(node_mode == MODE_NOTROOT)
		{
			if(CLASS == CLASS_B)
			{
				led_mode_set(LED_OFF);
				
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("[DAG Node] Root not found, sleep\n");
				
				if(process_is_running(&dag_node_button_process) == 1)
					process_exit(&dag_node_button_process);

				if(process_is_running(&root_find_process) == 1)
					process_exit(&root_find_process);

				if(process_is_running(&maintenance_process) == 1)
					process_exit(&maintenance_process);
				
				net_mode(RADIO_FREEDOM);
				net_off(RADIO_OFF_NOW);
				net_mode(RADIO_HOLD);

				etimer_set(&maintenance_timer, (5 * 60 * 60 * CLOCK_SECOND));
				PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&maintenance_timer) );
				watchdog_reboot();
			}

			if(CLASS == CLASS_C)
			{
				/*Мигаем светодиодом*/
				led_mode_set(LED_FAST_BLINK);
				
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("[DAG Node] Root not found, reboot\n"); //почему-то не перезагружается!
				
				watchdog_reboot();
			}
		}

		if(node_mode == MODE_JOIN_PROGRESS)
		{
			net_on(RADIO_ON_NORMAL);
			net_mode(RADIO_HOLD);
			led_mode_set(LED_SLOW_BLINK);

			if(process_is_running(&root_find_process) == 0)
				process_start(&root_find_process, NULL);
		}

		etimer_set(&maintenance_timer, MAINTENANCE_INTERVAL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&maintenance_timer) );
	}
	
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс поиска ROOT'а*/
PROCESS_THREAD(root_find_process, ev, data)
{
	PROCESS_BEGIN();

	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	static struct etimer find_root_timer;
	static struct etimer find_root_limit_timer;
	static rpl_dag_t *root_find_dag = NULL;
	static uip_ds6_addr_t *ds6_addr = NULL;
	PROCESS_PAUSE();

	etimer_set(&find_root_limit_timer, ROOT_FIND_LIMIT_TIME);

	while (1)
	{
		if(node_mode == MODE_JOIN_PROGRESS)
		{
			if(!etimer_expired(&find_root_limit_timer))
			{
				ds6_addr = uip_ds6_get_global(ADDR_PREFERRED);
				if(ds6_addr != NULL)
				{
					root_find_dag = rpl_get_dag(&ds6_addr->ipaddr);
					if(root_find_dag != NULL)
					{
						if(led_mode != LED_FAST_BLINK)
						led_mode_set(LED_FAST_BLINK);	/*Мигаем светодиодом*/
						join_stage_1_sender(&root_find_dag->dag_id);
					}
				}
			}
			else
			{
				node_mode = MODE_NOTROOT;
				
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("[DAG Node] mode set to MODE_NOTROOT\n");
				
				process_exit(&maintenance_process);
				process_start(&maintenance_process, NULL);
			}
		}
		etimer_set( &find_root_timer, ROOT_FIND_INTERVAL + (random_rand() % ROOT_FIND_INTERVAL) );
		PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&find_root_timer) );
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Запуск инициализации ноды (точка входа)*/
PROCESS_THREAD(dag_node_process, ev, data)
{
	PROCESS_BEGIN();
	PROCESS_PAUSE();

	/*Инициализация обработчика входящих пакетов*/
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_receiver);

	/*Выбор режима работы RPL*/
	if (CLASS == CLASS_B)
		rpl_set_mode(RPL_MODE_LEAF);
	else
		rpl_set_mode(RPL_MODE_MESH);

	
	node_mode = MODE_JOIN_PROGRESS; 	/*Установка начального режима работы устройства*/
	packet_counter_node.u16 = 1;		/*Инициализация счетчика*/

	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
		printf("Node started, %s mode, %s class, version %"PRIu8".%"PRIu8"\n",
				rpl_get_mode() == RPL_MODE_LEAF ? "leaf" : "no-leaf",
				CLASS == CLASS_B ? "B(sleep)" : "C(non-sleep)",
				BIG_VERSION, 
				LITTLE_VERSION);

	process_start(&dag_node_button_process, NULL);		/*Запускаем процес который отслеживает нажатие кнопок*/
	process_start(&maintenance_process, NULL);			/*Запускаем процес управления нодой*/

	SENSORS_ACTIVATE(batmon_sensor);					/*Инициализация встроенного датчика температуры и напряжения процессора*/

	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);	/*Ожидаем пока нода авторизируется в сети*/

	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
		printf("[DAG Node] DAG active, join stage 4 packet received, mode set to MODE_NORMAL\n");
	
	led_mode_set(LED_SLOW_BLINK);	/*Включаем медленное мигание светодиодами*/
	node_mode = MODE_NORMAL;		/*Изменение режима работы ноды. Нода работает в нормальном режиме*/
	net_mode(RADIO_FREEDOM);		/**/
	net_off(RADIO_OFF_NOW);			/**/

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/