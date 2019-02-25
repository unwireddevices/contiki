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

// #define DPRINT printf(">dag_node.c:%"PRIu16"\n", __LINE__);watchdog_periodic();

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"

#include "dev/leds.h"
#include "cc26xx/board.h"

#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6.h"
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
#include "common-node.h"
#include "crypto-common.h"
#include "rtc-common.h"
#include "int-flash-common.h"

#include "dev/serial-line.h"
#include "../cpu/cc26xx-cc13xx/dev/cc26xx-uart.h"
#include "../../core/dev/serial-line.h"

#include "../cpu/cc26xx-cc13xx/dev/pwm.h" /*PWM*/
#include "softel_lighting/softel_lighting.h"

/* DAG */
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

#include "xxf_types_helper.h"
#include "protocol.h"

/* DAG DEF */
#define MAINTENANCE_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define SHORT_STATUS_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define LONG_STATUS_INTERVAL			(20 * 60 * CLOCK_SECOND)
#define ROOT_FIND_INTERVAL				(2 * CLOCK_SECOND)
#define ROOT_FIND_LIMIT_TIME			(2 * 60 * CLOCK_SECOND)

#define REQ_DATA_FOR_OTA_INTERVAL		(2 * CLOCK_SECOND)

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

#define IOC_OUTPUT_PULL_UP	(IOC_CURRENT_2MA	| IOC_STRENGTH_AUTO	| \
							IOC_IOPULL_UP		| IOC_SLEW_DISABLE	| \
							IOC_HYST_DISABLE	| IOC_NO_EDGE		| \
							IOC_INT_DISABLE		| IOC_IOMODE_NORMAL	| \
							IOC_NO_WAKE_UP		| IOC_INPUT_DISABLE	)	

#define IOC_OUTPUT_HIGH_DRIVE_INV (IOC_CURRENT_8MA	| IOC_STRENGTH_AUTO	| \
								   IOC_NO_IOPULL	| IOC_SLEW_DISABLE	| \
								   IOC_HYST_DISABLE	| IOC_NO_EDGE		| \
								   IOC_INT_DISABLE	| IOC_IOMODE_INV	| \
								   IOC_NO_WAKE_UP	| IOC_INPUT_DISABLE	)

#define IOC_INPUT_PULL_DOWN (IOC_CURRENT_2MA	| IOC_STRENGTH_AUTO	| \
                             IOC_IOPULL_DOWN	| IOC_SLEW_DISABLE	| \
                             IOC_HYST_DISABLE	| IOC_NO_EDGE		| \
                             IOC_INT_DISABLE	| IOC_IOMODE_NORMAL | \
                             IOC_NO_WAKE_UP		| IOC_INPUT_ENABLE )

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

#define WAIT_RESPONSE 			3 	//Максимальное время ожидания ответа от счетчика в секундах

/*---------------------------------------------------------------------------*/
/* ОБЩИЕ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ */
/*---------------------------------------------------------------------------*/

uint8_t aes_key[16];				/* Ключ шифрования */
static uint8_t nonce_key[16];		/* Сессионный ключ */

/* Структура UDP подключения */
simple_udp_connection_t udp_connection;	

/* Настройки из EEPROM */
eeprom_t eeprom_settings;				

/* Счетчик пакетов */
static volatile union 
{ 
	uint16_t u16; 
	uint8_t u8[2]; 
} packet_counter_root;

/* Режим работы ROOT or DAG */
static volatile bool mode_node = 0;		

/*---------------------------------------------------------------------------*/
/* ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ROOT'а */
/*---------------------------------------------------------------------------*/
/* Сравниваем с этим адресом (аналог nullptr) */
static uip_ipaddr_t null_addr;			

/*---------------------------------------------------------------------------*/
/* ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ DAG'а */
/*---------------------------------------------------------------------------*/

volatile uint8_t led_mode;

volatile uint8_t non_answered_ping = 0;	/* Количество неотвеченых пингов */

static struct etimer maintenance_timer;	/* Таймер */

static uint16_t num_blocks;
static uint16_t blocks_counter = 0;

process_event_t ota_event_message; 		/* ota event */

static uint32_t pointer_on_last_state = 0;

/*---------------------------------------------------------------------------*/
/* ПРОТОТИПЫ ОБЩИХ ФУНКЦИЙ */
/*---------------------------------------------------------------------------*/

//
//
//

/*---------------------------------------------------------------------------*/
/* ПРОТОТИПЫ ФУНКЦИЙ ROOT'а */
/*---------------------------------------------------------------------------*/
/* Вторая стадия авторизации */
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

/* Четвертая стадия авторизации */
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

/* Pong */
static void pong_sender(const uip_ip6addr_t *dest_addr, ping_t *ping_pack);

/* ACK */
static void ack_handler(const uip_ip6addr_t *dest_addr, ack_t *ack_pack);

/* NACK */
static void nack_handler(const uip_ip6addr_t *dest_addr, nack_t *nack_pack);

/* Конструктор пакета из UART */
static void send_pack_from_cr(uint8_t* data);

/* Вывод принятого пакета микрокомпьютеру */ 
static void print_cr(const uip_ip6addr_t *dest_addr, uint8_t *data, uint16_t length);

/*---------------------------------------------------------------------------*/
/* ПРОТОТИПЫ ФУНКЦИЙ DAG'а */
/*---------------------------------------------------------------------------*/
/* Обработчик принятых пакетов */
static void dag_udp_data_receiver(struct simple_udp_connection *c,
								  const uip_ipaddr_t *sender_addr,
								  uint16_t sender_port,
								  const uip_ipaddr_t *receiver_addr,
								  uint16_t receiver_port,
								  const uint8_t *data, 
								  uint16_t datalen);
						
/* Первая стадия авторизации */
static void join_stage_1_sender(const uip_ipaddr_t *dest_addr);

/* Третья стадия авторизации */
static void join_stage_3_sender(const uip_ipaddr_t *dest_addr,
								const uint8_t *data,
								uint16_t datalen);

/* Обработчик четвертой стадии авторизации */
static void join_stage_4_handler(const uip_ipaddr_t *sender_addr,
								 const uint8_t *data,
								 uint16_t datalen);

/* Ping */
static void ping_sender(void);

/* Pong */
static void pong_handler(const uip_ipaddr_t *sender_addr,
						 pong_t *pong_pack);
						
/* ACK */
static void ack_sender(uint16_t counter);
						
/* NACK */
static void nack_sender(uint16_t counter);

/* Команда включения/выключения канала ШИМ'а c заданным duty cycle */
static bool dag_pwm_set(pwm_set_t *pwm_set_pack);

/* Команда запроса блока данных для OTA */
static void dag_ota_req_data_sender(uint16_t ota_block);

/* Команда отправки сообщения что прошивка обновлена */
static void dag_finish_ota_sender(int8_t status);

/* Функция восстановления последнего состояния освещения */
static void restore_last_state_of_lighting(void);

/**/
static void save_last_state_of_lighting(uint8_t state);

/*---------------------------------------------------------------------------*/
/* ПРОТОТИПЫ ПРОЦЕССОВ */
/*---------------------------------------------------------------------------*/
/* ОБЩИЕ ПРОЦЕССЫ */
/*---------------------------------------------------------------------------*/
/* Процесс инициализации настроек из EEPROM */
PROCESS(settings_init, "Initializing settings");

/* Процесс упарвления светодиодами */
PROCESS(led_process, "Led process");

/*---------------------------------------------------------------------------*/
/* ПРОЦЕССЫ ROOT'а */
/*---------------------------------------------------------------------------*/
/* Процесс управления ROOT'ом */
PROCESS(main_root_process, "Main root process");

/*---------------------------------------------------------------------------*/
/* ПРОЦЕССЫ DAG'а */
/*---------------------------------------------------------------------------*/
/* Процесс опроса ROOT'а на достижимость */
PROCESS(ping_process, "Ping process");

/* Запуск инициализации ноды (точка входа) */
PROCESS(dag_node_process, "DAG-node process");

/* Процесс отслеживает нажатие кнопки. При нажатии происходит перезагрузка */
PROCESS(dag_node_button_process, "DAG-node button process");

/* Процесс поиска ROOT'а */
PROCESS(root_find_process, "Root find process");

/* Процесс управления нодой */
PROCESS(maintenance_process, "Maintenance process");

/* Процесс обновления по воздуху */
PROCESS(ota_process, "OTA process");

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ОБЩИЕ ФУНКЦИИ */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Конструктор пакета */
void pack_sender(const uip_ip6addr_t *dest_addr, 
				 uint8_t device_id, 
				 uint8_t data_type, 
				 uint16_t payload_len, 
				 uint8_t *payload)
{
	/* Проверка на то что передан существующий адрес */
	if (dest_addr == NULL)
		return;
	
	/* Проверка на то что передан не нулевой адрес буфера */
	if ((payload == NULL) && (payload_len != 0))
		return;
	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint16_t crypto_length = iterator_to_byte(HEADER_DOWN_LENGTH + payload_len);
	uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	/* Отражаем структуры на массивы */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	u8_u16_t nonce;
	if(node_is_root())
	{
		/* Получаем nonce */
		nonce.u16 = get_nonce((uip_ip6addr_t*)dest_addr);	
		
		/* Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC) */
		for(int i = 0; i < 16; i += 2)
		{
			nonce_key[i] = nonce.u8[1];	
			nonce_key[i+1] = nonce.u8[0];	
		}
	}

	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = device_id;							/* ID устройства */
	header_pack->data_type = data_type;							/* Тип пакета */  
	header_pack->rssi = get_parent_rssi();						/* RSSI */ 
	header_pack->temperature = get_temperature();				/* Температура */ 
	header_pack->voltage = get_voltage();						/* Напряжение */ 
	if(node_is_root())
		header_pack->counter.u16 = packet_counter_root.u16;		/* Счетчик пакетов ROOT'а */ 
	else
		header_pack->counter.u16 = packet_counter_node.u16;		/* Счетчик пакетов DAG'а */ 
	header_pack->length.u16 = payload_len;						/* Размер пакета (незашифрованного) */
	
	/* Payload */
	/* Проверка что бы не улетел в бесконечный цикл */ 
	if((crypto_length == 0) || ((crypto_length - HEADER_DOWN_LENGTH) <= 0))
		return;

	/* Заполняем пакет, зашифровываем и отправляем */ 
	for(uint16_t i = 0; i < (crypto_length - HEADER_DOWN_LENGTH); i++)
	{
		if(i < payload_len)
			udp_buffer[PAYLOAD_OFFSET + i] = payload[i];
		else
			udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	}
	
	/* CRC16 */ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], header_pack->length.u16);
	
	/* Для отладки. Выводит содержимое пакета */ 
	// printf("Pack: \n");
	// hexraw_print(16, (uint8_t*)dest_addr);
	// hexraw_print(1, (uint8_t*)&(header_pack->device_id));
	// hexraw_print(1, (uint8_t*)&(header_pack->data_type));
	// hexraw_print(1, (uint8_t*)&(header_pack->length));	
	// hexraw_print(payload_len, (uint8_t*)payload);
	// printf("\n");
	
	// printf("Pack:\n");
	// hexraw_print((HEADER_UP_LENGTH + crypto_length), (uint8_t*)udp_buffer);
	// printf("\n");
	
	/* Зашифровываем данные */
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);
	
	/* Отправляем пакет */ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), dest_addr);

	/* Инкрементируем счетчик пакетов */
	if(node_is_root())
		packet_counter_root.u16++;
	else
		packet_counter_node.u16++;
}

/*---------------------------------------------------------------------------*/
/* Функция управления светодиодами */
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
/* Проверка является ли эта нода рутом */
bool node_is_root(void)
{
	return mode_node;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ФУНКЦИИ ROOT'а */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Обработчик принятых пакетов */
void root_udp_data_receiver(struct simple_udp_connection *connection,
                       		const uip_ipaddr_t *sender_addr, 
                       		uint16_t sender_port,
                       		const uip_ipaddr_t *receiver_addr, 
                       		uint16_t receiver_port,
                       		const uint8_t *data, 
                       		uint16_t datalen)
{
	/* Отражаем структуру на массив */ 
	header_up_t *header_up_pack = (header_up_t*)&data[HEADER_OFFSET];
	
	/* Для отладки. Выводит тип принятого сообщения */ 
	// printf("Recive pack: %x\n", header_up_pack->data_type);
	
	/* Вывод информационного сообщения в консоль */
	// printf("[");
	// uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
	// printf("] Packet crypto received(%"PRIu8"): ", datalen);
	// for (uint16_t i = 0; i < datalen; i++)	/* Выводим принятый пакет */ 
	// 	printf("%"PRIXX8, data[i]);
	// printf("\n");
	
	/* Проверяем версию протокола */ 
	if(header_up_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{
		/* Проверяем ID модуля и тип пакета */ 
		if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == JOIN_STAGE_1))
		{
			/* Вторая стадия авторизации */			
			join_stage_2_sender((uip_ip6addr_t*)sender_addr, data, datalen);
			
			/* Мигаем светодиодом */
			led_mode_set(LED_FLASH);	
			return;
		}
		
		else if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == JOIN_STAGE_3))
		{
			/* Четвертая стадия авторизации */
			join_stage_4_sender((uip_ip6addr_t*)sender_addr, data, datalen);
			
			/* Мигаем светодиодом */
			led_mode_set(LED_FLASH);
			return;
		}
		
		else
		{
			/* Получаем nonce */
			u8_u16_t nonce;
			nonce.u16 = get_nonce((uip_ip6addr_t*)sender_addr);	
			
			/* Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC) */
			for(int i = 0; i < 16; i += 2)
			{
				nonce_key[i] = nonce.u8[1];	
				nonce_key[i+1] = nonce.u8[0];	
			}
			
			/* Расшифровываем данные */
			aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], iterator_to_byte(datalen - HEADER_UP_LENGTH));
			
			/* Вывод информационного сообщения в консоль */
			// printf("Packet no crypto received(%"PRIu8"): ", datalen);
			// for (uint16_t i = 0; i < datalen; i++)	/* Выводим принятый пакет */ 
				// printf("%"PRIXX8, data[i]);
			// printf("\n");
			
			/* Отражаем структуру на массив */ 
			header_down_t *header_down_pack = (header_down_t*)&data[HEADER_DOWN_OFFSET];
						
			/* CRC16 проверка */ 
			if(header_down_pack->crc.u16 != crc16_arc((uint8_t*)&data[PAYLOAD_OFFSET], header_down_pack->length.u16))
			{
				/* Вывод сообщения об ошибке целостности пакета */
				printf("[");
				uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
				printf("] CRC16 Error!\n");
				
				/* Мигаем светодиодом */
				led_mode_set(LED_FLASH);
				return;
			}
			
			/* Защита от атаки повтором */
			/* Проверяем счетчик пакетов на валидность данного пакета */
			if(!valid_counter((uip_ip6addr_t*)sender_addr, header_down_pack->counter.u16))
			{
				/* Вывод сообщения об ошибке счетчика пакетов */
				printf("[");
				uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
				printf("] Counter error!\n");
				
				/* Мигаем светодиодом */
				led_mode_set(LED_FLASH);
				return;
			}
			
			/* Вывод принятого пакета микрокомпьютеру */ 
			print_cr((uip_ip6addr_t*)sender_addr, (uint8_t*)data, (HEADER_LENGTH + header_down_pack->length.u16));
				
			/* Проверяем ID модуля */ 
			/* UNWDS-6LOWPAN_SYSTEM */
			if(header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
			{
				if(header_up_pack->data_type == PING)
				{
					/* Pong */
					pong_sender((uip_ip6addr_t*)sender_addr, (ping_t*)&data[PAYLOAD_OFFSET]);
					
					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}
				
				else if(header_up_pack->data_type == ACK)
				{
					/* ACK */
					ack_handler((uip_ip6addr_t*)sender_addr, (ack_t*)&data[PAYLOAD_OFFSET]);
					
					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}
				
				else if(header_up_pack->data_type == NACK)
				{
					/* NACK */
					nack_handler((uip_ip6addr_t*)sender_addr, (nack_t*)&data[PAYLOAD_OFFSET]);
					
					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}

				else if(header_up_pack->data_type == REQ_DATA_FOR_OTA)
				{
					/* REQ_DATA_FOR_OTA */
					/* Отражаем структуру на массив */
					req_data_for_ota_t *req_data_for_ota_pack = (req_data_for_ota_t*)&data[PAYLOAD_OFFSET];

					/* Вывод информационного сообщения в консоль */
					printf("[");
					uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
					printf("] Request %i block data for OTA\n", req_data_for_ota_pack->ota_block);

					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}

				else if(header_up_pack->data_type == FINISH_OTA)
				{
					/* FINISH_OTA */
					printf("[");
					uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
					printf("] Finish OTA!\n");

					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}
				
				else
				{	
					/* Вывод сообщения об неизвестной команде */
					printf("[");
					uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
					printf("] Unknown command for system!\n");
			
					/* Мигаем светодиодом */
					led_mode_set(LED_FLASH);
					return;
				}
			}
			
			else
			{	
				/* Вывод сообщения о неизвестном модуле */
				printf("[");
				uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
				printf("] Unknown module!\n");
		
				/* Мигаем светодиодом */
				led_mode_set(LED_FLASH);
				return;
			}
		}	
	}
	
	else 
	{
		/* Вывод сообщения о неизвестном протоколе */
		printf("[");
		uip_debug_ipaddr_print((uip_ip6addr_t*)sender_addr);
		printf("] Unknown protocol version!\n");
		
		/* Мигаем светодиодом */
		led_mode_set(LED_FLASH);
		return;
	}
}	

/*---------------------------------------------------------------------------*/
/* Вторая стадия авторизации */
/* Генерирует сессионный ключ (nonce) и отправляет его зашифрованным AES128-ECB, добавляет маршрут в таблицу */
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, 
								const uint8_t *data, 
								const uint16_t length)
{	
	/* Вывод принятого пакета микрокомпьютеру */ 
	print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_1_LENGTH));

	/* Вывод сообщения об успешной авторизации */
	printf("[");
	uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
	printf("] Join stage 2\n");
	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH];
	
	/* Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_2;						/* Тип пакета */  
	header_pack->rssi = get_parent_rssi();						/* RSSI */ 
	header_pack->temperature = get_temperature();				/* Температура */ 
	header_pack->voltage = get_voltage();						/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_root.u16;			/* Счетчик пакетов */ 
	header_pack->length.u16 = JOIN_STAGE_2_LENGTH;				/* Размер пакета (незашифрованного) */
	
	/* Payload */ 
	join_stage_2_pack->nonce.u16 = random_rand();				/*Генерируем сессионный ключ */ 
	
	/* Добавляем маршрут */ 
	add_route ( (uip_ip6addr_t*)dest_addr,						/* Address */ 
				join_stage_2_pack->nonce.u16);					/* Nonce */ 
	
	/* Дозаполняем блок для шифрования нулями */ 
	for(uint16_t i = JOIN_STAGE_2_LENGTH; i < (JOIN_STAGE_2_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/* CRC16 */ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&join_stage_2_pack, sizeof(join_stage_2_pack));
	
	/* Зашифровываем блок */ 
	aes_ecb_encrypt((uint32_t*)aes_key, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]));
	
	/* Отправляем пакет */ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH), dest_addr);
	packet_counter_root.u16++;	/* Инкрементируем счетчик пакетов */ 
}

/*---------------------------------------------------------------------------*/
/* Четвертая стадия авторизации */
/* Принимает nonce зашифрованный AES128-CBC. Если сходится с тем что он сгенерировал, то авторизация прошла успешно, настройки шифрования верные. Отправляем пакет с нулями что бы DAG мог убедиться в этом */
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, 
								const uint8_t *data, 
								const uint16_t length)
{	
	/* Получаем nonce */
	u8_u16_t nonce;
	nonce.u16 = get_nonce((uip_ip6addr_t*)dest_addr);	
	
	/* Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC) */
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = nonce.u8[1];	
		nonce_key[i+1] = nonce.u8[0];	
	}

	/* Расшифровываем данные */
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);
	
	/* Вывод принятого пакета микрокомпьютеру */ 
	print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_3_LENGTH));
	
	/* Отражаем структуры на массивы */ 
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&data[PAYLOAD_OFFSET];
	
	/* Заполняем payload */
	join_stage_4_t join_stage_4_pack;				/* Создаем структуру */
	
	/* Проверяем одинаковые ли у нас настройки шифрования */ 
	if((get_nonce((uip_ip6addr_t*)dest_addr) + 1) != join_stage_3_pack->nonce.u16)
	{
		join_stage_4_pack.status_code = false;
		
		/* Вывод сообщения об успешной авторизации */
		printf("[");
		uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
		printf("] Authorization error node\n");
	}
	
	/* Если nonce'ы совпадают, то авторизация прошла успешно, шифрование настроено правильно */ 
	else
	{
		join_stage_4_pack.status_code = true;		/* Статус код */
		unlock_addr((uip_ip6addr_t*)dest_addr);		/* Разрешаем обрабатывать пакеты принятые с авторизованного устройства */
		
		/* Вывод сообщения об успешной авторизации */
		printf("[");
		uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
		printf("] Authorized node\n");		
	}
			
	/* Отправляем пакет */
	pack_sender(dest_addr, 							/* Адрес модуля */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID, 	/* Индентификатор модуля */
				JOIN_STAGE_4, 						/* Команда 4 стадии авторизации */
				JOIN_STAGE_4_LENGTH, 				/* Размер payload'а */
				(uint8_t*)&join_stage_4_pack);		/* Payload */
}

/*---------------------------------------------------------------------------*/
/* Pong */
static void pong_sender(const uip_ip6addr_t *dest_addr, 
						ping_t *ping_pack)
{
	/* Заполняем payload */
	pong_t pong_pack;							/* Создаем структуру */
	
	pong_pack.status_code = true;				/* Статус код */
	
	/* Проверяем массив. Если nonce совпали, то настройки шифрования верны */ 
	if(ping_pack->nonce.u16 == get_nonce((uip_ip6addr_t*)dest_addr))
	{
		pong_pack.status_code = true;
		
		/* Вывод сообщения об успешной авторизации */
		printf("[");
		uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
		printf("] Ping\n");
	}
	
	else
	{
		pong_pack.status_code = false;
	}
		
	/* Отправляем пакет */
	pack_sender(dest_addr, 							/* Адрес модуля */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID, 	/* Идентификатор модуля */ 
				PONG, 								/* Команда ответа на PING */
				PONG_LENGTH, 						/* Размер payload'а */
				(uint8_t*)&pong_pack);				/* Payload */
}

/*---------------------------------------------------------------------------*/
/* ACK */
static void ack_handler(const uip_ip6addr_t *dest_addr, 
						ack_t *ack_pack)
{	
	/* Вывод сообщения о успешной доставке пакета */
	printf("[");
	uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
	printf("] Package %i is confirmed\n", ack_pack->counter.u16);
}

/*---------------------------------------------------------------------------*/
/* NACK */
static void nack_handler(const uip_ip6addr_t *dest_addr, 
						 nack_t *nack_pack)
{
	/* Вывод сообщения о неуспешной доставке пакета */
	printf("[");
	uip_debug_ipaddr_print((uip_ip6addr_t*)dest_addr);
	printf("] Package %i is't confirmed\n", nack_pack->counter.u16);
}

/*---------------------------------------------------------------------------*/
/* Отправка настроек канала ШИМ'а */
void pwm_settings_sender(const uip_ip6addr_t *dest_addr, 
						 uint8_t channel, 
						 uint32_t frequency, 
						 uint8_t duty)
{	
	/* Заполняем payload */
	pwm_settings_t pwm_settings_pack;					/* Создаем структуру */
	
	pwm_settings_pack.channel = channel;				/* Номер канала */
	pwm_settings_pack.frequency = frequency;			/* Частота */
	pwm_settings_pack.duty = duty;						/* Коэффицент заполненния */
			
	/* Отправляем пакет */
	pack_sender(dest_addr, 							/* Адрес модуля UMDK-6FET */
				UNWDS_6FET_MODULE_ID, 				/* Индентификатор модуля UMDK-6FET */
				PWM_SETTINGS, 						/* Команда настройки канала ШИМ'а */
				PWM_SETTINGS_LENGTH, 				/* Размер payload'а */
				(uint8_t*)&pwm_settings_pack);		/* Payload */
}

/*---------------------------------------------------------------------------*/
/* Отправка команды включения/выключения канала ШИМ'а */
void pwm_power_channel_sender ( const uip_ip6addr_t *dest_addr, 
								uint8_t channel, 
								uint8_t pwm_power_channel)
{
	/* Заполняем payload */
	pwm_power_t pwm_power_pack;					/* Создаем структуру */
	
	pwm_power_pack.pwm_power = channel;			/* Номер канала */
	
	if(pwm_power_channel)						/* Если включить, то устанавливаем старший бит в единицу */
		pwm_power_pack.pwm_power |= 0x80;			
			
	/* Отправляем пакет */
	pack_sender(dest_addr, 						/* Адрес модуля UMDK-6FET */
				UNWDS_6FET_MODULE_ID, 			/* Индентификатор модуля UMDK-6FET */
				PWM_POWER, 						/* Команда включения канала ШИМ'а */
				PWM_POWER_LENGTH, 				/* Размер payload'а */
				(uint8_t*)&pwm_power_pack);		/* Payload */
}

/*---------------------------------------------------------------------------*/
/* Конструктор пакета из UART */
static void send_pack_from_cr(uint8_t* data)
{
	/* Отражаем структуры на массивы */ 
	uart_header_t *uart_header_pack = (uart_header_t*)&data[2];
	uint16_t *data_uart_len = (uint16_t*)data;

	// /* Для отладки. Выводит содержимое пакета */
	// printf("uart_event_message: ");	
	// hexraw_print(*len, (uint8_t*)&data[2]);
	// printf("\n");

	// printf("uart_header_pack->dest_addr: ");
	// uip_debug_ipaddr_print((uip_ip6addr_t*)&uart_header_pack->dest_addr);
	// printf("\n");
	// printf("uart_header_pack->device_id: %i\n", uart_header_pack->device_id);
	// printf("uart_header_pack->data_type: %i\n", uart_header_pack->data_type);
	// printf("uart_header_pack->payload_len: %i\n", uart_header_pack->payload_len);

	// printf("len = %i\n", *data_uart_len);
	// uint16_t crc16 = crc16_arc(&data[2], *data_uart_len - 2); 
	// printf("crc = 0x%04x\n", crc16);
	// printf("crc = 0x%04x\n", *(uint16_t*)(&data[*data_uart_len]));
	
	if ((crc16_arc(&data[2], *data_uart_len - 2)) == (*(uint16_t*)(&data[*data_uart_len]))) {
		/* Отправляем пакет */ 
		pack_sender(&(uart_header_pack->dest_addr),	/* Адрес модуля */
				uart_header_pack->device_id,		/* Индентификатор модуля */
				uart_header_pack->data_type, 		/* Команда */
				uart_header_pack->payload_len, 		/* Размер payload'а */
				(uint8_t*)&data[22] );				/* Payload */

		puts("[ROOT Node] Pack from coordinator sent");
	}
	else
	{
		puts("[ROOT Node] Pack from coordinator don't sent. ERROR CRC");
	}
}

/*---------------------------------------------------------------------------*/
/*Вывод принятого пакета микрокомпьютеру*/ 
static void print_cr(const uip_ip6addr_t *dest_addr, 
					 uint8_t *data, 
					 uint16_t length)
{
	/* Ожидаем завершения передачи */
	while(ti_lib_uart_busy(UART0_BASE));

	/* Отсоединяем пин от UART'а */ 
	ti_lib_gpio_set_dio(BOARD_IOID_UART_SHELL_TX);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_UART_SHELL_TX, GPIO_OUTPUT_ENABLE);
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_SHELL_TX, IOC_PORT_GPIO, IOC_OUTPUT_PULL_UP);

	/* Присоединяем пин к UART'у */ 
	ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);

	/* Небольшая задержка для того что бы UART успел переключиться */
	clock_wait(CLOCK_SECOND * 0.0025);

	/* Выводим адрес отправителя */
	for (uint8_t i = 0; i < sizeof(uip_ip6addr_t); i++)	
	{
		while(!ti_lib_uart_char_put_non_blocking(UART0_BASE, dest_addr->u8[i]));
	}
	
	/* Выводим принятый пакет */
	for (uint8_t i = 0; i < length; i++)	
	{
		while(!ti_lib_uart_char_put_non_blocking(UART0_BASE, data[i]));
	}
	
	/* Ожидаем завершения передачи */
	while(ti_lib_uart_busy(UART0_BASE));
	
	/* Отсоединяем пин от UART'а */ 
	ti_lib_gpio_set_dio(BOARD_IOID_UART_COORDINATOR_TX);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_UART_COORDINATOR_TX, GPIO_OUTPUT_ENABLE);
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_COORDINATOR_TX, IOC_PORT_GPIO, IOC_OUTPUT_PULL_UP);

	/* Присоединяем пин к UART'у */ 
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_SHELL_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
}

/*---------------------------------------------------------------------------*/
/* Иннициализация RPL */
void rpl_initialize()
{
	/* Устанавливаем режим работы MASH */
	rpl_set_mode(RPL_MODE_MESH);

	static uip_ipaddr_t ipaddr;
	
	/* Инициализируем нулевой адрес */
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
/* Иннициализация ноды */
void root_node_initialize()
{
	/* Устанавливаем обработчик входящих UDP данных */
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, root_udp_data_receiver);

	/* Мигнуть светодиодом */
	led_blink(LED_A);
	led_blink(LED_A);

	/* Запускаем главный процес */
	process_start(&main_root_process, NULL);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ФУНКЦИИ DAG'а */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Обработчик принятых пакетов */
static void dag_udp_data_receiver(struct simple_udp_connection *c,
								  const uip_ipaddr_t *sender_addr,
								  uint16_t sender_port,
								  const uip_ipaddr_t *receiver_addr,
								  uint16_t receiver_port,
								  const uint8_t *data, 
								  uint16_t datalen)
{
	/* Отражаем структуру на массив */ 
	header_up_t *header_up_pack = (header_up_t*)&data[HEADER_OFFSET];

	/* Вывод информационного сообщения в консоль */
	// printf("[DAG Node]: UDP crypto packet received(%"PRIu8"): ", datalen);
	// for (uint16_t i = 0; i < datalen; i++)	/* Выводим принятый пакет */ 
		// printf("%"PRIXX8, data[i]);
	// printf("\n");
	
	/* Проверяем версию протокола */ 
	if(header_up_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{
		/* Проверяем ID модуля и тип пакета */ 
		if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == JOIN_STAGE_2))
		{
			/* Третья стадия авторизации */
			join_stage_3_sender(sender_addr, data, datalen);
			
			led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
			return;
		}
		
		else if((header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_up_pack->data_type == JOIN_STAGE_4))
		{
			/* Обработчик четвертой стадии авторизации */
			join_stage_4_handler(sender_addr, data, datalen);
			
			led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
			return;
		}
		
		else 
		{	
			/* Расшифровываем данные */
			aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], iterator_to_byte(datalen - HEADER_UP_LENGTH));

			/*Вывод информационного сообщения в консоль*/
			// printf("DAG Node: UDP no crypto packet received(%"PRIu8"): ", datalen);
			// for (uint16_t i = 0; i < datalen; i++)	/* Выводим принятый пакет */ 
				// printf("%"PRIXX8, data[i]);
			// printf("\n");
			
			/* Отражаем структуру на массив */ 
			header_down_t *header_down_pack = (header_down_t*)&data[HEADER_DOWN_OFFSET];
			
			/* CRC16 проверка */ 
			if(header_down_pack->crc.u16 != crc16_arc((uint8_t*)&data[PAYLOAD_OFFSET], header_down_pack->length.u16))
			{
				/* Вывод сообщения об ошибке целостности пакета */
				printf("[DAG Node] CRC16 Error!\n");
				
				led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
				return;
			}
			
			/* Защита от атаки повтором */
			/* Проверяем счетчик пакетов на валидность данного пакета */
			if(packet_counter_root.u16 >= header_down_pack->counter.u16)
			{	
				/* Вывод сообщения об ошибке счетчика пакетов */
				printf("[DAG Node] Counter error!\n");

				led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
				return;
			}
			
			/* Обновляем значение счетчика ROOT'а */
			packet_counter_root.u16 = header_down_pack->counter.u16;	
			
			/* Проверяем ID модуля */ 
			/* UNWDS-6LOWPAN_SYSTEM */
			if(header_up_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
			{
				if(header_up_pack->data_type == PONG)
				{
					/* Pong */
					pong_handler(sender_addr, (pong_t*)&data[PAYLOAD_OFFSET]);
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}

				else if(header_up_pack->data_type == START_OTA)
				{
					process_start(&ota_process, (start_ota_t*)&data[PAYLOAD_OFFSET]);
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}

				else if(header_up_pack->data_type == DATA_FOR_OTA)
				{
					process_post(PROCESS_BROADCAST, ota_event_message, (data_for_ota_t*)&data[PAYLOAD_OFFSET]);
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}

				else
				{
					/* Вывод сообщения об неизвестной команде */
					printf("[DAG Node] Unknown command for system!\n");
			
					/* Отправляем пакет об ошибке */
					nack_sender(header_down_pack->counter.u16);
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}
					
			}

			/* UMDK-6FET */ 
			if(header_up_pack->device_id == UNWDS_6FET_MODULE_ID)
			{
				if(header_up_pack->data_type == PWM_SET)
				{
					/* Включение/выключение канала ШИМ'а */
					if(dag_pwm_set((pwm_set_t*)&data[PAYLOAD_OFFSET]))
						ack_sender(header_down_pack->counter.u16);		/* Отправляем пакет о подтверждении */
					else
						nack_sender(header_down_pack->counter.u16);		/* Отправляем пакет об ошибке */
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}

				else
				{
					/* Вывод сообщения об неизвестной команде */
					printf("[DAG Node] Unknown command for UMDK-6FET!\n");
					
					/* Отправляем пакет об ошибке */
					nack_sender(header_down_pack->counter.u16);
					
					led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
					return;
				}
			}

			else
			{
				/* Вывод сообщения о неизвестном модуле */
				printf("[DAG Node] Unknown module!\n");
				
				/* Отправляем пакет об ошибке */
				nack_sender(header_down_pack->counter.u16);
				
				led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
				return;
			}
		}
	}
	
	else
	{
		/* Вывод сообщения о неизвестном протоколе */
		printf("[DAG Node] Unknown protocol version!\n");
		
		led_mode_set(LED_FLASH);	/* Мигаем светодиодом */
		return;
	}
}

/*---------------------------------------------------------------------------*/
/* Первая стадия авторизации */
/* Передаем метадату текущей прошивки */
static void join_stage_1_sender(const uip_ipaddr_t *dest_addr)
{
	/* Проверка на то что передан существующий адрес */
	if (dest_addr == NULL)
		return;

	uip_ipaddr_t addr;						/* Выделяем память для адреса на который отправится пакет */
	uip_ip6addr_copy(&addr, dest_addr);		/* Копируем адрес */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send join packet to DAG-root node: ");
	uip_debug_ipaddr_print(&addr);
	printf("\n");

	/* Выделяем память под пакет. Общий размер пакета (header + payload) */	
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH];
	
	/* Отражаем структуры на массив */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_1;						/* Тип пакета */  
	header_pack->rssi = get_parent_rssi();						/* RSSI */ 
	header_pack->temperature = get_temperature();				/* Температура */ 
	header_pack->voltage = get_voltage();						/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_node.u16;			/* Счетчик пакетов */ 
	header_pack->length.u16 = JOIN_STAGE_1_LENGTH;				/* Размер пакета */

	/* Payload */
	OTAMetadata_t ota_metadata;
	get_current_metadata(&ota_metadata);

	// printf("\nOTA METADATA:\n");
	// print_metadata(&ota_metadata);
	// printf("\n");
	join_stage_1_pack->ota_metadata = ota_metadata;
	
	/* CRC16 */ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], header_pack->length.u16);
	
	/* Отправляем пакет */ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH), &addr);
}

/*---------------------------------------------------------------------------*/
/* Третья стадия авторизации */
/* Получаем nonce и отправляем его root'у зашифрованым AES128-CBC */
static void join_stage_3_sender(const uip_ipaddr_t *dest_addr,
								const uint8_t *data,
								uint16_t datalen)
{
	/* Проверка на то что передан существующий адрес */
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;						/* Выделяем память для адреса на который отправится пакет */
	uip_ip6addr_copy(&addr, dest_addr);		/* Копируем адрес */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send join packet stage 3 to DAG-root node:");
	uip_debug_ipaddr_print(&addr);
	printf("\n");

	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH];	
	
	/* Отражаем структуры на массивы */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&data[PAYLOAD_OFFSET];
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_3;						/* Тип пакета */  
	header_pack->rssi = get_parent_rssi();						/* RSSI */ 
	header_pack->temperature = get_temperature();				/* Температура */ 
	header_pack->voltage = get_voltage();						/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_node.u16;			/* Счетчик пакетов */ 
	header_pack->length.u16 = JOIN_STAGE_3_LENGTH;				/* Размер пакета */
	
	/* Payload */ 
	/* Расшифровываем блок */ 
	aes_ecb_decrypt((uint32_t*)aes_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET]);
	
	/* CRC16 */ 
	header_pack->crc.u16 = 0;
	// header_pack->crc.u16 = crc16_arc((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], header_pack->length.u16);
	
	/* Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC) */
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = join_stage_2_pack->nonce.u8[1];	
		nonce_key[i+1] = join_stage_2_pack->nonce.u8[0];	
	}
	
	/* Отправляем ROOT'у nonce на еденицу больше для того что бы он был уверен что у нас одинаковое шифрование */ 
	join_stage_3_pack->nonce.u16 = join_stage_2_pack->nonce.u16 + 1; /* Увеличиваем nonce на единицу: nonce += 1 */	
	
	/* Дозаполняем блок для шифрования нулями */ 
	for(uint8_t i = JOIN_STAGE_3_LENGTH; i < (JOIN_STAGE_3_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/* CRC16 */ 
	header_pack->crc.u16 = crc16_arc((uint8_t*)&join_stage_3_pack, sizeof(join_stage_3_pack));
	
	/* Зашифровываем данные */
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&udp_buffer[HEADER_DOWN_OFFSET], (uint32_t*)&udp_buffer[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);		
	
	/* Отправляем пакет */ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_DOWN_OFFSET + JOIN_STAGE_3_PAYLOAD_LENGTH), &addr);
}

/*---------------------------------------------------------------------------*/
/* Обработчик четвертой стадии авторизации */
/* Сравниваем данные пришедшие от root'а для того что бы удостоверится в том что используются правильные настройки шифрования */
static void join_stage_4_handler(const uip_ipaddr_t *sender_addr,
								 const uint8_t *data,
								 uint16_t datalen)
{	
	/* Проверка на то что передан существующий адрес */
	if (sender_addr == NULL)
		return;

	/* Отражаем структуры на массивы */ 
	header_t *header_pack = (header_t*)&data[HEADER_OFFSET];
	join_stage_4_t *join_stage_4_pack = (join_stage_4_t*)&data[PAYLOAD_OFFSET];
	
	/* Расшифровываем данные */
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)&data[HEADER_DOWN_OFFSET], CRYPTO_1_BLOCK_LENGTH);
	
	/* Проверяем массив. Если все нули, то авторизация прошла успешно */
	if(join_stage_4_pack->status_code)
	{
		packet_counter_root.u16 = header_pack->counter.u16;				/* Сохраняем счетчик пакетов ROOT'а */ 
		uip_ipaddr_copy(&root_addr, sender_addr); 						/* Копируем адрес ROOT'а с которым авторизировались */ 
		packet_counter_node.u16 = 1;									/* Инициализируем счетчик пакетов */
		etimer_set(&maintenance_timer, 0);								/* Устанавливаем таймер */ 
		process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);	/* Передаем управление dag_node_process */ 
		process_start(&ping_process, NULL);
		
		/* Устанавливаем флаг OTA */
		if(read_fw_flag() == FW_FLAG_NEW_IMG_INT)
		{
			write_fw_flag(FW_FLAG_PING_OK);
			printf("[DAG Node] Set flag to PING_OK\n");
		}

		return;
	}
	
	/* Выводим: Ошибка авторизации */
	printf("[DAG Node] Authorisation Error\n"); 
}

/*---------------------------------------------------------------------------*/
/* Ping */
static void ping_sender(void)
{
	/* Заполняем payload */
	ping_t ping_pack;							/* Создаем структуру */
	
	ping_pack.nonce.u8[1] = nonce_key[0];		/* Заполняем nonce */
	ping_pack.nonce.u8[0] = nonce_key[1];		/* Заполняем nonce */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send ping packet\n");
	
	/* Отправляем пакет */	
	pack_sender((const uip_ip6addr_t *)&root_addr,  /* Адрес ROOT'а */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID,		/* ID модуля */
				PING, 								/* Команда ping */
				PING_LENGTH,						/* Размер payload'а */
				(uint8_t*)&ping_pack); 				/* Payload */
}

/*---------------------------------------------------------------------------*/
/* Pong */
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
		/* Выводим: Ошибка авторизации */
		printf("[DAG Node] Crypto Error\n Reboot...\n");
		watchdog_reboot();
		
		// node_mode = MODE_JOIN_PROGRESS; 	/*Установка режима работы устройства*/
		// process_exit(&maintenance_process);
		// process_start(&maintenance_process, NULL);
	}
}

/*---------------------------------------------------------------------------*/
/* ACK */
static void ack_sender(uint16_t counter)
{
	/* Заполняем payload */
	ack_t ack_pack;								/* Создаем структуру */
	
	ack_pack.counter.u16 = counter;				/* Заполняем counter */
		
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send ACK packet\n");
	
	/* Отправляем пакет */	
	pack_sender((const uip_ip6addr_t *)&root_addr,  /* Адрес ROOT'а */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID, 	/* ID модуля */
				ACK, 								/* Команда ACK */
				ACK_LENGTH, 						/* Размер payload'а */
				(uint8_t*)&ack_pack);				/* Payload */			
}

/*---------------------------------------------------------------------------*/					
/* NACK */
static void nack_sender(uint16_t counter)
{
	/* Заполняем payload */
	nack_t nack_pack;							/* Создаем структуру */
	
	nack_pack.counter.u16 = counter;			/* Заполняем counter */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send NACK packet\n");
	
	/* Отправляем пакет */	
	pack_sender((const uip_ip6addr_t *)&root_addr,  /* Адрес ROOT'а */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID,		/* ID модуля */
				NACK, 								/* Команда ACK */
				NACK_LENGTH, 						/* Размер payload'а */
				(uint8_t*)&nack_pack);				/* Payload */		
}

/*---------------------------------------------------------------------------*/
/* Команда включения/выключения канала ШИМ'а c заданным duty cycle */
static bool dag_pwm_set(pwm_set_t *pwm_set_pack)
{

	uint8_t state = pwm_set_pack->state;
	// printf("[debug] Pointer: 0x%08lx State: %x\n", pointer_on_last_state, state);

	/* Сохраняем значение во flash */
	save_last_state_of_lighting(state);
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Relay: %s, PWM duty: %i%%\n", (pwm_set_pack->state >> 7) ? "on" : "off", (pwm_set_pack->state & 0x7F));
	
	/* Конфигурируем пин который управляет реле как high-drive выход */
	ti_lib_ioc_port_configure_set(BOARD_IOID_RELAY, IOC_PORT_GPIO, IOC_OUTPUT_HIGH_DRIVE_INV);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_RELAY, GPIO_OUTPUT_ENABLE);

	if(pwm_set_pack->state >> 7)
	{
		/* On */
		ti_lib_gpio_set_dio(BOARD_IOID_RELAY);
	}
	else
	{
		/* Off */
		ti_lib_gpio_clear_dio(BOARD_IOID_RELAY);
	}

	bool pwm_config_res = pwm_config(0, 500, (pwm_set_pack->state & 0x7F), BOARD_IOID_PWM); //ch, frec, duty, pin
	if(pwm_config_res)
		return pwm_start(0);
	else 
		return false;

	//pwm_stop(0);
}

/*---------------------------------------------------------------------------*/
/* Команда запроса блока данных для OTA */
static void dag_ota_req_data_sender(uint16_t ota_block)
{
	/* Создаем структуру */
	req_data_for_ota_t req_data_for_ota_pack;		
	
	/* Заполняем payload */
	req_data_for_ota_pack.ota_block = ota_block;	/* Номер запрашиваемого блока данных */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Request %i block data for OTA\n", ota_block);
	
	/* Отправляем пакет */	
	pack_sender((const uip_ip6addr_t *)&root_addr,  /* Адрес ROOT'а */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID,		/* ID модуля */
				REQ_DATA_FOR_OTA, 					/* Команда запроса блока данных для OTA */
				REQ_DATA_FOR_OTA_LENGTH, 			/* Размер payload'а */
				(uint8_t*)&req_data_for_ota_pack);	/* Payload */	
}

/*---------------------------------------------------------------------------*/
/* Команда отправки сообщения что прошивка обновлена */
static void dag_finish_ota_sender(int8_t status)
{	
	/* Создаем структуру */
	finish_ota_t finish_ota_pack;		
	
	/* Заполняем payload */
	finish_ota_pack.status = status;	/*  */

	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Finish OTA\n");
	
	/* Отправляем пакет */	
	pack_sender((const uip_ip6addr_t *)&root_addr,  /* Адрес ROOT'а */
				UNWDS_6LOWPAN_SYSTEM_MODULE_ID,		/* ID модуля */
				FINISH_OTA, 						/* Команда отправки сообщения что прошивка обновлена */
				FINISH_OTA_LENGTH, 					/* Размер payload'а */
				(uint8_t*)&finish_ota_pack);		/* Payload */	
}

/*---------------------------------------------------------------------------*/
/* Функция восстановления последнего состояния освещения */
static void restore_last_state_of_lighting(void)
{
	pointer_on_last_state = 0x0001C000;

	if(*((uint8_t*)(pointer_on_last_state)) == 0xFF)
	{
		printf("[DAG Node] Init flash for save last state of lighting\n");
		// printf("[DAG Node] Pointer: 0x%08lx, state: %x\n", pointer_on_last_state, *((uint8_t*)(pointer_on_last_state)));

		/* Установка первого состояния */
		uint8_t state = 0x00;
		ti_lib_flash_program(&state, pointer_on_last_state, sizeof(uint8_t));
		dag_pwm_set((pwm_set_t*)&state);

		return;
	}

	for(pointer_on_last_state = 0x0001C001; pointer_on_last_state < 0x0001D000; pointer_on_last_state++)
	{
		if(*((uint8_t*)(pointer_on_last_state)) == 0xFF)
		{
			break;
		}
	}

	pointer_on_last_state--;
	uint8_t state = *((uint8_t*)(pointer_on_last_state));

	// printf("[restore_last_state_of_lighting] Pointer: 0x%08lx, state: %x\n", pointer_on_last_state, *((uint8_t*)(pointer_on_last_state)));
	
	/* Установка состояния */
	dag_pwm_set((pwm_set_t*)&state);
}

/*---------------------------------------------------------------------------*/
/**/
static void save_last_state_of_lighting(uint8_t state)
{
	if(pointer_on_last_state == 0)
		return;

	if(state == *((uint8_t*)(pointer_on_last_state)))
		return;
	
	pointer_on_last_state++;

	if(pointer_on_last_state == 0x0001D000)
	{
		/* Стираем страницу */
		ti_lib_flash_sector_erase(START_USER_FLASH);
		pointer_on_last_state = 0x0001C000;
	}

	ti_lib_flash_program(&state, pointer_on_last_state, sizeof(uint8_t));
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ОБЩИЕ ПРОЦЕССЫ */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Процесс инициализации настроек из EEPROM */
PROCESS_THREAD(settings_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	/* Считываем данные из EEPROM в структуру */
	read_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
	
	/* Если настроек нет, то устанавливаем эти */
	if(eeprom_settings.aes_key_configured == true)
	{
		if((eeprom_settings.channel != 26) && (eeprom_settings.panid != 0xAABB))
		{
			eeprom_settings.channel = 26;
			eeprom_settings.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
		}
		
		printf("[Node] AES-128 key not declared\n[DAG Node] ***PLEASE SET AES KEY***\n");
		while(eeprom_settings.aes_key_configured)
		{
			PROCESS_YIELD();
		}
	}
	
	/* Если ключа шифрования нет, то информируем об этом */
	else
	{
		printf("[Node] AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_settings.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");		
	}
	
	radio_value_t channel;										/* Выделяем память под переменную channel */
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);	/* Считываем в channel текущий канал */
	
	/* Если текущий канал отличается от настроенного в EEPROM, изменяем канал на тот что из EEPROM */
	if(channel != eeprom_settings.channel)
	{
		/* Устанавливаем канал */
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_settings.channel);
		
		/* Если мы чип CC26XX, то выводим частоту */
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_settings.channel - 11));
			printf("[Node] Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_settings.channel, freq_mhz);
		}

		/* Если мы чип CC13XX, то выводим частоту */
		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_settings.channel * 200);
			printf("[Node] Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_settings.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;								/* Выделяем память под panid */
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);	/* Считываем в PAN ID */
		
		/* Если текущий PAN ID отличается от настроенного в EEPROM, изменяем PAN ID на тот что из EEPROM */
		if(panid != eeprom_settings.panid)
		{
			/* Устанавливаем PAN ID */
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_settings.panid);
			
			/* Выводим PAN ID */
			printf("[Node] PAN ID changed to: %"PRIXX16"\n", eeprom_settings.panid);
		}
	}

	/* Copy the current firmware into OTA slot 0 as the "Golden Image" */
	if(eeprom_settings.is_backup_golden_image)
	{
		/* Проверяем установлена ли флешка */
		bool eeprom_access = ext_flash_open();
		if(eeprom_access)
		{
			/* Записываем на флешку GOLDEN IMAGE */
			backup_golden_image();

			/* Очищаем флешку остальную часть флешки */
			for (uint8_t page = 2; page < 8; page++)
			{
				printf("[Node] Erasing page %i from 0x%08x\n", page, (page * 0x10000));
				erase_extflash_page(page * 0x10000);
			}
			ext_flash_close();

			eeprom_settings.is_backup_golden_image = 0;
			write_eeprom((uint8_t*)&eeprom_settings, sizeof(eeprom_settings));
		}
		else
		{
			ext_flash_close();
		}
	}
	
	/* Чтение GPIO и установка режима работы */
	// ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_BOOT_MODE); 
	// mode_node = ti_lib_gpio_read_dio(BOARD_IOID_BOOT_MODE);

	/* Конфигурируем вход как input pull down */
	ti_lib_ioc_port_configure_set(BOARD_IOID_BOOT_MODE, IOC_PORT_GPIO, IOC_INPUT_PULL_DOWN);
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_BOOT_MODE, GPIO_OUTPUT_ENABLE);

	/* Чтение GPIO и установка режима работы */
	mode_node = ti_lib_gpio_read_dio(BOARD_IOID_BOOT_MODE);

	/* Убираем подтяжку и выключаем GPIO*/
	ti_lib_ioc_port_configure_set(BOARD_IOID_BOOT_MODE, IOC_PORT_GPIO, IOC_STD_INPUT); 
	ti_lib_gpio_set_output_enable_dio(BOARD_IOID_BOOT_MODE, GPIO_OUTPUT_DISABLE);

	/* Проверяем состояние перемычки и устанавливаем режим работы: ROOT или NODE */
	if(!node_is_root())
		restore_last_state_of_lighting();	/* Если загрузились в режиме ноды, то устанавливаем последние состояние */

	/* Передаем управление main_process */
	process_post(&main_process, PROCESS_EVENT_CONTINUE, NULL);

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/* Процесс упарвления светодиодами */
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

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ПРОЦЕССЫ ROOT'а */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Процесс управления ROOT'ом */
PROCESS_THREAD(main_root_process, ev, data)
{
	PROCESS_BEGIN();

	packet_counter_root.u16 = 0;						/* Обнуляем счетчик пакетов */
	PROCESS_PAUSE();									/* Небольшая задержка */

	/* Цикл который ожидает события uart_event_message */
	while(1) 							
	{
		PROCESS_YIELD(); 
		if(ev == uart_event_message) 
		{
			/* Конструктор пакета из UART */
			send_pack_from_cr(data);
		} 
	}

	PROCESS_END();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ПРОЦЕССЫ DAG'а */
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Процесс опроса ROOT'а на достижимость */
PROCESS_THREAD(ping_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;
	
	static struct etimer ping_timer;							/* Создаём таймер для по истечении которого будет ROOT будет пинговаться */
	
	while (1)
	{
		etimer_set(&ping_timer, (CLOCK_SECOND * 10));			/* Устанавливаем таймер на 10 минут */
		
		if(non_answered_ping > 3)								/* Перезагрузить если больше трех неотвеченных пингов */
		{
			printf("[DAG Node] Ping error!\nReboot...");
			watchdog_reboot();
		}
		
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ping_timer));	/* Засыпаем до срабатывания таймера */
		
		non_answered_ping++;									/* Увеличиваем на еденицу. При ответе в pong_handler() должно обнулиться */		
		ping_sender();											/* Отправляем ping */
	}
	
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/* Процесс отслеживает нажатие кнопки. При нажатии происходит перезагрузка */
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
				/* Включаем светодиод */
				led_mode_set(LED_ON);	
				
				/* Вывод информационного сообщения в консоль */
				printf("[DAG Node] Button E long click, reboot\n");
				
				watchdog_reboot();
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/* Процесс управления нодой */
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
			led_mode_set(LED_OFF);	/* Выключаем светодиод */

			if(process_is_running(&root_find_process) == 1)
				process_exit(&root_find_process);
		}

		if(node_mode == MODE_NOTROOT)
		{
			if(CLASS == CLASS_B)
			{
				led_mode_set(LED_OFF);
				
				/* Вывод информационного сообщения в консоль */
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
				/* Мигаем светодиодом */
				led_mode_set(LED_FAST_BLINK);
				
				/* Вывод информационного сообщения в консоль */
				printf("[DAG Node] Root not found\n Reboot...\n"); //почему-то не перезагружается!
				
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
/* Процесс поиска ROOT'а */
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
						led_mode_set(LED_FAST_BLINK);	/* Мигаем светодиодом */
						join_stage_1_sender(&root_find_dag->dag_id);
					}
				}
			}
			else
			{
				node_mode = MODE_NOTROOT;
				
				/* Вывод информационного сообщения в консоль */
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
/* Запуск инициализации ноды (точка входа) */
PROCESS_THREAD(dag_node_process, ev, data)
{
	PROCESS_BEGIN();
	PROCESS_PAUSE();

	/* Инициализация обработчика входящих пакетов */
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, dag_udp_data_receiver);

	/* Выбор режима работы RPL */
	if (CLASS == CLASS_B)
		rpl_set_mode(RPL_MODE_LEAF);
	else
		rpl_set_mode(RPL_MODE_MESH);

	
	node_mode = MODE_JOIN_PROGRESS; 	/* Установка начального режима работы устройства */
	packet_counter_node.u16 = 1;		/* Инициализация счетчика */
	
	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Node started, %s mode, %s class, version %"PRIu8".%"PRIu8"\n",
			rpl_get_mode() == RPL_MODE_LEAF ? "leaf" : "no-leaf",
			CLASS == CLASS_B ? "B(sleep)" : "C(non-sleep)",
			BIG_VERSION, 
			LITTLE_VERSION);

	process_start(&dag_node_button_process, NULL);		/* Запускаем процес который отслеживает нажатие кнопок */
	process_start(&maintenance_process, NULL);			/* Запускаем процес управления нодой */

	SENSORS_ACTIVATE(batmon_sensor);					/* Инициализация встроенного датчика температуры и напряжения процессора */

	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);	/* Ожидаем пока нода авторизируется в сети */

	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] DAG active, join stage 4 packet received, mode set to MODE_NORMAL\n");
	
	led_mode_set(LED_SLOW_BLINK);	/* Включаем медленное мигание светодиодами */
	node_mode = MODE_NORMAL;		/* Изменение режима работы ноды. Нода работает в нормальном режиме */
	net_mode(RADIO_FREEDOM);		/*  */
	net_off(RADIO_OFF_NOW);			/*  */

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/* Процесс обновления по воздуху */
PROCESS_THREAD(ota_process, ev, data)
{
	PROCESS_BEGIN();
	
	ota_event_message = process_alloc_event();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;

	if(data == NULL)
		return 1;

	/* Получаем метадату до которой обновляемся */
	OTAMetadata_t ota_metadata;
	memcpy(&ota_metadata, &((start_ota_t*)data)->ota_metadata, sizeof(OTAMetadata_t));
	printf("[OTA] Metadata:\n");
	print_metadata(&ota_metadata);

	blocks_counter = 0;

	if(ota_metadata.size > (ota_metadata.size / 256) * 256)
		num_blocks = (ota_metadata.size / 256) + 1;
	else
		num_blocks = (ota_metadata.size / 256);

	/* Вывод информационного сообщения в консоль */
	printf("[OTA] Number of blocks: %i\n", num_blocks + 1);

	/* Создаём таймер для по истечении которого будет запрашиваться пакет с данными для OTA */
	static struct etimer req_data_for_ota_timer;

	while(erase_ota_image(2));
	
	/* Запрос блока данных для OTA */
	dag_ota_req_data_sender(blocks_counter);

	/* Цикл который ожидает события ota_event_message */
	while (1)
	{
		PROCESS_YIELD(); 
		if(ev == ota_event_message) 
		{
			/* Отражаем структуры на массивы */ 
			data_for_ota_t *data_for_ota_pack = (data_for_ota_t*)&((uint8_t*)data)[0];

			if(data_for_ota_pack->ota_block == blocks_counter)
			{
				/* Вывод информационного сообщения в консоль */
				printf("[OTA] Received %i block\n", data_for_ota_pack->ota_block);
				// hexraw_print(256, (uint8_t*)(&data_for_ota_pack->data_for_ota));
				// printf("\n");
				// printf("Address: 0x%08x\n", (ota_images[1] << 12) + (data_for_ota_pack->ota_block * 256));
				
				/* Сохраняем блок с данными для OTA в EEPROM */
				store_firmware_data((ota_images[1] << 12) + (data_for_ota_pack->ota_block * 256), 
									(uint8_t*)(&data_for_ota_pack->data_for_ota), 
									256);
				
				/* Инкрементируем счетчик блоков */
				blocks_counter++;
			}
			else
			{
				/* Вывод информационного сообщения в консоль */
				printf("[OTA] ERROR NUM BLOCK!!! Took %i block instead of %i\n", data_for_ota_pack->ota_block, blocks_counter);
			}
		} 

		/* Завершение обновления по воздуху */
		if(blocks_counter > num_blocks)
		{
			int8_t verify_result_ota_2 = verify_ota_slot(2);

			/* Отправка о завершении OTA */
			dag_finish_ota_sender(verify_result_ota_2);

			/* Вывод информационного сообщения в консоль */
			if (verify_result_ota_2 == VERIFY_SLOT_OK)
			{
				printf("[OTA] Correct CRC!\n");
				printf("[DAG Node] Set flag to NEW_IMG_EXT\n");
				write_fw_flag(FW_FLAG_NEW_IMG_EXT);
				printf("[DAG Node] Reboot...\n");
				watchdog_reboot();
			}
			else if (verify_result_ota_2 == VERIFY_SLOT_CRC_ERROR)
				printf("[OTA] Non-correct CRC!\n");
			else
				printf("[OTA] Unknown error!\n");

			process_exit(&ota_process);
			return 1;
		}

		/* Запрос блока данных для OTA */
		dag_ota_req_data_sender(blocks_counter);

		/* Устанавливаем таймер на REQ_DATA_FOR_OTA_INTERVAL */
		etimer_set(&req_data_for_ota_timer, REQ_DATA_FOR_OTA_INTERVAL);		

		/* Засыпаем до любого события */
		// PROCESS_WAIT_EVENT(); 	
	
		/* Засыпаем до срабатывания таймера */
		// PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&req_data_for_ota_timer));	
	}
	
	PROCESS_END();
}