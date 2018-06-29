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

#include "uart/root.h"

#define DIO_MASK				0x3F
#define STATUS_MASK				0xC0
#define CLICK					0x80
#define LONG_CLICK				0xC0

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

#define WAIT_RESPONSE 			3 	//Максимальное время ожидания ответа от счетчика в секундах

/*---------------------------------------------------------------------------*/
/*ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ*/

uint8_t uart_rx_buffer[128];			/*Буфер UART RX*/

uint8_t aes_key[16];					/*Ключ шифрования*/
uint8_t aes_buffer[128];				/*Буфер для операций шифрования*/
uint8_t nonce_key[16];					/*Сессионный ключ*/

static eeprom_t eeprom_root;			/*Настройки из EEPROM*/

static uint16_t data_iterator = 0;		/*Счетчик принятых байтов из UART*/
static struct timer timeout_timer;		/*Таймер срабатываем на пришедший по UART байт и перезапускается.*/
// static struct ctimer wait_response;		/*Таймер который считает время ожидания ответа от DAG'а*/

static bool uart = 0;					/*Отражает режим работы UART'а (UART или консоль)*/
// static bool wait_response_slave = 0;	/*Отражает ожидаем ли мы ответа от DAG'а*/

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
static void pong_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

/*Обработчик нажатой кнопки*/
static void button_status_handler(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

// /*Передает данные полученные от УСПД, счетчику по радио*/
// static void uart_to_air();

// /*Передает данные полученные от счетчика на УСПД*/
// static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length);

// /*Сбрасывает переменную которая показывает ожидаем ли мы ответа от DAG'а*/
// static void wait_response_reset(void *ptr);

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ПРОЦЕССОВ*/

/*Процесс инициализации настроек из EEPROM*/
PROCESS(settings_root_init, "Initializing settings of ROOT");

/*Процесс управления ROOT'ом*/
PROCESS(main_root_process, "main root process");

/*---------------------------------------------------------------------------*/
/*Обработчик принятых пакетов*/
void udp_data_receiver(struct simple_udp_connection *connection,
                       const uip_ipaddr_t *sender_addr,
                       uint16_t sender_port,
                       const uip_ipaddr_t *receiver_addr,
                       uint16_t receiver_port,
                       const uint8_t *data,
                       uint16_t datalen)
{
	led_on(LED_A); 								/*Включаем светодиод*/

	uip_ip6addr_t node_addr;					/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&node_addr, sender_addr);	/*Копируем адрес*/
	
	/*Отражаем структуру на массив*/ 
	header_t *header_pack = (header_t*)&data[HEADER_OFFSET];

	/*Для отладки. Выводит тип принятого сообщения*/ 
	printf("Recive pack: %x\n", header_pack->data_type);
	
	/*Вывод информационного сообщения в консоль*/
	{
		printf("Packet received(%"PRIu8"): ", datalen);
		for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
			printf("%"PRIXX8, data[i]);
		printf("\n");
	}
	
	/*Проверяем версию протокола*/ 
	if(header_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{
		/*Проверяем ID модуля*/ 
		if (header_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
		{
			/*Проверяем тип пакета*/ 
			if (header_pack->data_type == DATA_TYPE_JOIN_STAGE_1)
			{
				/*Вторая стадия авторизации*/
				join_stage_2_sender(&node_addr, data, datalen);
				led_off(LED_A);		/*Выключаем светодиод*/
				return;
			}
			
			else if(header_pack->data_type == DATA_TYPE_JOIN_STAGE_3)
			{
				/*Четвертая стадия авторизации*/
				join_stage_4_sender(&node_addr, data, datalen);
				led_off(LED_A);		/*Выключаем светодиод*/
				return;
			}
			
			else if(header_pack->data_type == PING)
			{
				/*Pong*/
				pong_sender(&node_addr, data, datalen);
				led_off(LED_A);		/*Выключаем светодиод*/
				return;
			}
			
			else if(header_pack->data_type == BUTTON_STATUS)
			{
				/*Передаём данные полученные от счетчика на УСПД*/
				// uart_from_air(&node_addr, data, datalen);
				button_status_handler(&node_addr, data, datalen);
				led_off(LED_A);		/*Выключаем светодиод*/
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
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
   
	uip_ipaddr_t addr;					/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&addr, dest_addr);	/*Копируем адрес*/
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	// join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&data[PAYLOAD_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&aes_buffer[0];
	
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
	uint16_t nonce = random_rand();				/*Генерируем сессионный ключ*/ 
	
	/*Для отладки. Выводим nonce*/ 
	// printf("Join_stage_2_sender nonce: %i\n", nonce);
	
	/*Добавляем маршрут*/ 
	add_route ( &addr,							/*Address*/ 
				nonce);							/*Nonce*/ 
						
	join_stage_2_pack->nonce.u16 = nonce;		/*Nonce*/ 
	
	/*Дозаполняем блок для шифрования нулями*/ 
	for(uint8_t i = 2; i < 16; i++)
		aes_buffer[i] = 0x00;
	
	/*Для отладки. Выводит незашифрованный payload пакета*/ 
	// printf("Join_stage_2_sender payload:\n");
	// hexraw_print(JOIN_STAGE_2_LENGTH, aes_buffer);
	// printf("\n");
	
	/*Зашифровываем блок*/ 
	aes_ecb_encrypt((uint32_t*)aes_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[PAYLOAD_OFFSET]));

	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Join_stage_2_sender pack:\n");
	// hexraw_print((HEADER_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH), udp_buffer);
	// printf("\n");
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH), &addr);
	packet_counter_root.u16++;	/*Инкрементируем счетчик пакетов*/ 
}

/*---------------------------------------------------------------------------*/
/*Четвертая стадия авторизации*/
/*Принимает nonce зашифрованный AES128-CBC. Если сходится с тем что он сгенерировал, то авторизация прошла успешно, настройки шифрования верные. Отправляем пакет с нулями что бы DAG мог убедиться в этом*/
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&addr, dest_addr);		/*Копируем адрес*/
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_4_PAYLOAD_LENGTH];	
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	// join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&data[PAYLOAD_OFFSET];
	join_stage_4_t *join_stage_4_pack = (join_stage_4_t*)&aes_buffer[0];
	
	/*Получаем nonce*/
	u8_u16_t nonce;
	nonce.u16 = get_nonce(&addr);	
	
	/*Для отладки. Выводит serial и nonce*/ 
	// printf("Join_stage_4_sender serial: %lu\n", join_stage_3_pack->serial.u32);
	// printf("Join_stage_4_sender nonce: %i\n", nonce.u16);

	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = nonce.u8[1];	
		nonce_key[i+1] = nonce.u8[0];	
	}
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[PAYLOAD_OFFSET + SERIAL_LENGTH], (uint32_t*)(aes_buffer), CRYPTO_1_BLOCK_LENGTH);
	
	/*Если nonce'ы совпадают, то авторизация прошла успешно, шифрование настроенно правильно*/ 
	if((aes_buffer[0] == nonce_key[1]) && (aes_buffer[1] == nonce_key[0]))
	{		
		unlock_addr(&addr);			/*Разрешаем обрабатывать пакеты принятые с авторизированного устройства*/ 		
	}
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = DATA_TYPE_JOIN_STAGE_4;			/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_root.u16;			/*Счетчик пакетов*/ 
	header_pack->length = JOIN_STAGE_4_LENGTH;					/*Размер пакета*/
	
	/*Payload*/ 
	/*Заполняем пакет нулями и отправляем его DAG'у. Если они после расшифровки получит массив нулей, то он считает что авторизавался и настройки шифрования верны*/ 
	for(uint8_t i = 0; i < 16; i++)
		join_stage_4_pack->array_of_zeros[i] = 0;
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[PAYLOAD_OFFSET]), CRYPTO_1_BLOCK_LENGTH);
	
	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Join_stage_4_sender pack:\n");
	// hexraw_print((HEADER_LENGTH + JOIN_STAGE_4_PAYLOAD_LENGTH), udp_buffer);
	// printf("\n");
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_4_PAYLOAD_LENGTH), &addr);
	packet_counter_root.u16++;		/*Инкрементируем счетчик пакетов*/
}

/*---------------------------------------------------------------------------*/
/*Pong*/
static void pong_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
	uip_ip6addr_copy(&addr, dest_addr);		/*Копируем адрес*/
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + PONG_PAYLOAD_LENGTH];	
	
	/*Отражаем структуры на массивы*/ 
	ping_t *ping_pack = (ping_t*)&data[PAYLOAD_OFFSET];
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	pong_t *pong_pack = (pong_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/*Получение nonce*/	
	uip_ds6_route_t *r;						/*Создаем указатель на элемент списка в котором мы ищем nonce*/
	r = uip_ds6_route_lookup(&addr);		/*Ищем nonce по IPv6 в RPL таблице*/
	
	/*Проверка на то что мы нашли данный IP адрес*/
	if(r == NULL) 
	{
		return;
	}
	
	u8_u16_t nonce; 						/*Выделяем память под nonce*/
	nonce.u16 = r->nonce;					/*Получаем nonce*/
	
	uint8_t status_code = STATUS_OK;		/*Выделяем память под статус код. В нем передаем номер ошибки*/
	
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = nonce.u8[1];	
		nonce_key[i+1] = nonce.u8[0];	
	}
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&ping_pack->array_of_zeros[0], (uint32_t*)(aes_buffer), CRYPTO_1_BLOCK_LENGTH);
	
	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Pong_sender decrypt pack:\n");
	// hexraw_print(CRYPTO_1_BLOCK_LENGTH, aes_buffer);
	// printf("\n");
	
	/*Проверяем массив. Если все нули, то настройки шифрования верны*/ 
	if(!is_array_zero(aes_buffer))
	{
		status_code = STATUS_ERROR;
	}
	
	/*Заполняем пакет*/  
	/*Header*/
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = PONG;								/*Тип пакета*/  
	header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = get_temperature();				/*Температура*/ 
	header_pack->voltage = get_voltage();						/*Напряжение*/ 
	header_pack->counter.u16 = packet_counter_root.u16;			/*Счетчик пакетов*/ 
	header_pack->length = PONG_LENGTH;							/*Размер пакета*/
	
	/*Payload*/ 
	pong_pack->status_code = status_code;						/*Статус код*/	
	
	/*Отражаем структуру на массив*/ 
	pong_pack = (pong_t*)&aes_buffer[0];
	
	for(uint8_t i = 0; i < 16; i++)
		pong_pack->array_of_zeros[i] = 0x00;
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&pong_pack->array_of_zeros[0], (uint32_t*)(&udp_buffer[PAYLOAD_OFFSET + STATUS_CODE_LENGTH]), CRYPTO_1_BLOCK_LENGTH);
	
	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Pong_sender pack:\n");
	// hexraw_print((HEADER_LENGTH + PONG_PAYLOAD_LENGTH), udp_buffer);
	// printf("\n");
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + PONG_PAYLOAD_LENGTH), &addr);
	packet_counter_root.u16++;		/*Инкрементируем счетчик пакетов*/
}

/*---------------------------------------------------------------------------*/
/*Обработчик нажатой кнопки*/
static void button_status_handler(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	/*Отражаем структуры на массивы*/ 
	button_status_t *button_status_pack = (button_status_t*)&data[PAYLOAD_OFFSET];
	
	uint8_t status = button_status_pack->button_status & STATUS_MASK;
	uint8_t dio = button_status_pack->button_status & DIO_MASK;

	if(dio == BOARD_IOID_KEY_A)
		printf("Button A ");
	else if(dio == BOARD_IOID_KEY_B)
		printf("Button B ");
	else if(dio == BOARD_IOID_KEY_C)
		printf("Button C ");
	else if(dio == BOARD_IOID_KEY_D)
		printf("Button D ");
	else if(dio == BOARD_IOID_KEY_E)
		printf("Button E ");
	else
		printf("Button huy_znaet ");
	
	if(status == CLICK)
		printf("click\n");
	else if (status == LONG_CLICK)
		printf("long click\n");
	else
		printf("kak_dolgo_click\n");
}

/*---------------------------------------------------------------------------*/


// /*---------------------------------------------------------------------------*/
// /*Передает данные полученные от УСПД, счетчику по радио*/
// static void uart_to_air() 
// {
	// uip_ipaddr_t addr;		/*Выделяем память для адреса на который отправится пакет*/
	// u8_u16_t nonce;			/*Выделяем память для nonce*/
	
	// /*Парсим серийник из данных принятых из UART от УСПД и ищем в таблице IPv6 адрес*/
	// /*Если команда меньше 7 байт, то она для однобайтно адресуемых счетчиков*/
	// if(data_iterator < 7) 
	// {
		// /*Ищем IPv6 адрес по серийному номеру*/
		// addr = find_addr((uint32_t)(uart_rx_buffer[0]));
		
		// /*Проверка на то что не это не нулевой адрес*/
		// if(uip_ip6addr_cmp(&addr, &null_addr))
		// {
			// return;
		// }
		
		// /*Получаем nonce*/
		// nonce.u16 = get_nonce((uint32_t)(uart_rx_buffer[0]));
	// }
	
	// /*В первую очередь проверяем по четырехбайтному адресу, а потом по однобайтному*/
	// else 
	// {
		// /*Ищем IPv6 адрес по серийному номеру*/
		// addr = find_addr((uint32_t)((uart_rx_buffer[0] << 24) |
									// (uart_rx_buffer[1] << 16) |
									// (uart_rx_buffer[2] << 8)  |
									 // uart_rx_buffer[3]));
		
		// /*Проверка на то что не это не нулевой адрес*/
		// if(uip_ip6addr_cmp(&addr, &null_addr))	
		// {	
			// /*Ищем IPv6 адрес по серийному номеру*/
			// addr = find_addr((uint32_t)(uart_rx_buffer[0]));

			// /*Проверка на то что не это не нулевой адрес*/
			// if(uip_ip6addr_cmp(&addr, &null_addr))
			// {
				// return;
			// }
			
			// /*Получаем nonce*/
			// nonce.u16 = get_nonce((uint32_t)(uart_rx_buffer[0]));
		// }
		// else
		// {
			// /*Получаем nonce*/
			// nonce.u16 = get_nonce((uint32_t)((uart_rx_buffer[0] << 24)|
											// (uart_rx_buffer[1] << 16) |
											// (uart_rx_buffer[2] << 8)  |
											// (uart_rx_buffer[3] )));
		// }
	// }
	
	// /*Копируем nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	// for(int i = 0; i < 16; i += 2)
	// {
		// nonce_key[i] = nonce.u8[1];	
		// nonce_key[i+1] = nonce.u8[0];	
	// }

	// /*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	// /*Нижняя часть header'а будет шифроваться. Поэтому для рассчета payload'а нужно учитывать её*/
	// uint8_t crypto_length = iterator_to_byte(data_iterator + HEADER_DOWN_LENGTH);
	// uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	// /*Отражаем структуры на массивы*/ 
	// header_up_t *header_up_pack = (header_up_t*)&udp_buffer[HEADER_OFFSET];
	// header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];	
	
	// /*Заполняем пакет*/  
	// /*Header*/ 
	// header_up_pack->protocol_version = UDBP_PROTOCOL_VERSION; 	/*Текущая версия протокола*/ 
	// header_up_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	// header_up_pack->data_type = UART_FROM_AIR_TO_TX;			/*Тип пакета*/  
	// header_up_pack->rssi = get_parent_rssi();					/*RSSI*/ 
	// header_up_pack->temperature = get_temperature();			/*Температура*/ 
	// header_up_pack->voltage = get_voltage();					/*Напряжение*/ 
	
	// /*Шифрованая часть header'а*/ 
	// header_down_pack->counter.u16 = packet_counter_root.u16;	/*Счетчик пакетов*/ 
	// header_down_pack->length = data_iterator;					/*Размер пакета*/
	
	// /*Заполняем блок для шифрования*/ 
	// for(uint8_t i = HEADER_DOWN_LENGTH; i < crypto_length; i++)
	// {
		// if(i < (data_iterator + HEADER_DOWN_LENGTH))
			// aes_buffer[i] = uart_rx_buffer[i-3];		/*Заполняем блок для шифрования данными*/ 
		// else
			// aes_buffer[i] = 0x00;						/*Дозаполняем блок для шифрования нулями*/ 
	// }
	
	// /*Зашифровываем данные*/
	// aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);
	
	// /*Отправляем пакет*/ 
	// simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), &addr);
	// packet_counter_root.u16++;		/*Инкрементируем счетчик пакетов*/
// }

// /*---------------------------------------------------------------------------*/
// /*Передает данные полученные от счетчика на УСПД*/
// static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length)
// {
	// /*Если мы не ожидаем приёма от утройства, то пакет отбрасываем*/
	// if(wait_response_slave == 0)
		// return;
	
	// /*Отражаем структуры на массивы*/ 
	// header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];
	
	// /*Расшифровываем данные*/
	// aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)(aes_buffer), (length - HEADER_UP_LENGTH));
	
	// /*Получаем из принятого сообщения серийный номер*/
	// uint32_t serial = ( (aes_buffer[3] << 24) |
						// (aes_buffer[4] << 16) |
						// (aes_buffer[5] << 8)  |
						// (aes_buffer[6] ));					
	
	// /*Проверка счетчика пакета на валидность*/
	// if(valid_counter(serial, header_down_pack->counter.u16) || valid_counter(aes_buffer[3], header_down_pack->counter.u16))
	// {
		// /*Отправка данных в UART*/
		// for(uint16_t i = 0; i < aes_buffer[2]; i++)
			// cc26xx_uart_write_byte(aes_buffer[i + 3]);
	// }
// }

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

	printf("UDM: Created a new RPL DAG, i'm root!\n");
}

/*---------------------------------------------------------------------------*/
/*Иннициализация ноды*/
void root_node_initialize()
{
	/*Устанавливаем обработчик входящих UDP данных*/
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_data_receiver);

	/*Устанавливаем обработчик входящих UART данных*/
	cc26xx_uart_set_input(&uart_data_receiver);

	/*Мигнуть светодиодом*/
	led_blink(LED_A);
	led_blink(LED_A);

	/*Запускаем главный процес*/
	process_start(&main_root_process, NULL);
}

/*---------------------------------------------------------------------------*/
/*Обработчик прерывания UART*/
int uart_data_receiver(unsigned char uart_char)
{
	/*Мигнуть светодиодом*/
	led_blink(LED_A);
	
	/*Проверка от переполнения буфера*/
	if(data_iterator < 128)
	{
		uart_rx_buffer[data_iterator] = uart_char;	/*Записываем принятый байт в буфер*/
		data_iterator++;							/*Увеличиваем счетчик принятых байтов*/ 
		timer_restart(&timeout_timer);				/*Перезапускаем таймер*/
	}
	
	return 1;
}

/*---------------------------------------------------------------------------*/
/*Устанавливает режим работы UART. Работает в режиме UART*/
void set_uart_r(void)
{
	uart = 1;
}

/*---------------------------------------------------------------------------*/
/*Устанавливает режим работы UART. Работает в режиме консоли*/
void unset_uart_r(void)
{
	uart = 0;
}

/*---------------------------------------------------------------------------*/
/*Возвращает режим работы UART*/
uint8_t uart_status_r(void)
{
	if(uart == 1)
		return 1;
	else
		return 0;
}

/*---------------------------------------------------------------------------*/
// /*Сбрасывает переменную которая показывает ожидаем ли мы ответа от DAG'а*/
// static void wait_response_reset(void *ptr)
// {
	// wait_response_slave = 0;
// }

/*---------------------------------------------------------------------------*/
/*Процесс инициализации настроек из EEPROM*/
PROCESS_THREAD(settings_root_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	/*Считываем данные из EEPROM в структуру*/
	read_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
	
	/*Если настроек нет, то устанавливаем эти*/
	if(eeprom_root.aes_key_configured == true) 
	{
		if((eeprom_root.channel != 26) && (eeprom_root.panid != 0xAABB))
		{
			eeprom_root.channel = 26;
			eeprom_root.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
		}
	}
	
	/*Если ключа шифрования нет, то информируем об этом*/
	if(!eeprom_root.aes_key_configured) 
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_root.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");
	}
	else
	{
		printf("AES-128 key not declared\n");
		while(eeprom_root.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	radio_value_t channel;										/*Выделяем память под переменную channel*/
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);	/*Считываем в channel текущий канал*/
	
	/*Если текущий канал отличается от настроенного в EEPROM, изменяем канал на тот что из EEPROM*/
	if(channel != eeprom_root.channel)
	{
		/*Устанавливаем канал*/
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_root.channel);
		
		/*Если мы чип CC26XX, то выводим частоту*/
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_root.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_root.channel, freq_mhz);
		}

		/*Если мы чип CC13XX, то выводим частоту*/
		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_root.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_root.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;								/*Выделяем память под panid*/
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);	/*Считываем в PAN ID*/
		
		/*Если текущий PAN ID отличается от настроенного в EEPROM, изменяем PAN ID на тот что из EEPROM*/
		if(panid != eeprom_root.panid)
		{
			/*Устанавливаем PAN ID*/
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_root.panid);
			
			/*Выводим PAN ID*/
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_root.panid);
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
	timer_set(&timeout_timer, 10);					/*Устанавливаем таймер*/

	static struct etimer main_root_process_timer;	/*Таймер*/
	packet_counter_root.u16 = 0;					/*Обнуляем счетчик пакетов*/
	PROCESS_PAUSE();								/*Небольшая задержка*/

	// uint16_t crc_uart;								/*Выделяем память под контрольную сумму*/

	while (1)
	{	
		/**/
		etimer_set(&main_root_process_timer, 1);
		
		/**/
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&main_root_process_timer));

		/*Проверяет поступилили к нам данные по UART*/
		if(timer_expired(&timeout_timer) && data_iterator > 0)
		{
			/*Проверяем в каком режиме работает устройство (консоль или UART)*/
			if(uart == 1)
			{
				// /*Проверка на минимальную длину*/
				// if(data_iterator > 3)
				// {
					// /*Рассчитываем CRC16-MODBUS*/
					// crc_uart = crc16_modbus(uart_rx_buffer, data_iterator-2);
					
					// /*Если контрольная сумма совпала то отправляем сообщение DAG'у*/
					// if(crc_uart == (uint16_t) ((uart_rx_buffer[data_iterator-1] << 8) | 
												// uart_rx_buffer[data_iterator-2]))
					// {
						// /*Устанавливаем время ожидание ответа от DAG'а*/
						// wait_response_slave = 1;
						// ctimer_set(&wait_response, WAIT_RESPONSE * CLOCK_SECOND, wait_response_reset, NULL);
						
						// /*Отправляем сообщение DAG'у*/
						// uart_to_air();
					// }
				// }
				
				// /*Обнуляем счетчик принятых байтов*/
				// data_iterator = 0;
			}
			else
			{
				/*Обнуляем счетчик принятых байтов*/
				data_iterator = 0;
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
