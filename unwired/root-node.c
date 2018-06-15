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

#include "ud_binary_protocol.h"
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

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

#define WAIT_RESPONSE 			3 	//Максимальное время ожидания ответа от счетчика в секундах

/*---------------------------------------------------------------------------*/

uint8_t uart_rx_buffer[127];

uint8_t aes_key[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
uint8_t aes_buffer[128];
uint8_t nonce_key[16];

static eeprom_t eeprom_root;

static uint16_t data_iterator = 0;
static struct timer udup_v5_timeout_timer;
static struct ctimer wait_response;

static bool uart = 0;
static bool wait_response_slave = 0;

static volatile union 
{ 
	uint16_t u16; 
	uint8_t u8[2]; 
} packet_counter_root; 					/*Счетчик покетов*/

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF FUNCTIONS*/
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);
static void uart_to_air();
static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length);
static void wait_response_reset(void *ptr);

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF PROCESS*/

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
			else if(header_pack->data_type == UART_FROM_RX_TO_AIR)
			{
				/*Передаём данные полученные от счетчика на УСПД*/
				uart_from_air(&node_addr, data, datalen);
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
	join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&data[PAYLOAD_OFFSET];
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
	add_route ( join_stage_1_pack->serial.u32, 	/*Serial*/ 
				&addr,							/*Address*/ 
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
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&data[PAYLOAD_OFFSET];
	join_stage_4_t *join_stage_4_pack = (join_stage_4_t*)&aes_buffer[0];
	
	/*Получаем nonce*/
	u8_u16_t nonce;
	nonce.u16 = get_nonce(join_stage_3_pack->serial.u32);	
	
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
		unlock_addr(join_stage_3_pack->serial.u32);		/*Разрешаем обрабатывать пакеты принятые с авторизированного устройства*/ 		
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
/*Передает данные полученные от УСПД счетчику по радио*/
static void uart_to_air() 
{
	uip_ipaddr_t addr;		/*Выделяем память для адреса на который отправится пакет*/
	uint16_t nonce;			/*Выделяем память для nonce*/
	
	/*Парсим серийник из данных принятых из UART от УСПД и ищем в таблице IPv6 адрес*/
	/*Если команда меньше 7 байт, то она для однобайтно адресуемых счетчиков*/
	if(data_iterator < 7) 
	{
		addr = find_addr((uint32_t)(uart_rx_buffer[0]));
		
		if((((&addr)->u16[0])  == 0x00) && 
		(((&addr)->u16[1])  == 0x00) && 
		(((&addr)->u16[2])  == 0x00) && 
		(((&addr)->u16[3])  == 0x00) &&
		(((&addr)->u16[4])  == 0x00) &&
		(((&addr)->u16[5])  == 0x00) && 
		(((&addr)->u16[6])  == 0x00) && 
		(((&addr)->u16[7])  == 0x00))
		{
			return;
		}
		
		nonce = get_nonce((uint32_t)(uart_rx_buffer[0]));
	}
	
	/*В первую очередь проверяем по четырехбайтному адресу, а потом по однобайтному*/
	else 
	{
		addr = find_addr((uint32_t)((uart_rx_buffer[0] << 24) |
												 (uart_rx_buffer[1] << 16) |
												 (uart_rx_buffer[2] << 8)  |
												  uart_rx_buffer[3]));
		if((((&addr)->u16[0])  == 0x00) && 
		(((&addr)->u16[1])  == 0x00) && 
		(((&addr)->u16[2])  == 0x00) && 
		(((&addr)->u16[3])  == 0x00) &&
		(((&addr)->u16[4])  == 0x00) &&
		(((&addr)->u16[5])  == 0x00) && 
		(((&addr)->u16[6])  == 0x00) && 
		(((&addr)->u16[7])  == 0x00))	
		{	
					addr = find_addr((uint32_t)(uart_rx_buffer[0]));
		
					if((((&addr)->u16[0])  == 0x00) && 
					(((&addr)->u16[1])  == 0x00) && 
					(((&addr)->u16[2])  == 0x00) && 
					(((&addr)->u16[3])  == 0x00) &&
					(((&addr)->u16[4])  == 0x00) &&
					(((&addr)->u16[5])  == 0x00) && 
					(((&addr)->u16[6])  == 0x00) && 
					(((&addr)->u16[7])  == 0x00))
					{
						return;
					}
					
					nonce = get_nonce((uint32_t)(uart_rx_buffer[0]));
		}
		else
		{
			nonce = get_nonce((uint32_t)((uart_rx_buffer[0] << 24) |
										 (uart_rx_buffer[1] << 16) |
										 (uart_rx_buffer[2] << 8)  |
										 (uart_rx_buffer[3] )));
		}
	}
	
	nonce_key[0] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[1] = (uint8_t)(nonce & 0xFF);
	nonce_key[2] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[3] = (uint8_t)(nonce & 0xFF);
	nonce_key[4] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[5] = (uint8_t)(nonce & 0xFF);
	nonce_key[6] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[7] = (uint8_t)(nonce & 0xFF);
	nonce_key[8] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[9] = (uint8_t)(nonce & 0xFF);
	nonce_key[10] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[11] = (uint8_t)(nonce & 0xFF);
	nonce_key[12] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[13] = (uint8_t)(nonce & 0xFF);
	nonce_key[14] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[15] = (uint8_t)(nonce & 0xFF);

	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	/*Нижняя часть header'а будет шифроваться. Поэтому для рассчета payload'а нужно учитывать её*/
	uint8_t crypto_length = iterator_to_byte(data_iterator + HEADER_DOWN_LENGTH);
	uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	/*Отражаем структуры на массивы*/ 
	header_up_t *header_up_pack = (header_up_t*)&udp_buffer[HEADER_OFFSET];
	header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];	
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_up_pack->protocol_version = UDBP_PROTOCOL_VERSION; 	/*Текущая версия протокола*/ 
	header_up_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_up_pack->data_type = UART_FROM_AIR_TO_TX;			/*Тип пакета*/  
	header_up_pack->rssi = get_parent_rssi();					/*RSSI*/ 
	header_up_pack->temperature = get_temperature();			/*Температура*/ 
	header_up_pack->voltage = get_voltage();					/*Напряжение*/ 
	
	/*Шифрованая часть header'а*/ 
	header_down_pack->counter.u16 = packet_counter_root.u16;	/*Счетчик пакетов*/ 
	header_down_pack->length = data_iterator;					/*Размер пакета*/
	
	/*Заполняем блок для шифрования*/ 
	for(uint8_t i = HEADER_DOWN_LENGTH; i < crypto_length; i++)
	{
		if(i < (data_iterator + HEADER_DOWN_LENGTH))
			aes_buffer[i] = uart_rx_buffer[i-3];		/*Заполняем блок для шифрования данными*/ 
		else
			aes_buffer[i] = 0x00;						/*Дозаполняем блок для шифрования нулями*/ 
	}
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), &addr);
	packet_counter_root.u16++;		/*Инкрементируем счетчик пакетов*/
}

/*---------------------------------------------------------------------------*/
/*Передаём данные полученные от счетчика на УСПД*/
static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length)
{
	/*Если мы не ожидаем приёма от утройства, то пакет отбрасываем*/
	if(wait_response_slave == 0)
		return;
	
	/*Отражаем структуры на массивы*/ 
	header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)(aes_buffer), (length - HEADER_UP_LENGTH));
	
	uint32_t serial = ( (aes_buffer[3] << 24) |
						(aes_buffer[4] << 16) |
						(aes_buffer[5] << 8)  |
						(aes_buffer[6] ));					
	
	if(valid_counter(serial, header_down_pack->counter.u16) || valid_counter(aes_buffer[3], header_down_pack->counter.u16))
	{
		for(uint16_t i = 0; i < aes_buffer[2]; i++)
			cc26xx_uart_write_byte(aes_buffer[i + 3]);
	}
	// else
	// {
		// cc26xx_uart_write_byte(0xFF);
		// printf("%i %lu", header_down_pack->counter.u16, serial);
	// }
}

/*---------------------------------------------------------------------------*/
/*Иннициализация RPL*/
void rpl_initialize()
{
	/* Set MESH-mode for dc-power rpl-root(not leaf-mode) */
	rpl_set_mode(RPL_MODE_MESH);

	static uip_ipaddr_t ipaddr;

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
	/* register udp-connection, set incoming upd-data handler(udp_data_receiver) */
	simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_data_receiver);

	/* set incoming uart-data handler */
	cc26xx_uart_set_input(&uart_data_receiver);

	/* blink-blink LED */
	led_blink(LED_A);
	led_blink(LED_A);

	/* start main root process */
	process_start(&main_root_process, NULL);
}

/*---------------------------------------------------------------------------*/
/*Обработчик прерывания UART*/
int uart_data_receiver(unsigned char uart_char)
{
	led_blink(LED_A);
	if(data_iterator < UDUP_V5_RC_MAX_LENGTH)
	{
		uart_rx_buffer[data_iterator] = uart_char;
		data_iterator++;
		timer_restart(&udup_v5_timeout_timer);
	}
	return 1;
}

/*---------------------------------------------------------------------------*/
/**/
void set_uart_r(void)
{
	uart = 1;
}

/*---------------------------------------------------------------------------*/
/**/
void unset_uart_r(void)
{
	uart = 0;
}

/*---------------------------------------------------------------------------*/
/**/
uint8_t uart_status_r(void)
{
	if(uart == 1)
		return 1;
	else
		return 0;
}

/*---------------------------------------------------------------------------*/
/**/
static void wait_response_reset(void *ptr)
{
	wait_response_slave = 0;
}

/*---------------------------------------------------------------------------*/
/*Процесс инициализации настроек из EEPROM*/
PROCESS_THREAD(settings_root_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	read_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
	
	if(eeprom_root.aes_key_configured == true) //При первом включении забивает нормальные настройки сети
	{
		if((eeprom_root.channel != 26) && (eeprom_root.panid != 0xAABB))
		{
			eeprom_root.channel = 26;
			eeprom_root.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
		}
	}
	
	if(!eeprom_root.aes_key_configured) 
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_root.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");
		;
	}
	else
	{
		printf("AES-128 key not declared\n******************************\n******PLEASE SET AES KEY******\n******************************\n");
		// led_mode_set(LED_FAST_BLINK);
		while(eeprom_root.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	radio_value_t channel = 0;
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	if(channel != eeprom_root.channel)
	{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_root.channel);
		
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_root.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_root.channel, freq_mhz);
		}

		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_root.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_root.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		if(panid != eeprom_root.panid)
		{
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_root.panid);
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_root.panid);
		}
	}
	
	process_post(&rpl_root_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*Процесс управления ROOT'ом*/
PROCESS_THREAD(main_root_process, ev, data)
{
	PROCESS_BEGIN();
	timer_set(&udup_v5_timeout_timer, 10);

	static struct etimer main_root_process_timer;
	packet_counter_root.u16 = 0;
	PROCESS_PAUSE();

	uint16_t crc_uart;

	while (1)
	{
		etimer_set(&main_root_process_timer, 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&main_root_process_timer));

		if (timer_expired(&udup_v5_timeout_timer) && data_iterator > 0)
		{
			if(uart == 1)
			{
				if(data_iterator > 3)
				{
					crc_uart = crc16_modbus(uart_rx_buffer, data_iterator-2);
					if(crc_uart == (uint16_t) ((uart_rx_buffer[data_iterator-1] << 8) | 
												uart_rx_buffer[data_iterator-2]))
					{
						wait_response_slave = 1;
						ctimer_set(&wait_response, WAIT_RESPONSE * CLOCK_SECOND, wait_response_reset, NULL);
						uart_to_air();
					}
				}
				
				data_iterator = 0;
			}
			else
			{
				// disable_interrupts();
				data_iterator = 0;
				// enable_interrupts();
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
