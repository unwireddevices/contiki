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
#include "ud_binary_protocol.h"
#include "protocol.h"

#include "dag_node.h"
#include "uart/uart.h"

#define MAINTENANCE_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define SHORT_STATUS_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define LONG_STATUS_INTERVAL			(20 * 60 * CLOCK_SECOND)
#define ROOT_FIND_INTERVAL				(2 * CLOCK_SECOND)
#define ROOT_FIND_LIMIT_TIME			(2 * 60 * CLOCK_SECOND)
#define FW_DELAY						(2 * CLOCK_SECOND)
#define FW_MAX_ERROR_COUNTER			5

#define FALSE							0x00
#define TRUE							0x01

#define MAX_NON_ANSWERED_PINGS			3

#define WAIT_RESPONSE					0.150 	//Максимальное время ожидания ответа от счетчика в секундах

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/

/* struct for simple_udp_send */
simple_udp_connection_t udp_connection;


volatile uint8_t led_mode;

volatile uint8_t non_answered_packet = 0;
volatile uip_ipaddr_t root_addr;		/*Адресс root'а*/

static struct etimer maintenance_timer;

static struct ctimer wait_response; 	/*Таймер который запускатся после выдачи сообщения счетчику, если счетчик не ответил за это время, то сообщение принятые данные не передаются root'у*/
static bool wait_response_slave = 0; 	/*Переменная которая отражает состояние таймера*/

/*Счетчик покетов*/
static volatile union 
{ 
	uint16_t u16; 
	uint8_t u8[2]; 
} packet_counter_root;					

static uint8_t aes_buffer[128];			/*Буффер для шифрования*/
static uint8_t aes_key[16];				/*Ключ шифрования для ECB*/
static uint8_t nonce_key[16];			/*Сессионный ключ*/

static eeprom_t eeprom_dag;				/*Структура с значениями из EEPROM*/
static uint8_t interface; 				/*Интерфейс общения с счетчиком*/
extern uint32_t serial;					/*Серийный номер*/

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF FUNCTIONS*/

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

/*Передает данные полученные из радио от ROOT'а на счетчик через UART*/
static void uart_from_air ( const uip_ipaddr_t *sender_addr,
							const uint8_t *data,
							uint16_t datalen);

/**/
static void wait_response_reset(void *ptr);

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF PROCESS*/

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
	header_t *header_pack = (header_t*)&data[HEADER_OFFSET];

	/*Вывод информационного сообщения в консоль*/
	if(uart_status() == 0)
	{
		printf("DAG Node: UDP packet received(%"PRIu8"): ", datalen);
		for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
			printf("%"PRIXX8, data[i]);
		printf("\n");
	}
	
	/*Проверяем версию протокола*/ 
	if(header_pack->protocol_version == UDBP_PROTOCOL_VERSION)
	{
		/*Проверяем ID модуля*/ 
		if(header_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
		{
			/*Проверяем тип пакета*/ 
			if (header_pack->data_type == DATA_TYPE_JOIN_STAGE_2)
			{
				/*Третья стадия авторизации*/
				join_stage_3_sender(sender_addr, data, datalen);
			}
			
			else if (header_pack->data_type == DATA_TYPE_JOIN_STAGE_4)
			{
				/*Обработчик четвертой стадии авторизации*/
				join_stage_4_handler(sender_addr, data, datalen);
			}
			
			else if (header_pack->data_type == UART_FROM_AIR_TO_TX)
			{
				/*Передает данные полученные из радио от root'а на счетчик через UART*/
				uart_from_air(sender_addr, data, datalen);
			}
			
			else
			{
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("DAG Node: Incompatible packet type(endpoint UNWDS_6LOWPAN_SYSTEM_MODULE_ID): %"PRIXX8"\n", header_pack->data_type);
			}
		}
	}
	
	else
	{
		/*Вывод информационного сообщения в консоль*/
		if(uart_status() == 0)
			printf("DAG Node: Incompatible protocol version: %"PRIXX8"\n", header_pack->protocol_version);
	}

	/*Мигаем светодиодом*/
	led_mode_set(LED_FLASH);
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
		printf("DAG Node: Send join packet to DAG-root node: ");
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
	join_stage_1_pack->serial.u32 = serial;						/*Серийный номер*/ 

	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Join_stage_1_sender serial: %i\n", join_stage_1_pack->serial.u32);
	// printf("Join_stage_1_sender plaintext:\n");
	// hexraw_print((HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH), udp_buffer);
	// printf("\n");

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
		printf("DAG Node: Send join packet stage 3 to DAG-root node:");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH];	
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&aes_buffer[0];
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
	join_stage_3_pack->serial.u32 = serial;						/*Serial*/ 
	
	/*Расшифровываем блок*/ 
	aes_ecb_decrypt((uint32_t*)aes_key, (uint32_t*)&data[PAYLOAD_OFFSET], (uint32_t*)aes_buffer);
	
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	for(int i = 0; i < 16; i += 2)
	{
		nonce_key[i] = join_stage_2_pack->nonce.u8[1];	
		nonce_key[i+1] = join_stage_2_pack->nonce.u8[0];	
	}
	
	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Join_stage_3_sender nonce: %i\n", join_stage_2_pack->nonce.u16);
	// hexraw_print(16, nonce_key);
	// printf("\n");
	
	/*Зашифровываем данные*/
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&join_stage_3_pack->crypto_1_block), CRYPTO_1_BLOCK_LENGTH);		

	/*Для отладки. Выводит содержимое пакета*/ 
	// printf("Join_stage_3_sender pack:\n");
	// hexraw_print((HEADER_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH), udp_buffer);
	// printf("\n");
	
	/*Отправляем пакет*/ 
	simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH), &addr);
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
	// join_stage_4_t *join_stage_4_pack = (join_stage_4_t*)&aes_buffer[0];
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[PAYLOAD_OFFSET], (uint32_t*)(aes_buffer), CRYPTO_1_BLOCK_LENGTH);
	
	/*Проверяем массив. Если все нули, то авториция прошла успешно*/ 
	if( (aes_buffer[0] == 0x00)  &&
		(aes_buffer[1] == 0x00)  &&
		(aes_buffer[2] == 0x00)  &&
		(aes_buffer[3] == 0x00)  &&
		(aes_buffer[4] == 0x00)  &&
		(aes_buffer[5] == 0x00)  &&
		(aes_buffer[6] == 0x00)  &&
		(aes_buffer[7] == 0x00)  &&
		(aes_buffer[8] == 0x00)  &&
		(aes_buffer[9] == 0x00)  &&
		(aes_buffer[10] == 0x00) &&
		(aes_buffer[11] == 0x00) &&
		(aes_buffer[12] == 0x00) &&
		(aes_buffer[13] == 0x00) &&
		(aes_buffer[14] == 0x00) &&
		(aes_buffer[15] == 0x00))
	{
		packet_counter_root.u16 = header_pack->counter.u16;				/*Сохраняем счетчик пакетов ROOT'а*/ 
		uip_ipaddr_copy(&root_addr, sender_addr); 						/*Копируем адрес ROOT'а с которым авторизировались*/ 
		packet_counter_node.u16 = 1;									/*Инициализируем счетчик пакетов*/
		etimer_set(&maintenance_timer, 0);								/*Устанавливаем таймер*/ 
		process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);	/*Передаем управление dag_node_process*/ 
		return;
	}
	
	/*Выводим: Ошибка авторизации*/
	printf("Authorisation Error\n"); 
}

/*---------------------------------------------------------------------------*/
/*Передает данные полученные из радио от ROOT'а на счетчик через UART*/
static void uart_from_air ( const uip_ipaddr_t *sender_addr,
							const uint8_t *data,
							uint16_t datalen)
{
	/*Отражаем структуры на массивы*/ 
	header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];	
	
	/*Расшифровываем данные*/
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[HEADER_DOWN_OFFSET], (uint32_t*)(aes_buffer), (datalen - HEADER_UP_LENGTH));
	
	/*Проверяем счетчик пакетов на валидность данного пакета*/
	if(packet_counter_root.u16 < header_down_pack->counter.u16)
	{	
		/*Обновляем значение счетчика ROOT'а*/
		packet_counter_root.u16 = header_down_pack->counter.u16;		
		
		/*Если интерфейс RS485, то устанавливаем DE и RE в высокий уровень*/
		if(get_interface() == INTERFACE_RS485)
		{
			ti_lib_gpio_set_dio(RS485_DE);		/*Устанавливаем DE в высокий уровень*/	
			ti_lib_gpio_set_dio(RS485_RE);		/*Устанавливаем RE в высокий уровень*/
		}
		
		/*Запрещаем прерывания*/
		ti_lib_uart_int_disable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		
		/*Отправляем данные на счетчик через UART*/
		for(uint16_t i = 0; i < aes_buffer[HEADER_DOWN_LENGTH_OFFSET]; i++)
			cc26xx_uart_write_byte(aes_buffer[i + 3]);
		
		/*Ожидаем окончание передачи*/
		while(ti_lib_uart_busy(UART0_BASE));
		
		/*Если интерфейс RS485, то устанавливаем DE и RE в низкий уровень*/
		if(get_interface() == INTERFACE_RS485)
		{
			ti_lib_gpio_clear_dio(RS485_DE);	/*Устанавливаем DE в низкий уровень*/	
			ti_lib_gpio_clear_dio(RS485_RE);	/*Устанавливаем RE в низкий уровень*/
		}
		
		/*Очищаем FIFO буферы*/
		while(ti_lib_uart_chars_avail(UART0_BASE))
		{
			UARTCharGetNonBlocking(UART0_BASE);
		}
		
		/*Разрешаем прерывания*/
		ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		
		reset_uart();
		wait_response_slave = 1;
		ctimer_set(&wait_response, (WAIT_RESPONSE * CLOCK_SECOND), wait_response_reset, NULL);
	}
}

/*---------------------------------------------------------------------------*/
/*Передает данные полученные от счетчика ROOT'у по радио*/
void uart_to_air(char* data)
{
	/*Если не нормальный режим работы, то перезагружаемся*/
	if (node_mode == 2) 
	{
		watchdog_reboot();
	}

	/*Если нормальный режим работы, то отправляем данные УСПД*/
	if (node_mode == MODE_NORMAL)
	{
		uint8_t *data_iterator;					/*Выделяем память под указатель на data_iterator*/
		data_iterator = (uint8_t*)&data[0];		/*В data[0] хранится размер принятых данных из UART*/
		
		uint16_t crc_uart;													/*Выделяем память под CRC16-MODBUS*/
		crc_uart = crc16_modbus((uint8_t*)&data[1], (*data_iterator - 2));	/*Рассчитываем CRC16-MODBUS*/
		
		/*Проверяем по размеру на минимально возможный*/ 
		if(*data_iterator > 3)
		{
			/*Если CRC16 не совпадает, то дальше пакет не обрабатываем*/
			if(crc_uart != (uint16_t)((data[*data_iterator] << 8) | data[*data_iterator - 1]))
			{
				return; /*CRC16 не совпала*/
			}
		}
		else
		{
			return; 	/*Слишком маленькая длина фрейма*/
		}
		
		uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
		uip_ip6addr_copy(&addr, &root_addr);	/*Копируем адрес ROOT'а*/
		
		/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
		/*Нижняя часть header'а будет шифроваться. Поэтому для рассчета payload'а нужно учитывать её*/
		uint8_t crypto_length = iterator_to_byte(*data_iterator + HEADER_DOWN_LENGTH); 
		uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
		
		/*Отражаем структуры на массивы*/ 
		header_up_t *header_up_pack = (header_up_t*)&udp_buffer[HEADER_OFFSET];
		header_down_t *header_down_pack = (header_down_t*)&aes_buffer[0];	
		
		/*Заполняем пакет*/ 
		/*Header*/ 
		header_up_pack->protocol_version = UDBP_PROTOCOL_VERSION; 	/*Текущая версия протокола*/ 
		header_up_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
		header_up_pack->data_type = UART_FROM_RX_TO_AIR;			/*Тип пакета*/  
		header_up_pack->rssi = get_parent_rssi();					/*RSSI*/ 
		header_up_pack->temperature = get_temperature();			/*Температура*/ 
		header_up_pack->voltage = get_voltage();					/*Напряжение*/ 

		/*Шифрованая часть header'а*/ 
		header_down_pack->counter.u16 = packet_counter_node.u16;	/*Счетчик пакетов*/ 
		header_down_pack->length = *data_iterator;					/*Размер пакета*/
		
		/*Заполняем блок для шифрования*/ 
		for(uint8_t i = HEADER_DOWN_LENGTH; i < crypto_length; i++)
		{
			if(i < (*data_iterator + HEADER_DOWN_LENGTH))
				aes_buffer[i] = data[i-2];		/*Заполняем блок для шифрования данными*/ 
			else
				aes_buffer[i] = 0x00;			/*Дозаполняем блок для шифрования нулями*/ 
		}
	
		/*Зашифровываем данные*/
		aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[HEADER_DOWN_OFFSET]), crypto_length);

		net_on(RADIO_ON_TIMER_OFF);
		
		/*Отправляем пакет*/ 
		simple_udp_sendto(&udp_connection, udp_buffer, (HEADER_UP_LENGTH + crypto_length), &addr);
		packet_counter_node.u16++;		/*Инкрементируем счетчик пакетов*/
		led_mode_set(LED_FLASH);		/*Мигаем светодиодом*/
	}
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
/*Возвращает текущий интерфейс общения с счетчиком*/
uint8_t get_interface(void)
{
	return interface;
}

/*---------------------------------------------------------------------------*/
/*Обновляет интерфейс общения с счетчиком в EEPROM и перезагружает*/
void interface_update(uint8_t interface_new)
{
	eeprom_dag.interface_configured = false;
	eeprom_dag.interface = interface_new;
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}

/*---------------------------------------------------------------------------*/
/*Обновляет ключ шифрования и перезагружает*/
void aes128_key_update(const uint8_t *aes_key_new)
{	
	eeprom_dag.aes_key_configured = false;
	
	for(uint8_t i = 0; i < 16; i++)
		eeprom_dag.aes_key[i] = aes_key_new[i];
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}

/*---------------------------------------------------------------------------*/
/*Возвращает указатель на массив в котором хранится ключ шифрования*/
uint8_t *get_aes128_key(void)
{
	return aes_key;
}

/*---------------------------------------------------------------------------*/
/*Обновляет серийный номер в EEPROM и перезагружает*/
void serial_update(uint32_t serial_new)
{
	eeprom_dag.serial_configured = false;
	eeprom_dag.serial = serial_new;
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}

/*---------------------------------------------------------------------------*/
/*Возвращает серийный номер*/
uint32_t get_serial(void)
{
	return serial;
}

/*---------------------------------------------------------------------------*/
/*Обновляет channel в EEPROM*/
void channel_update(uint8_t channel_new)
{
	eeprom_dag.channel = channel_new;
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
}

/*---------------------------------------------------------------------------*/
/*Обновляет PANID в EEPROM*/
void panid_update(uint16_t panid_new)
{
	eeprom_dag.panid = panid_new;
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
}

/*---------------------------------------------------------------------------*/
/**/
static void wait_response_reset(void *ptr)
{
	wait_response_slave = 0;
}

/*---------------------------------------------------------------------------*/
/**/
bool wait_response_status(void)
{
	return wait_response_slave;
}

/*---------------------------------------------------------------------------*/
/*Процесс инициализации настроек из EEPROM*/
PROCESS_THREAD(settings_dag_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	read_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
	
	if(eeprom_dag.serial_configured == true) //При первом включении забивает нормальные настройки сети
	{
		if(eeprom_dag.aes_key_configured == true)
		{
			if((eeprom_dag.channel != 26) && (eeprom_dag.panid != 0xAABB))
			{
				eeprom_dag.channel = 26;
				eeprom_dag.panid = 0xAABB;
				write_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
			}
		}
	}

	if(!eeprom_dag.serial_configured) 
	{
		serial = eeprom_dag.serial;
		printf("Serial: %lu\n", serial);
	}
	else
	{
		printf("Serial number not declared\n******************************\n***PLEASE SET SERIAL NUMBER***\n******************************\n");
		led_mode_set(LED_FAST_BLINK);	/*Мигаем светодиодом*/
		
		while(eeprom_dag.serial_configured)
		{
			PROCESS_YIELD();
		}	
	}
	
	if(!eeprom_dag.aes_key_configured) 
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_dag.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");
		;
	}
	else
	{
		printf("AES-128 key not declared\n******************************\n******PLEASE SET AES KEY******\n******************************\n");
		led_mode_set(LED_FAST_BLINK);	/*Мигаем светодиодом*/
		while(eeprom_dag.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	if(!eeprom_dag.interface_configured) 
	{
		interface = eeprom_dag.interface;
	
		if(interface == INTERFACE_RS485)
			printf("Installed interface RS485\n");
		else if(interface == INTERFACE_CAN)
			printf("Installed interface CAN\n");
		else
			printf("Unknown interface\n");
	}
	else
	{
		printf("Interface not declared\n******************************\n*****PLEASE SET INTERFACE*****\n******************************\n");
		led_mode_set(LED_FAST_BLINK);	/*Мигаем светодиодом*/
		
		while(eeprom_dag.interface_configured)
		{
			PROCESS_YIELD();
		}	
	}
	
	radio_value_t channel = 0;
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	if(channel != eeprom_dag.channel)
	{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_dag.channel);
		
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_dag.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_dag.channel, freq_mhz);
		}

		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_dag.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_dag.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		if(panid != eeprom_dag.panid)
		{
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_dag.panid);
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_dag.panid);
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
					printf("SYSTEM: Button E long click, reboot\n");
				
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
			etimer_set(&maintenance_reboot_timer, (5*CLOCK_SECOND));
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&maintenance_reboot_timer) );
			watchdog_reboot();
		}

		if(node_mode == MODE_NORMAL)
		{
			led_mode_set(LED_OFF);	/*Выключаем светодиод*/

			if(process_is_running(&root_find_process) == 1)
				process_exit(&root_find_process);

			if(non_answered_packet > MAX_NON_ANSWERED_PINGS)
			{
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("DAG Node: Root not available, reboot\n");
				watchdog_reboot();
			}
		}

		if(node_mode == MODE_NOTROOT)
		{
			if(CLASS == CLASS_B)
			{
				led_mode_set(LED_OFF);
				
				/*Вывод информационного сообщения в консоль*/
				if(uart_status() == 0)
					printf("DAG Node: Root not found, sleep\n");
				
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
					printf("DAG Node: Root not found, reboot\n"); //почему-то не перезагружается!
				
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

	etimer_set( &find_root_limit_timer, ROOT_FIND_LIMIT_TIME);

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
					printf("DAG Node: mode set to MODE_NOTROOT\n");
				
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
		printf("DAG Node: DAG active, join stage 4 packet received, mode set to MODE_NORMAL\n");
	
	led_mode_set(LED_SLOW_BLINK);	/*Включаем медленное мигание светодиодами*/
	node_mode = MODE_NORMAL;		/*Изменение режима работы ноды. Нода работает в нормальном режиме*/
	net_mode(RADIO_FREEDOM);		/**/
	net_off(RADIO_OFF_NOW);			/**/

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/