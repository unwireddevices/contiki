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

#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ip/simple-udp.h"

/*---------------------------------------------------------------------------*/

#define MODE_NORMAL				0x01
#define MODE_NOTROOT			0x02
#define MODE_JOIN_PROGRESS		0x03
#define MODE_NEED_REBOOT		0x04

#define LED_OFF					0x00
#define LED_ON					0x01
#define LED_FLASH				0x02
#define LED_SLOW_BLINK			0x03
#define LED_FAST_BLINK			0x04

#define INTERFACE_RS485			0x00 
#define INTERFACE_CAN			0x01 

#define CLICK					0x80
#define LONG_CLICK				0xC0

/*---------------------------------------------------------------------------*/

simple_udp_connection_t udp_connection;		/*Структура UDP подключения*/
volatile uip_ipaddr_t root_addr;			/*Адресс root'а*/
volatile uint8_t node_mode;					/*Режим работы ноды*/

/*Счетчик пакетов*/
volatile union 
{ 
	uint16_t u16;
	uint8_t u8[2];
} packet_counter_node;						

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ФУНКЦИЙ*/

/*Функция отправки состояния кнопок*/
void button_status_sender ( uint8_t button_number,
							uint8_t click_type);

/*Функция управления светодиодами*/
void led_mode_set(uint8_t mode);

/**/
void pack_sender(uint8_t device_id, 
				uint8_t data_type, 
				uint8_t *payload, 
				uint8_t payload_len);


/*---------------------------------------------------------------------------*/
/*ИМЕНА ПРОЦЕССОВ*/

/*Процесс опроса ROOT'а на достижимость*/
PROCESS_NAME(ping_process);

/*Процесс инициализации настроек из EEPROM*/
PROCESS_NAME(settings_dag_init);

/*Запуск инициализации ноды (точка входа)*/
PROCESS_NAME(dag_node_process);

/*Процесс отслеживает нажатие кнопки. При нажатии происходит перезагрузка*/
PROCESS_NAME(dag_node_button_process);

/*Процесс поиска ROOT'а*/
PROCESS_NAME(root_find_process);

/*Процесс управления нодой*/
PROCESS_NAME(maintenance_process);

/*Процесс упарвления светодиодами*/
PROCESS_NAME(led_process);

/*---------------------------------------------------------------------------*/
