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
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"

#include <stdio.h>
#include <string.h>

#define INTERFACE_RS485 			0 
#define INTERFACE_CAN 				1 

/*---------------------------------------------------------------------------*/
/* main UPD connection */
struct simple_udp_connection udp_connection;

/*---------------------------------------------------------------------------*/
/*ПРОТОТИПЫ ФУНКЦИЙ*/

/*Обработчик принятых пакетов*/
void udp_data_receiver(struct simple_udp_connection *connection,
                       const uip_ipaddr_t *sender_addr,
                       uint16_t sender_port,
                       const uip_ipaddr_t *receiver_addr,
                       uint16_t receiver_port,
                       const uint8_t *data,
                       uint16_t datalen);

/*Отправка настроек канала ШИМ'а*/
void pwm_settings_sender(const uip_ip6addr_t *dest_addr, 
						uint8_t channel, 
						uint32_t frequency, 
						uint8_t duty);

/*Отправка команды включения/выключения канала ШИМ'а*/
void pwm_power_channel_sender ( const uip_ip6addr_t *dest_addr, 
								uint8_t channel, 
								uint8_t pwm_power_channel);
						
/*Иннициализация RPL*/
void rpl_initialize();

/*Иннициализация ноды*/
void root_node_initialize();

/*Обработчик прерывания UART*/
int uart_data_receiver(unsigned char uart_char);

/*Устанавливает режим работы UART. Работает в режиме UART*/
void set_uart_r(void);

/*Устанавливает режим работы UART. Работает в режиме консоли*/
void unset_uart_r(void);

/*Возвращает режим работы UART*/
uint8_t uart_status_r(void);

/*---------------------------------------------------------------------------*/
/*ИМЕНА ПРОЦЕССОВ*/

/*Процесс инициализации настроек из EEPROM*/
PROCESS_NAME(settings_root_init);

/*Процесс управления ROOT'ом*/
PROCESS_NAME(main_root_process);

/*---------------------------------------------------------------------------*/
