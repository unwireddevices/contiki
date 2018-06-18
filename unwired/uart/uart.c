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
*         Header file for UART service
* \author
*         Manchenko Oleg man4enkoos@gmail.com
*/
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "uip-ds6-route.h"
#include "net/ip/uip-debug.h"
#include "dev/leds.h"
#include "../../apps/shell/shell.h"
#include "../../apps/serial-shell/serial-shell.h"
#include "dev/serial-line.h" //
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "button-sensor.h"
#include "board.h"
#include "board-peripherals.h"
#include "dev/watchdog.h"
#include "simple-udp.h"
#include "system-common.h"
#include "radio_power.h"

#include "uart.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"
#include "../dag_node.h"

#include "net/rpl/rpl-private.h"

/*---------------------------------------------------------------------------*/
/* Register button sensors */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

/* register main button process */
PROCESS(main_process, "UART process");

/* set autostart processes */
AUTOSTART_PROCESSES(&main_process);

/*---------------------------------------------------------------------------*/
/*Стартовый процесс (точка входа)*/
PROCESS_THREAD(main_process, ev, data)
{
	PROCESS_BEGIN();
	printf("Start Unwired UART device.\n");
	PROCESS_PAUSE();
	
	ti_lib_ioc_pin_type_gpio_output(RS485_DE); 				/*Настройка ноги RS485_DE на выход*/
	ti_lib_ioc_pin_type_gpio_output(RS485_RE); 				/*Настройка ноги RS485_RE на выход*/
	
	if (BOARD_IOID_UART_RX == IOID_UNUSED)
	{
		printf("DAG Node: Shell not active, uart RX set to IOID_UNUSED\n");
		cc26xx_uart_set_input(NULL);
	}
	else
	{
		serial_shell_init();
		shell_reboot_init();
		shell_time_init();
		unwired_shell_init();
		printf("DAG Node: Shell activated, type \"help\" for command list\n");
	}
	
	process_start(&settings_dag_init, NULL); 				/*Запуск процесса инициализации настроек из EEPROM*/
	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE); 		/*Ожидание окончание настройки*/
	process_exit(&settings_dag_init); 						/*Завершение процесса*/
	
	process_start(&dag_node_process, NULL);					/*Запуск ноды*/
	
	static struct etimer shell_off;							/*Создание таймера по истечению которого выключается шелл*/
	etimer_set(&shell_off, CLOCK_SECOND * 5);				/*Заводится таймер на 5 секунд*/
	
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));	/*Ожидание пока сработает таймер*/
		
	/*Инициализируем UART в нужном режиме*/
	if(get_interface() == INTERFACE_RS485) 					/*Инициализация в режиме интерфейса: RS485*/
	{
		if (BOARD_IOID_UART_TX != BOARD_IOID_ALT_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
		{
			printf("UDM: UART change to alt(RX: 26, TX: 25)\n");
			off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
			on_uart(BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX, 9600);
			ti_lib_gpio_clear_dio(RS485_DE);				/*Устанавливаем RS485_DE в низкий уровень. Интерфейс работает на приём*/
			ti_lib_gpio_clear_dio(RS485_RE);				/*Устанавливаем RS485_RE в низкий уровень. Интерфейс работает на приём*/
		}
	}
	else if(get_interface() == INTERFACE_CAN)				/*Инициализация в режиме интерфейса: CAN*/
	{
		if (BOARD_IOID_UART_TX != BOARD_IOID_CAN_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
		{
			printf("UDM: UART change to alt(RX: 30, TX: 29)\n");
			off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
			on_uart(BOARD_IOID_CAN_UART_RX, BOARD_IOID_CAN_UART_TX, 9600);
		}
	}
	
	set_uart();							/*Запрещает выводить данные консоли в UART*/
	
	while(1) 							/*Цикл который ожидает события uart_event_message*/
	{
		PROCESS_YIELD(); 
		if(ev == uart_event_message) 
		{
			if(wait_response_status()) 
			{
				uart_to_air(data); 		/*Отправляет данные счетчику по радио*/
			}
		} 
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/