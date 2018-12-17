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

#include "pwm_fet.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"
#include "../dag_node.h"

#include "../../cpu/cc26xx-cc13xx/dev/pwm.h" /*PWM*/
#include "../../platform/unwired/udboards/opt3001.h" /*PWM*/

#include "net/rpl/rpl-private.h"

/*---------------------------------------------------------------------------*/
/* Register button sensors */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

/* register main button process */
PROCESS(main_process, "PWM process");

/* set autostart processes */
AUTOSTART_PROCESSES(&main_process);

/*---------------------------------------------------------------------------*/
/*Стартовый процесс (точка входа)*/
PROCESS_THREAD(main_process, ev, data)
{
	PROCESS_BEGIN();
	printf("Start Unwired PWM device.\n");
	PROCESS_PAUSE();
	
	if (BOARD_IOID_UART_RX == IOID_UNUSED)
	{
		printf("[DAG Node] Shell not active, uart RX set to IOID_UNUSED\n");
		cc26xx_uart_set_input(NULL);
	}
	else
	{
		serial_shell_init();
		shell_reboot_init();
		shell_time_init();
		unwired_shell_init();
		printf("[DAG Node] Shell activated, type \"help\" for command list\n");
	}
	
	process_start(&settings_dag_init, NULL); 				/*Запуск процесса инициализации настроек из EEPROM*/
	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE); 		/*Ожидание окончание настройки*/
	process_exit(&settings_dag_init); 						/*Завершение процесса*/
	
	process_start(&dag_node_process, NULL);					/*Запуск ноды*/
	
	// opt3001_init();
	// opt3001_measure();
	
	// pwm_config(0, 100, 20, IOID_5); //channel, frequency, duty, pin
	// pwm_config(1, 100, 30, IOID_6); //channel, frequency, duty, pin
	// pwm_config(2, 1000, 40, IOID_7); //channel, frequency, duty, pin
	// pwm_config(3, 1000, 50, IOID_24); //channel, frequency, duty, pin
	// pwm_config(4, 10000, 60, IOID_25); //channel, frequency, duty, pin
	// pwm_config(5, 100000, 70, IOID_26); //channel, frequency, duty, pin
	
	// pwm_start(0);
	// pwm_start(1);
	// pwm_start(2);
	// pwm_start(3);
	// pwm_start(4);
	// pwm_start(5);
	
	// while(1)
	// {
		// PROCESS_YIELD();
		
		// uint8_t duty = 100 - (opt3001_measure() / 10);
		// pwm_config(0, 100, duty, IOID_5); //channel, frequency, duty, pin
		// pwm_start(0);
		// printf("[UMDK-LIT] Luminocity: %lu lux\n", opt3001_measure());
	// }
	
	while(1)
	{
		PROCESS_YIELD();
		
		if (ev == sensors_event)
		{
			if (data == &button_e_sensor_long_click)
			{
				printf("[BCP] Button e long click\nReboot...");
				watchdog_reboot();
			}
			
			if (data == &button_e_sensor_click)
			{
				pwm_config(0, 100, 20, IOID_5); //channel, frequency, duty, pin
				pwm_config(1, 100, 30, IOID_6); //channel, frequency, duty, pin
				pwm_config(2, 100, 40, IOID_7); //channel, frequency, duty, pin
				pwm_config(3, 100, 50, IOID_24); //channel, frequency, duty, pin
				pwm_config(4, 100, 60, IOID_25); //channel, frequency, duty, pin
				pwm_config(5, 100, 70, IOID_26); //channel, frequency, duty, pin

				pwm_start(0);
				pwm_start(1);
				pwm_start(2);
				pwm_start(3);
				pwm_start(4);
				pwm_start(5);

				// printf("[UMDK-LIT] Luminocity: %lu lux\n", opt3001_measure());
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/