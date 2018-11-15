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

#include "../../core/dev/serial-line.h"
#include "../../apps/serial-shell/serial-shell.h"
#include "../../apps/shell/shell.h"

#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"
#include "simple-udp.h"
#include "net/rpl/rpl.h"

#include <stdio.h>
#include <string.h>

#include "button-sensor.h"
#include "board-peripherals.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"

#include "xxf_types_helper.h"
#include "dev/watchdog.h"
#include "root.h"
#include "../root-node.h"
#include "../system-common.h"//
#include "../protocol.h"//

#include "sys/etimer.h"
								
#define IOC_INPUT_PULL_UP	(IOC_CURRENT_2MA	| IOC_STRENGTH_AUTO	| \
							IOC_IOPULL_UP		| IOC_SLEW_DISABLE	| \
							IOC_HYST_DISABLE	| IOC_NO_EDGE		| \
							IOC_INT_DISABLE		| IOC_IOMODE_NORMAL	| \
							IOC_NO_WAKE_UP		| IOC_INPUT_ENABLE	)

/*---------------------------------------------------------------------------*/

/* Buttons on DIO 1 */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

PROCESS(rpl_root_process, "Unwired RPL root and udp data receiver");

AUTOSTART_PROCESSES(&rpl_root_process);

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(rpl_root_process, ev, data)
{
	PROCESS_BEGIN();
	
	printf("Start Unwired RLP root.\n");
	
	if (BOARD_IOID_UART_RX == IOID_UNUSED)
	{
		printf("[ROOT Node] Shell not active, uart RX set to IOID_UNUSED\n");
		cc26xx_uart_set_input(NULL);
	}
	else
	{
		serial_shell_init();
		shell_reboot_init();
		shell_time_init();
		unwired_shell_init();
		printf("[ROOT Node] Shell activated, type \"help\" for command list\n");
	}
	
	process_start(&settings_root_init, NULL);
	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	process_exit(&settings_root_init);
	
	/* if you do not execute "cleanall" target, rpl-root can build in "leaf" configuration. Diagnostic message */
	if(RPL_CONF_LEAF_ONLY == 1)
		printf("\n[ROOT Node] WARNING: leaf mode on rpl-root!\n");

	rpl_initialize();
	root_node_initialize();

	static struct etimer shell_off;	
	printf("[ROOT Node] Command line is lock. For unlock, press the button E\n");
	ti_lib_ioc_port_configure_set(BOARD_IOID_UART_RX, IOC_PORT_GPIO, IOC_INPUT_PULL_UP);
	ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_RX, IOC_PORT_MCU_UART0_RX, IOC_INPUT_PULL_UP);
	set_uart();	

	while (1)
	{
		PROCESS_WAIT_EVENT();
		
		/*Перезагрузка*/
		if(ev == sensors_event && data == &button_e_sensor_long_click)
		{
			led_on(LED_A);
			printf("[ROOT Node] Button E long click, reboot\n");
			watchdog_reboot();
		}
		
		/*Разблокировка командной строки на 15 секунд.*/
		if(ev == sensors_event && data == &button_e_sensor_click)
		{	
			printf("[ROOT Node] Command line is unlocked for 15 seconds\n");
			ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_RX, IOC_PORT_GPIO, IOC_INPUT_PULL_UP);
			ti_lib_ioc_port_configure_set(BOARD_IOID_UART_RX , IOC_PORT_MCU_UART0_RX, IOC_INPUT_PULL_UP);
			unset_uart();		
	
			etimer_set(&shell_off, CLOCK_SECOND * 15);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
			
			printf("[ROOT Node] Command line is lock\n");
			ti_lib_ioc_port_configure_set(BOARD_IOID_UART_RX, IOC_PORT_GPIO, IOC_INPUT_PULL_UP);
			ti_lib_ioc_port_configure_set(BOARD_IOID_ALT_UART_RX, IOC_PORT_MCU_UART0_RX, IOC_INPUT_PULL_UP);
			set_uart();	
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/