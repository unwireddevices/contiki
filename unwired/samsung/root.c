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

#include "sys/etimer.h"

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

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
		
	process_start(&settings_root_init, NULL);
	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	process_exit(&settings_root_init);
	
	/* if you do not execute "cleanall" target, rpl-root can build in "leaf" configuration. Diagnostic message */
	if (RPL_CONF_LEAF_ONLY == 1)
		printf("\nWARNING: leaf mode on rpl-root!\n");

	rpl_initialize();
	root_node_initialize();

	// static struct etimer shell_off;
	// etimer_set(&shell_off, CLOCK_SECOND * 5);
	// PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
   
	// if (BOARD_IOID_UART_TX != BOARD_IOID_ALT_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
	// {
		// if(uart_status_r() == 0)
		// {
			// printf("UDM: UART change to alt(RX: %"PRIu16", TX: %"PRIu16")\n", BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX);
		// }
		// off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
		// on_uart(BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX, 9600);
		// set_uart_r();
	// }

	while (1)
	{
		PROCESS_WAIT_EVENT();
		if (ev == sensors_event && data == &button_e_sensor_long_click)
		{
			led_on(LED_A);
			if(uart_status_r() == 0)
				printf("UDM: Button E long click, reboot\n");
			watchdog_reboot();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/