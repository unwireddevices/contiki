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
*         Button service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
* \author
*         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
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
//#include "sys/etimer.h"

//#include "../../../"

#include "uart.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"
#include "../ud_binary_protocol.h"
#include "../dag_node.h"
#include "../int-flash-common.h"

#include "net/rpl/rpl-private.h"

#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
   UART_INT_RX | UART_INT_CTS)
   
#define RS485_DE IOID_29
#define RS485_RE IOID_30

/*---------------------------------------------------------------------------*/
/* Register button sensors */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);


/* register main button process */
PROCESS(main_process, "UD Buttons control process");
//PROCESS(uart_process, "TEST Process");

/* set autostart processes */
AUTOSTART_PROCESSES(&main_process);

/*---------------------------------------------------------------------------*/

static void off_uart(uint32_t rx_dio, uint32_t tx_dio)
{
   ti_lib_ioc_port_configure_set(tx_dio, IOC_PORT_GPIO, IOC_STD_OUTPUT);
   ti_lib_ioc_port_configure_set(rx_dio, IOC_PORT_GPIO, IOC_STD_INPUT);
   ti_lib_gpio_set_output_enable_dio(tx_dio, GPIO_OUTPUT_ENABLE);
   ti_lib_gpio_set_dio(tx_dio);

   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_disable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_fifo_disable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_disable(UART0_BASE);
}
/*---------------------------------------------------------------------------*/
static void on_uart(uint32_t rx_dio, uint32_t tx_dio, uint32_t baud_rate)
{
   if(baud_rate == 9600) //Не обновляется скорость UART
   {
	  (*(unsigned long*)(0x40001024)) = 312;
	  (*(unsigned long*)(0x40001028)) = 32;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   else 
   {
	  (*(unsigned long*)(0x40001024)) = 26;
	  (*(unsigned long*)(0x40001028)) = 3;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   
   ti_lib_ioc_pin_type_gpio_output(tx_dio);
   ti_lib_gpio_set_dio(tx_dio);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_ioc_pin_type_uart(UART0_BASE, rx_dio, tx_dio, IOID_UNUSED, IOID_UNUSED);
   ti_lib_uart_config_set_exp_clk(UART0_BASE, ti_lib_sys_ctrl_clock_get(), baud_rate, 
                  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   ti_lib_uart_fifo_level_set(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
   ti_lib_uart_fifo_enable(UART0_BASE);
   ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_enable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{
	PROCESS_BEGIN();
	//printf("Unwired buttons device. HELL-IN-CODE free. I hope.\n");
	printf("Start Unwired UART device.\n");
	PROCESS_PAUSE();
	
	//ti_lib_gpio_clear_dio(RS485_DE);
	//ti_lib_gpio_clear_dio(RS485_RE);
	ti_lib_ioc_pin_type_gpio_output(RS485_DE);
	ti_lib_ioc_pin_type_gpio_output(RS485_RE);
	
	//PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
	
	//settings_init();
	
	
	
	//serial = ((user_flash_read_byte(0) << 24) |
	//		 (user_flash_read_byte(1) << 16)  |
	//		 (user_flash_read_byte(2) << 8)   |
	//		 user_flash_read_byte(3));
	
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
	
	//settings_init();
	process_start(&settings_init, NULL);
	
	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	
	process_exit(&settings_init);
	
	process_start(&dag_node_process, NULL);
	
	static struct etimer shell_off;
	etimer_set(&shell_off, CLOCK_SECOND * 15);
	
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
	
	/*
	if(serial != 0xFFFFFFFF)
	{
		printf("Serial: %lu\n", serial);
		process_start(&dag_node_process, NULL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
	}
	else
	{
		printf("Serial number not declared\n******************************\n***PLEASE SET SERIAL NUMBER***\n******************************\n");
		led_mode_set(LED_FAST_BLINK);
		while(serial == 0xFFFFFFFF)
		{
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
			if(serial == 0xFFFFFFFF)
			{
				etimer_reset(&shell_off);
				//printf("Reset etimer\n");
			}
		}	
	}
	*/
	
	//if (BOARD_IOID_UART_TX != BOARD_IOID_ALT_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
		
	if(get_interface() == INTERFACE_RS485)
	{
		if (BOARD_IOID_UART_TX != BOARD_IOID_ALT_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
		{
			printf("UDM: UART change to alt(RX: 26, TX: 25)\n");
			off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
			on_uart(BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX, 9600);
			ti_lib_gpio_clear_dio(RS485_DE);
			ti_lib_gpio_clear_dio(RS485_RE);
		}
	}
	else if(get_interface() == INTERFACE_CAN)
	{
		if (BOARD_IOID_UART_TX != BOARD_IOID_CAN_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
		{
			printf("UDM: UART change to alt(RX: 30, TX: 29)\n");
			off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
			on_uart(BOARD_IOID_CAN_UART_RX, BOARD_IOID_CAN_UART_TX, 9600);
		}
	}
	//else
		//printf("Unknown interface\n");
	
	// if (BOARD_IOID_UART_TX != BOARD_IOID_CAN_UART_TX || BOARD_IOID_CAN_UART_RX != BOARD_IOID_ALT_UART_RX)
	// {
		// if(uart_status() == 0)
		// {
			// printf("UDM: UART change to alt(RX: 26, TX: 25)\n");
			// printf("UDM: UART change to alt(RX: 30, TX: 29)\n");
			// printf("\nUART_IBRD: %lu \nUART_FBRD: %lu \nLHCR: %lu \n ", (*(unsigned long*)(0x40001024)), (*(unsigned long*)(0x40001028)), (*(unsigned long*)(0x4000102C)) );
			// printf("UDM: UART change to alt(RX: %"PRIu16", TX: %"PRIu16")\n", BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX);
		// }
		// off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
		// on_uart(BOARD_IOID_CAN_UART_RX, BOARD_IOID_CAN_UART_TX, 9600);
		// set_uart();
		// on_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX, 9600);
		// clock_delay((CLOCK_SECOND / 9600) * 1);
		// printf("UDM: Alt UART active\n");
		// printf("\nUART_IBRD: %lu \nUART_FBRD: %lu \nLHCR: %lu \n ", (*(unsigned long*)(0x40001024)), (*(unsigned long*)(0x40001028)), (*(unsigned long*)(0x4000102C)) );
	// }
	
	set_uart();
	while (1)
	{
		PROCESS_YIELD();
		if(ev == uart_event_message) 
		{
			if(wait_response_status() == 1) 
			{
				udbp_v5_uart_to_root_sender(data);
			}
			//printf("len: %i\n", ((char *)data)[0]);
			//printf("received line: %s\n", (char *)data);
			//udbp_v5_uart_to_root_sender(data);
		} 
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*PROCESS_THREAD(test_process, ev, data)
{
	static struct etimer et;
	PROCESS_BEGIN();
	etimer_set(&et, CLOCK_SECOND);
	printf("Start TEST\n");
	
	while(1) 
	{
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		
		printf("TEST: sec\n");
		etimer_reset(&et);
	}
	PROCESS_END();
 }*/