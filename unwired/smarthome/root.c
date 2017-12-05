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
*         RPL-root service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
* \author
*         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
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

#include "../ud_binary_protocol.h"
#include "xxf_types_helper.h"
#include "dev/watchdog.h"
#include "root.h"
#include "../root-node.h"


#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
   UART_INT_RX | UART_INT_CTS)


#define UART_ROOT_RX IOID_26
#define UART_ROOT_TX IOID_25

#define  UART_NORMAL_TX BOARD_IOID_UART_TX
#define  UART_NORMAL_RX BOARD_IOID_UART_RX
#define  UART_ALT_TX UART_ROOT_TX
#define  UART_ALT_RX UART_ROOT_RX

/*---------------------------------------------------------------------------*/

/* Buttons on DIO 1 */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

PROCESS(rpl_root_process,"Unwired RPL root and udp data receiver");

AUTOSTART_PROCESSES(&rpl_root_process);

/*---------------------------------------------------------------------------*/


void off_uart(uint32_t rx_dio, uint32_t tx_dio)
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

void on_uart(uint32_t rx_dio, uint32_t tx_dio, uint32_t baud_rate)
{
   ti_lib_ioc_pin_type_gpio_output(UART_NORMAL_TX);
   ti_lib_gpio_set_dio(UART_NORMAL_TX);
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


PROCESS_THREAD(rpl_root_process, ev, data)
{

   PROCESS_BEGIN();

   printf("Unwired RLP root. HELL-IN-CODE free. I hope.\n");

   /* if you do not execute "cleanall" target, rpl-root can build in "leaf" configuration. Diagnostic message */
   if (RPL_CONF_LEAF_ONLY == 1)
      printf("\nWARNING: leaf mode on rpl-root!\n");

   rpl_initialize();

   root_node_initialize();

   off_uart(UART_NORMAL_RX, UART_NORMAL_TX);
   on_uart(UART_ALT_RX, UART_ALT_TX, 115200);
   printf("Alt uart active\n");

   while (1)
   {
      PROCESS_WAIT_EVENT();
      if (ev == sensors_event && data == &button_e_sensor_long_click)
      {
         led_on(LED_A);
         printf("SYSTEM: Button E long click, reboot\n");
         watchdog_reboot();
      }
   }

   PROCESS_END();
}
