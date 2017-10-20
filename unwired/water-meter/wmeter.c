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
*         water meter service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "button-sensor.h"
#include "board.h"
#include "board-peripherals.h"
#include "simple-udp.h"

#include "wmeter.h"
#include "radio_power.h"
#include "../dag_node.h"
#include "gpio-interrupt.h"
#include "dev/cc26xx-uart.h"

#include "xxf_types_helper.h"

#include "ti-lib.h"
#include "clock.h"
#include "../ud_binary_protocol.h"
#include "../int-flash-common.h"
#include "../rtc-common.h"
#include "../system-common.h"

#define L_OUT_DIO IOID_4
#define T_OUT_DIO IOID_11
#define T_IN_DIO IOID_5

#define SEND_TIME             (15 * CLOCK_SECOND)

/*---------------------------------------------------------------------------*/

static uint32_t water_counter = 0;

/*---------------------------------------------------------------------------*/

/* Register buttons sensors */
SENSORS(&button_e_sensor_click);

/* register dimmer process */
PROCESS(main_process, "water meter control process");
PROCESS(send_data_process, "Uart-data send process");

/* set autostart processes */
AUTOSTART_PROCESSES(&dag_node_process, &main_process, &send_data_process);

/*---------------------------------------------------------------------------*/


void send_wmeter_counter_packet(uint32_t water_counter_u32)
{
   if (node_mode != MODE_NORMAL)
      return;

   uip_ipaddr_t addr;
   uip_ip6addr_copy(&addr, &root_addr);

   u8_u32_t water_counter;

   water_counter.u32 = water_counter_u32;

   uint8_t udp_buffer[PROTOCOL_VERSION_V2_16BYTE];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_MESSAGE;
   udp_buffer[3] = DEVICE_MESSAGE_FREE_DATA;
   udp_buffer[4] = water_counter.u8[0];
   udp_buffer[5] = water_counter.u8[1];
   udp_buffer[6] = water_counter.u8[2];
   udp_buffer[7] = water_counter.u8[3];
   udp_buffer[8] = DATA_NONE;
   udp_buffer[9] = DATA_NONE;
   udp_buffer[10] = DATA_NONE;
   udp_buffer[11] = DATA_NONE;
   udp_buffer[12] = DATA_NONE;
   udp_buffer[13] = DATA_NONE;
   udp_buffer[14] = DATA_NONE;
   udp_buffer[15] = DATA_NONE; // << 16-byte packet, ready to encrypt v2 protocol

   net_on(RADIO_ON_TIMER_OFF);
   simple_udp_sendto(&udp_connection, udp_buffer, PROTOCOL_VERSION_V2_16BYTE, &addr);
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(send_data_process, ev, data)
{
   PROCESS_BEGIN();

   static struct etimer send_data_process_timer;
   PROCESS_PAUSE();

   while (1)
   {
      send_wmeter_counter_packet(water_counter);
      printf("Send counter: %"PRIu32"\n", water_counter);
      etimer_set(&send_data_process_timer, SEND_TIME);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_data_process_timer));
   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{
   PROCESS_BEGIN();

   static struct etimer water_meter_timet;
   static uint32_t current_value = 0;
   static uint32_t prev_measured = 0;
   static uint32_t prev_saved = 0;

   PROCESS_PAUSE();

   printf("Unwired water meter device. HELL-IN-CODE free. I hope.\n");
   ti_lib_ioc_pin_type_gpio_output(L_OUT_DIO);
   ti_lib_ioc_pin_type_gpio_output(T_OUT_DIO);
   ti_lib_ioc_pin_type_gpio_input(T_IN_DIO);

   while (1)
   {
      ti_lib_gpio_set_dio(L_OUT_DIO);
      ti_lib_gpio_set_dio(T_OUT_DIO);
      clock_delay_usec(25);
      current_value = ti_lib_gpio_read_dio(T_IN_DIO);
      ti_lib_gpio_clear_dio(L_OUT_DIO);
      ti_lib_gpio_clear_dio(T_OUT_DIO);

      if (current_value == prev_measured)
      {
         if ((prev_saved == 0) && (current_value == 1))
         {
            water_counter++;
         }
         prev_saved = current_value;
      }
      prev_measured = current_value;

      etimer_set(&water_meter_timet, 13); //~100ms
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&water_meter_timet));
   }

   PROCESS_END();
}
