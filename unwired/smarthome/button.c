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


#include "button.h"

#include "ti-lib.h"
#include "../ud_binary_protocol.h"
#include "../dag_node.h"

#include "net/rpl/rpl-private.h"

/*---------------------------------------------------------------------------*/
/* Register button sensors */
SENSORS(&button_a_sensor_click, &button_a_sensor_long_click,
        &button_b_sensor_click, &button_b_sensor_long_click,
        &button_c_sensor_click, &button_c_sensor_long_click,
        &button_d_sensor_click, &button_d_sensor_long_click,
        &button_e_sensor_click, &button_e_sensor_long_click);


/* register main button process */
PROCESS(main_process, "UD Buttons control process");

/* set autostart processes */
AUTOSTART_PROCESSES(&dag_node_process, &main_process);

/*---------------------------------------------------------------------------*/

void udbp_v5_button_status_sender(uint8_t button_number,
                                  uint8_t click_type)
{
   if (node_mode == 2) //MODE_NOTROOT_SLEEP
   {
      watchdog_reboot();
   }

   if (node_mode == MODE_NORMAL)
   {
      uip_ipaddr_t addr;
      uip_ip6addr_copy(&addr, &root_addr);

      uint8_t short_press_mask = 0b10000000;
      uint8_t long_press_mask = 0b11000000;

      uint8_t dio_and_status = button_number;

      if (click_type == DEVICE_ABILITY_BUTTON_EVENT_CLICK)
         dio_and_status |= short_press_mask;

      else if (click_type == DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK)
         dio_and_status |= long_press_mask;

      printf("DAG Node: Send button packet to DAG-root node\n");

      uint8_t payload_length = 16;
      uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
      udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
      udp_buffer[1] = packet_counter_node.u8[0];
      udp_buffer[2] = packet_counter_node.u8[1];
      udp_buffer[3] = get_parent_rssi();
      udp_buffer[4] = get_temperature();
      udp_buffer[5] = get_voltage();

      udp_buffer[6] = UNWDS_4BTN_MODULE_ID;
      udp_buffer[7] = dio_and_status;
      udp_buffer[8] = DATA_RESERVED;
      udp_buffer[9] = DATA_RESERVED;
      udp_buffer[10] = DATA_RESERVED;
      udp_buffer[11] = DATA_RESERVED;
      udp_buffer[12] = DATA_RESERVED;
      udp_buffer[13] = DATA_RESERVED;
      udp_buffer[14] = DATA_RESERVED;
      udp_buffer[15] = DATA_RESERVED;
      udp_buffer[16] = DATA_RESERVED;
      udp_buffer[17] = DATA_RESERVED;
      udp_buffer[18] = DATA_RESERVED;
      udp_buffer[19] = DATA_RESERVED;
      udp_buffer[20] = DATA_RESERVED;
      udp_buffer[21] = DATA_RESERVED;

      net_on(RADIO_ON_TIMER_OFF);
      simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
      packet_counter_node.u16++;
      led_mode_set(LED_FLASH);
   }
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{
   PROCESS_BEGIN();
   printf("Unwired buttons device. HELL-IN-CODE free. I hope.\n");

   PROCESS_PAUSE();


   while (1)
   {
      PROCESS_YIELD();
      if (ev == sensors_event)
      {
         if (data == &button_a_sensor_click)
         {
            printf("BCP: Button A click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_A, DEVICE_ABILITY_BUTTON_EVENT_CLICK);
         }
         if (data == &button_a_sensor_long_click)
         {
            printf("BCP: Button A long click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_A, DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK);
         }
         if (data == &button_b_sensor_click)
         {
            printf("BCP: Button B click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_B, DEVICE_ABILITY_BUTTON_EVENT_CLICK);
         }
         if (data == &button_b_sensor_long_click)
         {
            printf("BCP: Button B long click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_B, DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK);
         }
         if (data == &button_c_sensor_click)
         {
            printf("BCP: Button C click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_C, DEVICE_ABILITY_BUTTON_EVENT_CLICK);
         }
         if (data == &button_c_sensor_long_click)
         {
            printf("BCP: Button C long click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_C, DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK);
         }
         if (data == &button_d_sensor_click)
         {
            printf("BCP: Button D click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_D, DEVICE_ABILITY_BUTTON_EVENT_CLICK);
         }
         if (data == &button_d_sensor_long_click)
         {
            printf("BCP: Button D long click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_D, DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK);
         }
         if (data == &button_e_sensor_click)
         {
            printf("BCP: Button e click\n");
            udbp_v5_button_status_sender(BOARD_IOID_KEY_E, DEVICE_ABILITY_BUTTON_EVENT_CLICK);
         }

      }
   }

   PROCESS_END();
}
