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
*         Relay service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
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
#include "simple-udp.h"

#include "relay.h"

#include "xxf_types_helper.h"

#include "ti-lib.h"
#include "../ud_binary_protocol.h"
#include "../int-flash-common.h"
#include "../dag_node.h"
#include "system-common.h"

/*---------------------------------------------------------------------------*/

#define BOARD_RELAY IOID_16


typedef enum {
	UMDK_GPIO_REPLY_OK_0 = 0,
	UMDK_GPIO_REPLY_OK_1 = 1,
	UMDK_GPIO_REPLY_OK = 2,
	UMDK_GPIO_REPLY_ERR_PIN = 3,
	UMDK_GPIO_REPLY_ERR_FORMAT = 4,
   UMDK_GPIO_REPLY_OK_AINAF = 5,
   UMDK_GPIO_REPLY_OK_ALL = 6
} umdk_gpio_reply_t;

typedef enum {
	UMDK_GPIO_GET = 0,
	UMDK_GPIO_SET_0 = 1,
	UMDK_GPIO_SET_1 = 2,
	UMDK_GPIO_TOGGLE = 3,
   UMDK_GPIO_GET_ALL = 4,
} umdk_gpio_action_t;

/*---------------------------------------------------------------------------*/

/* Register buttons sensors */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

/* register relay process */
PROCESS(main_process, "Relay control process");

/* set autostart processes */
AUTOSTART_PROCESSES(&dag_node_process, &main_process);

/*---------------------------------------------------------------------------*/

void udbp_v5_gpio_reply_sender(uint8_t reply)
{

   if (node_mode == MODE_NORMAL)
   {
      uip_ipaddr_t addr;
      uip_ip6addr_copy(&addr, &root_addr);

      printf("DAG Node: Send gpio reply packet to DAG-root node\n");

      uint8_t payload_length = 2;
      uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
      udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
      udp_buffer[1] = packet_counter_node.u8[0];
      udp_buffer[2] = packet_counter_node.u8[1];
      udp_buffer[3] = get_parent_rssi();
      udp_buffer[4] = get_temperature();
      udp_buffer[5] = get_voltage();

      udp_buffer[6] = UNWDS_GPIO_MODULE_ID;
      udp_buffer[7] = reply;

      simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
      packet_counter_node.u16++;
      led_mode_set(LED_FLASH);
   }
}

PROCESS_THREAD(main_process, ev, data)
{
   PROCESS_BEGIN();

   static struct interpocess_message *message_data = NULL;

   PROCESS_PAUSE();

   printf("Unwired relay device. HELL-IN-CODE free. I hope.\n");
   ti_lib_ioc_pin_type_gpio_output(BOARD_RELAY);
/*
   uint8_t command = 1;
   uint8_t gpio = 15;

   uint8_t gpio_cmd = command << 6;
   gpio_cmd |= gpio & 0b00011111;

   uint8_t command_2 = (gpio_cmd & 0b11100000) >> 6;
   uint8_t gpio_2 = gpio_cmd & 0b00011111;

   printf("TEST: GPIO %"PRIu8" and CMD %"PRIu8" => 0x%"PRIXX8" => GPIO %"PRIu8" and CMD %"PRIu8"\n", gpio, command, gpio_cmd, gpio_2, command_2);
*/
   while (1)
   {
      PROCESS_YIELD();
      if (ev == PROCESS_EVENT_CONTINUE)
      {
         message_data = data;
         if (message_data->payload[0] == UNWDS_GPIO_MODULE_ID)
         {
            uint8_t gpio_cmd = message_data->payload[1];
            uint8_t command = (gpio_cmd & 0b11100000) >> 5;
            uint8_t gpio = gpio_cmd & 0b00011111;
            printf("RELAY: GPIO: %"PRIu8", command: %"PRIu8"\n", gpio, command);

            if (gpio != BOARD_RELAY)
            {
               printf("Relay: gpio error: %"PRIu8"\n", gpio);
               udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_ERR_PIN);
            }
            else
            {
               if (command == UMDK_GPIO_SET_0)
               {
                  ti_lib_gpio_clear_dio(BOARD_RELAY);
                  udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_OK);
               }
               else if (command == UMDK_GPIO_SET_1)
               {
                  ti_lib_gpio_set_dio(BOARD_RELAY);
                  udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_OK);
               }
               else if (command == UMDK_GPIO_GET)
               {
                  udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_ERR_FORMAT);
               }
               else if (command == UMDK_GPIO_TOGGLE)
               {
                  ti_lib_gpio_toggle_dio(BOARD_RELAY);
                  udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_OK);
               }
               else
               {
                  udbp_v5_gpio_reply_sender(UMDK_GPIO_REPLY_ERR_FORMAT);
               }
            }
         }
      }
   }

   PROCESS_END();
}
