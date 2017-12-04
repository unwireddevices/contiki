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
*         fire button service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
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
#include "dev/contiki-watchdog.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "button-sensor.h"
#include "board.h"
#include "board-peripherals.h"
#include "simple-udp.h"

#include "firebutton.h"
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

#define OUT_LED_DIO IOID_24

/*---------------------------------------------------------------------------*/

/* Register button sensors */
SENSORS(&button_a_sensor_click);

/* register main button process */
PROCESS(main_process, "fire button control process");

PROCESS(blink_led_firebutton_process, "fire button led control process");

/* set autostart processes */
AUTOSTART_PROCESSES(&dag_node_process, &main_process, &blink_led_firebutton_process);

static struct etimer blink_led_firebutton_process_timer;

clock_time_t led_blink_off = 15*CLOCK_SECOND;
clock_time_t led_blink_on = CLOCK_SECOND/10;

/*---------------------------------------------------------------------------*/

void send_button_status_packet(uint8_t button_number,
                               uint8_t click_type)
{
   struct sensor_packet button_sensor_packet;
   button_sensor_packet.protocol_version = CURRENT_PROTOCOL_VERSION;
   button_sensor_packet.device_version = CURRENT_DEVICE_VERSION;
   button_sensor_packet.data_type = DATA_TYPE_SENSOR_DATA;
   button_sensor_packet.number_ability = DEVICE_ABILITY_BUTTON;
   button_sensor_packet.sensor_number = button_number;
   button_sensor_packet.sensor_event = click_type;
   send_sensor_event(&button_sensor_packet);

   if (node_mode == 2) //MODE_NOTROOT_SLEEP
   {
      watchdog_reboot();
   }

   led_blink(LED_A);
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{
   PROCESS_BEGIN();

   printf("Unwired fire button device. HELL-IN-CODE free. I hope.\n");
   static struct etimer firebutton_led_process_timer;
   ti_lib_ioc_pin_type_gpio_output(OUT_LED_DIO);

   PROCESS_PAUSE();

   while (1)
   {
      PROCESS_YIELD();
      if (ev == sensors_event)
      {
         printf("BCP: Button click\n");
         if (data == &button_a_sensor_click && node_mode == MODE_NORMAL)
         {
            ti_lib_gpio_set_dio(OUT_LED_DIO);
            ti_lib_gpio_clear_dio(OUT_LED_DIO);

            send_button_status_packet('a', DEVICE_ABILITY_BUTTON_EVENT_CLICK);
            led_blink_off = CLOCK_SECOND/2;
            led_blink_on = CLOCK_SECOND/10;
            etimer_reset_with_new_interval(&blink_led_firebutton_process_timer, 1);

            PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_MSG && *(uint8_t*)data == PT_MESSAGE_PONG_RECIEVED);
            printf("BCP: Pong!\n");
            led_blink_off = CLOCK_SECOND/100;
            led_blink_on = CLOCK_SECOND*5;
            etimer_reset_with_new_interval(&blink_led_firebutton_process_timer, 1);

            etimer_set(&firebutton_led_process_timer, 60*CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&firebutton_led_process_timer));
            led_blink_off = 15*CLOCK_SECOND;
            led_blink_on = CLOCK_SECOND/10;
         }
      }
   }

   PROCESS_END();
}


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(blink_led_firebutton_process, ev, data)
{
   PROCESS_BEGIN();


   PROCESS_PAUSE();

   while (1)
   {
      ti_lib_gpio_set_dio(OUT_LED_DIO);
      //printf("BCP: Set!\n");
      etimer_set(&blink_led_firebutton_process_timer, led_blink_on);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&blink_led_firebutton_process_timer));
      ti_lib_gpio_clear_dio(OUT_LED_DIO);
      //printf("BCP: Clean!\n");
      etimer_set(&blink_led_firebutton_process_timer, led_blink_off);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&blink_led_firebutton_process_timer));
   }

   PROCESS_END();
}