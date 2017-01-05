/*
 * Copyright (c) 2016, Unwired Devices LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */


 /*---------------------------------------------------------------------------*/
 /*
 * \file
 *         UDP reporting button service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *         Mikhail Churikov mc@unwds.com
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
#include "board-peripherals.h"
#include "dev/watchdog.h"
#include "simple-udp.h"

#include "ud-button.h"
#include "ud-main.h"

SENSORS(&button_a_sensor, &button_b_sensor, &button_c_sensor, &button_d_sensor, &button_e_sensor);


/*---------------------------------------------------------------------------*/
#define DEBUG 1
#include "net/ip/uip-debug_UD.h"
/*---------------------------------------------------------------------------*/

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 	0
#define UDP_SERVER_PORT 	4003

uip_ip6addr_t dest_ip_addr;
uint8_t connected_flag = 0;

char udp_message_buf[20]; //buffer for simple_udp_send
static struct simple_udp_connection unicast_connection; //struct for simple_udp_send


PROCESS(udp_button_process, "UDP Buttons control process");
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {  }
  return;
}
/*---------------------------------------------------------------------------*/
void
ipv6_addr_copy(uip_ip6addr_t *dest, uip_ip6addr_t *source)
{
  memcpy(dest, source, sizeof(uip_ip6addr_t));
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(udp_button_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Buttons control process: started\n");

  //aaaa::212:4b00:79e:b282, aaaa::212:4b00:6e2:728c
  dest_ip_addr.u16[0] = UIP_HTONS(0xAAAA); //зачем заполнять адрес, который потом заменится, осмысленной информацией?
  dest_ip_addr.u16[1] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[2] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[3] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[4] = UIP_HTONS(0x0212);
  dest_ip_addr.u16[5] = UIP_HTONS(0x4B00);
  dest_ip_addr.u16[6] = UIP_HTONS(0x079E);
  dest_ip_addr.u16[7] = UIP_HTONS(0xB804);

  simple_udp_register(&unicast_connection, 4003, NULL, 4003, NULL); //register simple_udp_connection

  PRINTF("Buttons control process: paused\n");
  PROCESS_PAUSE();
  PRINTF("Buttons control process: resumed\n");
  

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    else 
    if(ev == PROCESS_EVENT_CONTINUE) {
      if(data != NULL) {
        connected_flag = ((connect_info_t *)data)->connected;
        if(((connect_info_t *)data)->root_addr != NULL) {
          ipv6_addr_copy(&dest_ip_addr, ((connect_info_t *)data)->root_addr); //replace dest_ip_addr<-rlp_root(see cetic-6lbr-client)
          PRINTF("Found RPL root. Addr: ");
          PRINT6ADDR(&dest_ip_addr);
          PRINTF("\n");
        }
      }
    }
    else
    if(ev == sensors_event) {
      if(data == &button_a_sensor) {
        PRINTF("Buttons control process: Button A\n");
        if(connected_flag == 1) {
          PRINTF("Buttons control process: send message to RPL root node on ");
          PRINT6ADDR(&dest_ip_addr);
          PRINTF("\n");
          sprintf(udp_message_buf, "Button A", 1);
          simple_udp_sendto(&unicast_connection, udp_message_buf, strlen(udp_message_buf) + 1, &dest_ip_addr);
        }
        led_blink(LED_B);
      }
      if(data == &button_b_sensor) {
        PRINTF("Buttons control process: Button B\n");
        if(connected_flag == 1) {

        }
        led_blink(LED_B);
      }
      if(data == &button_c_sensor) {
        PRINTF("Buttons control process: Button C\n");
        if(connected_flag == 1) {

        }
        led_blink(LED_B);
      }
      if(data == &button_d_sensor) {
        PRINTF("Buttons control process: Button D\n");
        if(connected_flag == 1) {

        }
        led_blink(LED_B);
      }
      if(data == &button_e_sensor) {
        PRINTF("Buttons control process: Button E\n");
        //lpm_shutdown(BOARD_IOID_KEY_RIGHT, IOC_IOPULL_UP, IOC_WAKE_ON_LOW);
        if(connected_flag == 1) {

        }
        led_blink(LED_B);
      }
    }
  }
  printf("Buttons control process: end\r\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/