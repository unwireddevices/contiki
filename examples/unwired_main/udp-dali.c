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
/**
 * \file
 *         UDP to DALI protocol translating service.  
 * \author
 *         Mikhail Churikov mc@unwds.com
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/dali-manchester.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ud-world.h"
#include "udp-common.h"

#define DEBUG 1
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	0
#define UDP_SERVER_PORT1	4007

status_reply_data_t dali_reply_data;
static struct uip_udp_conn *server_conn0;
static struct uip_udp_conn *server_conn1;
uip_ip6addr_t dest_ip_addr;

PROCESS(udp_dali_process, "UDP DALI interface process");
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint8_t num = 0;
  if(uip_newdata()) {
    switch ((incoming_command_t)((uint8_t *)uip_appdata)[0]) {
      case GET:
        PRINTF("Got ADC measurement request. \n");
        memcpy(&dali_reply_data.ip_addr, &UIP_IP_BUF->srcipaddr, sizeof(uip_ip6addr_t));
        process_post(&udp_dali_process, PROCESS_EVENT_CONTINUE, &dali_reply_data);
        break;

      case SET:
        num = atoi(uip_appdata + 2);
        PRINTF("Got ADC_SET command. Desired Pin: %d\n", num);
        break;
      default:
        PRINTF("Wrong command byte in the incoming packet.\n");
    }
  }

  return;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_dali_process, ev, data)
{
  PROCESS_BEGIN();
  char buffer[32];

  dali_init();
  dali_send_forward_frame(0xFF, 0xFF);

  //aaaa::212:4b00:8fb:7d28
  dest_ip_addr.u16[0] = UIP_HTONS(0xAAAA);
  dest_ip_addr.u16[1] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[2] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[3] = UIP_HTONS(0x0000);
  dest_ip_addr.u16[4] = UIP_HTONS(0x0212);
  dest_ip_addr.u16[5] = UIP_HTONS(0x4B00);
  dest_ip_addr.u16[6] = UIP_HTONS(0x08FB);
  dest_ip_addr.u16[7] = UIP_HTONS(0x7D28);

  PRINTF("MY UDP DALI server started\n");
  PROCESS_PAUSE();

  if (open_udp_connection(server_conn0, UDP_SERVER_PORT1) < 0)
      PROCESS_EXIT();
  server_conn1 = udp_new(&dest_ip_addr, UIP_HTONS(UDP_SERVER_PORT1), NULL);

  memset(buffer, '\0', 32);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    if(ev == PROCESS_EVENT_CONTINUE) {
      if(data != NULL) {
        PRINTF("Woke up and try to send reply.\n");
        PRINTF("Destination address: ");
        PRINT6ADDR(&((status_reply_data_t *)data)->ip_addr);
        PRINTF("\n");

        send_udp_to_mote(server_conn1, &((status_reply_data_t *)data)->ip_addr, 
                          UDP_SERVER_PORT1, buffer);
        PRINTF("Sended.\n");
      }
    }
  }
  printf("disable --- UDP 4006\r\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
