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
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"

#include <stdio.h>
#include <string.h>

#include "../ud_binary_protocol.h"

/*---------------------------------------------------------------------------*/

#define MODE_NORMAL                             0x01
#define MODE_NOTROOT                            0x02
#define MODE_JOIN_PROGRESS                      0x03
#define MODE_NEED_REBOOT                        0x04

#define LED_OFF                                 0x00
#define LED_ON                                  0x01
#define LED_FLASH                               0x02
#define LED_SLOW_BLINK                          0x03
#define LED_FAST_BLINK                          0x04

/*---------------------------------------------------------------------------*/
 typedef enum hex_to_bin_errno_t{
   HEX2BIN_SUCCESS,
   HEX2BIN_NONPARITY,
   HEX2BIN_INCONVERTIBLE
} hex_to_bin_errno_t;

/* main UPD connection */
struct simple_udp_connection udp_connection;

PROCESS_NAME(main_root_process);
// PROCESS_NAME(settings_init);
PROCESS_NAME(led_process);

/*---------------------------------------------------------------------------*/

void rpl_initialize();

void root_node_initialize();

void led_mode_set(uint8_t mode);

void udp_data_receiver(struct simple_udp_connection *connection,
                       const uip_ipaddr_t *sender_addr,
                       uint16_t sender_port,
                       const uip_ipaddr_t *receiver_addr,
                       uint16_t receiver_port,
                       const uint8_t *data,
                       uint16_t datalen);

int uart_data_receiver_udup_v5(unsigned char uart_char);

void set_uart_r(void);
void unset_uart_r(void);
uint8_t uart_status_r(void);

uint32_t* get_aes128_key(void);
void aes128_key_update(const uint8_t *aes_key_new);
void channel_update(uint8_t channel_new);
void panid_update(uint16_t panid_new);

/*---------------------------------------------------------------------------*/
