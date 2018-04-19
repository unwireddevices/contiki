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
/**
 * \file
 *         Header file for DAG-node service
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ip/simple-udp.h"

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

simple_udp_connection_t udp_connection;
volatile uip_ipaddr_t root_addr;
volatile uint8_t node_mode;
volatile union { uint16_t u16; uint8_t u8[2]; } packet_counter_node;

struct interpocess_message
{
   volatile uint8_t payload[16];
};

void udbp_v5_message_sender(uint8_t message_type, uint8_t data_1, uint8_t data_2);
void udbp_v5_uart_to_root_sender(char* data);
//void uart_console(unsigned char uart_char); //ХЗ
//void send_time_sync_req_packet();
void led_mode_set(uint8_t mode);
//void udbp_v5_join_stage_3_sender(const uip_ipaddr_t *dest_addr);
bool wait_response_status(void);

PROCESS_NAME(dag_node_process);
PROCESS_NAME(dag_node_button_process);
PROCESS_NAME(root_find_process);
PROCESS_NAME(status_send_process);
PROCESS_NAME(maintenance_process);
PROCESS_NAME(led_process);
PROCESS_NAME(fw_update_process);

/*---------------------------------------------------------------------------*/
