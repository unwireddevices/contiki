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
#include "system-common.h"

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"

#include "../ud_binary_protocol.h"
#include "xxf_types_helper.h"
#include "dev/watchdog.h"
#include "root-node.h"
#include "lpm.h"
#include "crypto-common.h"
#include "rtc-common.h"


#define UART_DATA_POLL_INTERVAL 5 //in main timer ticks, one tick ~8ms

static uint8_t lpm_mode_return(void);
void send_time_sync_resp_packet(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

LPM_MODULE(root_lpm_module, lpm_mode_return, NULL, NULL, LPM_DOMAIN_NONE);

/*---------------------------------------------------------------------------*/

/* The sequence of start and end command */
uint8_t uart_magic_sequence[MAGIC_SEQUENCE_LENGTH] = {MAGIC_SEQUENCE}; //deprecated
uint8_t uart_data[MAX_UART_DATA_LENGTH]; //deprecated

uint8_t uart_data_udup_v4[MAX_UART_PACKET_LENGTH_UDUP_V4];
uint8_t udp_data_udup_v4[256];  //сделать дефайном!

static uint16_t udup_v4_data_iterator = 0;
static struct timer udup_v4_timeout_timer;

PROCESS(main_root_process, "main root process");

/*---------------------------------------------------------------------------*/

static uint8_t lpm_mode_return(void)
{
   return LPM_MODE_AWAKE;
}

/*---------------------------------------------------------------------------*/

void send_confirmation_packet(const uip_ip6addr_t *dest_addr)
{
   if (dest_addr == NULL)
   {
      printf("UDM: dest_addr in send_confirmation_packet null\n");
      return;
   }

   uint8_t length = 10;
   uint8_t udp_buffer[length];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_JOIN_CONFIRM;
   udp_buffer[3] = DATA_RESERVED;
   udp_buffer[4] = DATA_RESERVED;
   udp_buffer[5] = DATA_RESERVED;
   udp_buffer[6] = DATA_RESERVED;
   udp_buffer[7] = DATA_RESERVED;
   udp_buffer[8] = DATA_RESERVED;
   udp_buffer[9] = DATA_RESERVED;
   simple_udp_sendto(&udp_connection, udp_buffer, length, dest_addr);
}

/*---------------------------------------------------------------------------*/

void send_pong_packet(const uip_ip6addr_t *dest_addr)
{
   if (dest_addr == NULL)
   {
      printf("UDM: dest_addr in send_pong_packet null\n");
      return;
   }

   uint8_t length = 10;
   uint8_t udp_buffer[length];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_PONG;
   udp_buffer[3] = DATA_RESERVED;
   udp_buffer[4] = DATA_RESERVED;
   udp_buffer[5] = DATA_RESERVED;
   udp_buffer[6] = DATA_RESERVED;
   udp_buffer[7] = DATA_RESERVED;
   udp_buffer[8] = DATA_RESERVED;
   udp_buffer[9] = DATA_RESERVED;
   simple_udp_sendto(&udp_connection, udp_buffer, length, dest_addr);
}

/*---------------------------------------------------------------------------*/

void dag_root_hex_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length)
{
   if (addr == NULL || data == NULL)
      return;

   u8_u16_t payload_length;
   payload_length.u16 = length;

   udp_data_udup_v4[0] = MAGIC_SEQUENCE_UDUP_V4;
   udp_data_udup_v4[1] = UART_PROTOCOL_VERSION_V4;
   udp_data_udup_v4[2] = UDUP_V4_COMMAND_TYPE_PACKET_SEND;

   udp_data_udup_v4[3] = ((uint8_t *)addr)[8];
   udp_data_udup_v4[4] = ((uint8_t *)addr)[9];
   udp_data_udup_v4[5] = ((uint8_t *)addr)[10];
   udp_data_udup_v4[6] = ((uint8_t *)addr)[11];

   udp_data_udup_v4[7] = ((uint8_t *)addr)[12];
   udp_data_udup_v4[8] = ((uint8_t *)addr)[13];
   udp_data_udup_v4[9] = ((uint8_t *)addr)[14];
   udp_data_udup_v4[10] = ((uint8_t *)addr)[15];

   udp_data_udup_v4[11] = 0x00;
   udp_data_udup_v4[12] = 0x00;

   if (payload_length.u16 > 256-15-2-1)
      payload_length.u16 = 256-15-2-1;
   else
      payload_length.u16 = length;

   udp_data_udup_v4[13] = payload_length.u8[0];
   udp_data_udup_v4[14] = payload_length.u8[1];

   for (uint16_t i = 0; i < 15; i++)
   {
      printf("%"PRIXX8, udp_data_udup_v4[i]);
   }

   for (uint16_t i = 0; i < length; i++)
   {
      printf("%"PRIXX8, data[i]);
   }

   printf("%"PRIXX8, 0x00);
   printf("%"PRIXX8, 0x00);


   printf("\n");
}

/*---------------------------------------------------------------------------*/

void dag_root_bin_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length)
{
   if (addr == NULL || data == NULL)
      return;

   u8_u16_t payload_length;
   payload_length.u16 = length;

   udp_data_udup_v4[0] = MAGIC_SEQUENCE_UDUP_V4;
   udp_data_udup_v4[1] = UART_PROTOCOL_VERSION_V4;
   udp_data_udup_v4[2] = UDUP_V4_COMMAND_TYPE_PACKET_SEND;

   udp_data_udup_v4[3] = ((uint8_t *)addr)[8];
   udp_data_udup_v4[4] = ((uint8_t *)addr)[9];
   udp_data_udup_v4[5] = ((uint8_t *)addr)[10];
   udp_data_udup_v4[6] = ((uint8_t *)addr)[11];

   udp_data_udup_v4[7] = ((uint8_t *)addr)[12];
   udp_data_udup_v4[8] = ((uint8_t *)addr)[13];
   udp_data_udup_v4[9] = ((uint8_t *)addr)[14];
   udp_data_udup_v4[10] = ((uint8_t *)addr)[15];

   udp_data_udup_v4[11] = 0x00;
   udp_data_udup_v4[12] = 0x00;

   if (payload_length.u16 > 256-15-2-1)
      payload_length.u16 = 256-15-2-1;
   else
      payload_length.u16 = length;

   udp_data_udup_v4[13] = payload_length.u8[0];
   udp_data_udup_v4[14] = payload_length.u8[1];

   for (uint16_t i = 0; i < 15; i++)
   {
      cc26xx_uart_write_byte(udp_data_udup_v4[i]);
   }

   for (uint16_t i = 0; i < length; i++)
   {
      cc26xx_uart_write_byte(data[i]);
   }

   cc26xx_uart_write_byte(0x00);
   cc26xx_uart_write_byte(0x00);

}

/*---------------------------------------------------------------------------*/

void free_data_raw_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length)
{
   if (addr == NULL || data == NULL)
      return;

   u8_u32_t water_counter;
   water_counter.u8[0] = data[4];
   water_counter.u8[1] = data[5];
   water_counter.u8[2] = data[6];
   water_counter.u8[3] = data[7];

   printf("New message from address ");
   uip_debug_ipaddr_print(addr);
   printf(": counter %"PRIu32"\n", water_counter.u32);
}


/*---------------------------------------------------------------------------*/

void decrypted_data_processed(const uip_ip6addr_t *sender_addr, const uint8_t *data, uint16_t datalen)
{
   uint8_t packet_type = data[2];
   uint8_t packet_subtype = data[3];

   if (packet_type == DATA_TYPE_JOIN)
   {
      send_confirmation_packet(sender_addr);
   }

   else if (packet_type == DATA_TYPE_STATUS || packet_type == DATA_TYPE_SENSOR_DATA)
   {
      send_pong_packet(sender_addr);
   }

   else if (packet_type == DATA_TYPE_SET_TIME && packet_subtype == DATA_TYPE_SET_TIME_REQUEST)
   {
      send_time_sync_resp_packet(sender_addr, data, datalen);
      return;
   }

   else if (packet_type == DATA_TYPE_MESSAGE && packet_subtype == DEVICE_MESSAGE_FREE_DATA)
   {
      free_data_raw_print(sender_addr, data, datalen);
      return;
   }

   //dag_root_hex_print(sender_addr, data, datalen);
   dag_root_bin_print(sender_addr, data, datalen);
}

/*---------------------------------------------------------------------------*/

void encrypted_data_processed(const uip_ip6addr_t *sender_addr, const uint8_t *data, uint16_t datalen)
{
   printf("UDM: encrypted data received\n");
}

/*---------------------------------------------------------------------------*/

void udp_data_receiver(struct simple_udp_connection *connection,
                       const uip_ipaddr_t *sender_addr,
                       uint16_t sender_port,
                       const uip_ipaddr_t *receiver_addr,
                       uint16_t receiver_port,
                       const uint8_t *data,
                       uint16_t datalen)
{
   led_on(LED_A);

   uip_ip6addr_t node_addr;
   uip_ip6addr_copy(&node_addr, sender_addr);

   if (data[0] == PROTOCOL_VERSION_V1)
   {
      decrypted_data_processed(&node_addr, data, datalen);
   }
   else if (data[0] == PROTOCOL_VERSION_V2)
   {
      encrypted_data_processed(&node_addr, data, datalen);
   }

   led_off(LED_A);
}

/*---------------------------------------------------------------------------*/

void rpl_initialize()
{
   /* Set MESH-mode for dc-power rpl-root(not leaf-mode) */
   rpl_set_mode(RPL_MODE_MESH);

   static uip_ipaddr_t ipaddr;

   /* Fill in the address with zeros and the local prefix */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

   /* Generate an address based on the chip ID */
   uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);

   /* Adding autoconfigured address as the device address */
   uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

   /* make local address as rpl-root */
   rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
   rpl_dag_t *dag = rpl_get_any_dag();

   uip_ipaddr_t prefix;
   uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
   rpl_set_prefix(dag, &prefix, 64);

   printf("UDM: Created a new RPL DAG, i'm root!\n");

   printf("UDM: Time sync needed\n");
}

/*---------------------------------------------------------------------------*/

void root_node_initialize()
{
   /* register udp-connection, set incoming upd-data handler(udp_data_receiver) */
   simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_data_receiver);

   /* set incoming uart-data handler */
   cc26xx_uart_set_input(&uart_data_receiver_udup_v4);

   /* blink-blink LED */
   led_blink(LED_A);
   led_blink(LED_A);

   /* set LPM mode to always awake */
   lpm_register_module(&root_lpm_module);

   /* start main root process */
   process_start(&main_root_process, NULL);
}

/*---------------------------------------------------------------------------*/

void send_time_sync_resp_packet(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length)
{
   if (dest_addr == NULL && udp_connection.udp_conn == NULL)
      return;

   time_data_t root_time;
   root_time = get_epoch_time();

   uint8_t udp_buffer[PROTOCOL_VERSION_V2_16BYTE];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_SET_TIME;
   udp_buffer[3] = DATA_TYPE_SET_TIME_RESPONSE;
   udp_buffer[4] = root_time.seconds_u8[0];
   udp_buffer[5] = root_time.seconds_u8[1];
   udp_buffer[6] = root_time.seconds_u8[2];
   udp_buffer[7] = root_time.seconds_u8[3];
   udp_buffer[8] = root_time.milliseconds_u8[0];
   udp_buffer[9] = root_time.milliseconds_u8[1];
   udp_buffer[10] = data[10];
   udp_buffer[11] = data[11];
   udp_buffer[12] = data[12];
   udp_buffer[13] = data[13];
   udp_buffer[14] = data[14];
   udp_buffer[15] = data[15]; // << 16-byte packet, ready to encrypt v2 protocol

   printf("UDM: time sync responce sended: %" PRIu32 " sec, %" PRIu16 " ms\n", root_time.seconds, root_time.milliseconds);

   simple_udp_sendto(&udp_connection, udp_buffer, PROTOCOL_VERSION_V2_16BYTE, dest_addr);
}

/*---------------------------------------------------------------------------*/

void send_uart_packet(struct uart_data *uart_message)
{
   if (udp_connection.udp_conn == NULL)
      return;

   uip_ip6addr_t addr;
   uip_ip6addr_copy(&addr, &uart_message->destination_address);

   printf("UDM: returned_data_lenth: %" PRIXX8 "\n", uart_message->returned_data_lenth);
   printf("UDM: data_lenth: %" PRIXX8 "\n", uart_message->data_lenth);
   printf("UDM: payload: ");
   for (int i = 0; i < uart_message->data_lenth; i++)
   {
      printf("0x%" PRIXX8 " ", uart_message->payload[i]);
   }
   printf("\n");

   uint8_t length = 23;
   uint8_t udp_buffer[length];

   udp_buffer[0] = uart_message->protocol_version;
   udp_buffer[1] = uart_message->device_version;
   udp_buffer[2] = DATA_TYPE_UART;

   udp_buffer[3] = uart_message->returned_data_lenth;
   udp_buffer[4] = uart_message->data_lenth;
   udp_buffer[5] = uart_message->payload[0];
   udp_buffer[6] = uart_message->payload[1];
   udp_buffer[7] = uart_message->payload[2];
   udp_buffer[8] = uart_message->payload[3];
   udp_buffer[9] = uart_message->payload[4];
   udp_buffer[10] = uart_message->payload[5];
   udp_buffer[11] = uart_message->payload[6];
   udp_buffer[12] = uart_message->payload[7];
   udp_buffer[13] = uart_message->payload[8];
   udp_buffer[14] = uart_message->payload[9];
   udp_buffer[15] = uart_message->payload[10];
   udp_buffer[16] = uart_message->payload[11];
   udp_buffer[17] = uart_message->payload[12];
   udp_buffer[18] = uart_message->payload[13];
   udp_buffer[19] = uart_message->payload[14];
   udp_buffer[20] = uart_message->payload[15];
   udp_buffer[21] = DATA_RESERVED;
   udp_buffer[22] = DATA_RESERVED;

   simple_udp_sendto(&udp_connection, udp_buffer, length, &addr);
}

/*---------------------------------------------------------------------------*/

void send_firmware_cmd_packet(struct firmware_cmd *firmware_cmd_message)
{
   if (udp_connection.udp_conn == NULL)
      return;

   uip_ip6addr_t addr;
   uip_ip6addr_copy(&addr, &firmware_cmd_message->destination_address);

   uint8_t length = 10;
   uint8_t udp_buffer[length];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_FIRMWARE_CMD;
   udp_buffer[3] = DATA_TYPE_FIRMWARE_COMMAND_NEW_FW;
   udp_buffer[4] = firmware_cmd_message->chunk_quantity_b1;
   udp_buffer[5] = firmware_cmd_message->chunk_quantity_b2;
   udp_buffer[6] = DATA_RESERVED;
   udp_buffer[7] = DATA_RESERVED;
   udp_buffer[8] = DATA_RESERVED;
   udp_buffer[9] = DATA_RESERVED;
   simple_udp_sendto(&udp_connection, udp_buffer, length, &addr);
}

/*---------------------------------------------------------------------------*/

void send_firmware_packet(struct firmware_data *firmware_message)
{
   if (udp_connection.udp_conn == NULL)
      return;

   uip_ip6addr_t addr;
   uip_ip6addr_copy(&addr, &firmware_message->destination_address);

   uint16_t payload_length = firmware_message->chunk_size;
   uint16_t packet_length = payload_length + FIRMWARE_PAYLOAD_OFFSET;
   uint8_t udp_buffer[packet_length];

   udp_buffer[0] = firmware_message->protocol_version;
   udp_buffer[1] = firmware_message->device_version;
   udp_buffer[2] = DATA_TYPE_FIRMWARE;
   udp_buffer[3] = firmware_message->chunk_number_b1;
   udp_buffer[4] = firmware_message->chunk_number_b2;
   udp_buffer[5] = firmware_message->reserved_b1;
   udp_buffer[6] = firmware_message->reserved_b2; //7 = FIRMWARE_PAYLOAD_OFFSET

   for (uint16_t i = 0; i < payload_length; i++)
   {
      udp_buffer[FIRMWARE_PAYLOAD_OFFSET + i] = firmware_message->firmware_payload.data[i];
   }
   //printf("UDM: send fw packet %" PRIu16 " b\n", payload_length);
   simple_udp_sendto(&udp_connection, udp_buffer, packet_length, &addr);
}

/*---------------------------------------------------------------------------*/

void send_command_packet(struct command_data *command_message)
{
   if (udp_connection.udp_conn == NULL)
      return;

   uip_ip6addr_t addr;
   uip_ip6addr_copy(&addr, &command_message->destination_address);

   uint8_t length = 10;
   uint8_t udp_buffer[length];
   udp_buffer[0] = command_message->protocol_version;
   udp_buffer[1] = command_message->device_version;
   udp_buffer[2] = DATA_TYPE_COMMAND;
   udp_buffer[3] = command_message->ability_target;
   udp_buffer[4] = command_message->ability_number;
   udp_buffer[5] = command_message->ability_state;
   udp_buffer[6] = DATA_RESERVED;
   udp_buffer[7] = DATA_RESERVED;
   udp_buffer[8] = DATA_RESERVED;
   udp_buffer[9] = DATA_RESERVED;
   simple_udp_sendto(&udp_connection, udp_buffer, length, &addr);
}

/*---------------------------------------------------------------------------*/

void uart_packet_processed(uip_ipaddr_t node_ipaddr)
{
   uint8_t packet_datatype = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+2];

   if (packet_datatype == DATA_TYPE_COMMAND)
   {
      for (uint8_t i = 0; i < 16; i++)
         command_message.destination_address.u8[i] = node_ipaddr.u8[i]; //заменить на копирование    uip_ip6addr_copy(&node_addr, sender_addr);

      command_message.protocol_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+0];
      command_message.device_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+1];
      packet_datatype = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+2];
      command_message.ability_target = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+3];
      command_message.ability_number = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+4];
      command_message.ability_state = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+5];
      command_message.ready_to_send = 1;
   }

   else if (packet_datatype == DATA_TYPE_FIRMWARE)
   {
      printf("UDM: DATA_TYPE_FIRMWARE not implemented\n");
      return;
/*
      uint16_t fw_payload_length = uart_data_length - 23; // 23 = 16(address) + 7(packet header)
      for (uint8_t i = 0; i < 16; i++)
         firmware_message.destination_address.u8[i] = node_ipaddr.u8[i];

      firmware_message.protocol_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+0]; //16
      firmware_message.device_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+1]; //17
      packet_datatype = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+2]; //18
      firmware_message.chunk_number_b1 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+3]; //19
      firmware_message.chunk_number_b2 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+4]; //20
      firmware_message.reserved_b1 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+5]; //21
      firmware_message.reserved_b2 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+6]; //22
      firmware_message.chunk_size = fw_payload_length;

      for (uint16_t i = 0; i < fw_payload_length; i++)
         firmware_message.firmware_payload.data[i] = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+7 + i]; //23

      firmware_message.ready_to_send = 1;
*/
   }

   else if (packet_datatype == DATA_TYPE_FIRMWARE_CMD)
   {
      printf("UDM: DATA_TYPE_FIRMWARE_CMD not implemented\n");
      return;
/*
      for (uint8_t i = 0; i < 16; i++)
         firmware_cmd_message.destination_address.u8[i] = node_ipaddr.u8[i];

      firmware_cmd_message.protocol_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+0]; //16
      firmware_cmd_message.device_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+1]; //17
      packet_datatype = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+2]; //18
      firmware_cmd_message.firmware_command = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+3]; //19
      firmware_cmd_message.chunk_quantity_b1 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+4]; //20
      firmware_cmd_message.chunk_quantity_b2 = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+5]; //21
      firmware_cmd_message.ready_to_send = 1;
      //printf("UDM: chunk_quantity: 0x%" PRIXX8 " 0x%" PRIXX8 "\n", uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+4], uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+5]);
*/
   }

   else if (packet_datatype == DATA_TYPE_UART)
   {
      for (uint8_t i = 0; i < 16; i++)
         uart_message.destination_address.u8[i] = node_ipaddr.u8[i];

      uart_message.protocol_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+0]; //16
      uart_message.device_version = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+1]; //17
      packet_datatype = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+2]; //18
      uart_message.data_lenth = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+3]; //19
      uart_message.returned_data_lenth = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+4]; //20
      for (uint8_t i = 0; i < 16; i++)
         uart_message.payload[i] = uart_data_udup_v4[PAYLOAD_OFFSET_UDUP_V4+5 + i]; //21

      uart_message.ready_to_send = 1;
   }
}

/*---------------------------------------------------------------------------*/
void udup_v4_short_command_process()
{
   u8_u16_t crc16;
   uint8_t command = uart_data_udup_v4[2];

   /* Считаем CRC для коротких команд */
   crc16.u8[0] = uart_data_udup_v4[3];
   crc16.u8[1] = uart_data_udup_v4[4];

   if (false)
   {
      printf("UDM: Non-correct crc: %"PRIXX16"\n", crc16.u16);
      return;
   }

   /* Выполянем короткие команды */
   if (command == UDUP_V4_COMMAND_TYPE_REBOOT)
   {
      printf("UDM: reboot\n");
      watchdog_reboot();
   }

   if (command == UDUP_V4_COMMAND_TYPE_BOOTLOADER_ACTIVATE)
   {
      printf("UDM: bootloader activate\n");
      ti_lib_flash_sector_erase(0x0001F000);
   }

   if (command == UDUP_V4_COMMAND_TYPE_BINARY_CR_MODE)
   {
      printf("UDM: CR mode is binary(really not)\n");
   }

   if (command == UDUP_V4_COMMAND_TYPE_ASCII_CR_MODE)
   {
      printf("UDM: CR mode is ascii(really not)\n");
   }

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v4_time_command_process()
{
   u8_u16_t crc16;

   if (udup_v4_data_iterator != 18)
   {
      printf("UDM: Non-correct time set command length(iter<18b): %"PRId16"\n", udup_v4_data_iterator);
      return;
   }
   crc16.u8[0] = uart_data_udup_v4[17];
   crc16.u8[1] = uart_data_udup_v4[18];

   if (false)
   {
      printf("UDM: Non-correct crc: %"PRIXX16"\n", crc16.u16);
      return;
   }
   u8_u32_t epoch;
   epoch.u8[0] = uart_data_udup_v4[13];
   epoch.u8[1] = uart_data_udup_v4[14];
   epoch.u8[2] = uart_data_udup_v4[15];
   epoch.u8[3] = uart_data_udup_v4[16];

   time_data_t new_time;
   new_time.seconds = epoch.u32;
   new_time.milliseconds = 0;

   set_epoch_time(new_time);

   printf("UDM: epoch time: %"PRIu32"(%"PRIXX8"%"PRIXX8"%"PRIXX8"%"PRIXX8")\n", epoch.u32, epoch.u8[0], epoch.u8[1], epoch.u8[2], epoch.u8[3]);

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v4_packet_send_command_process()
{
   eui64_addr_t address;
   u8_u16_t crc16;
   u8_u16_t payload_length;

   if (udup_v4_data_iterator < 15)
   {
      printf("UDM: Non-correct packet send command length(iter<15b): %"PRId16"\n", udup_v4_data_iterator);
      return;
   }
   payload_length.u8[0] = uart_data_udup_v4[11];
   payload_length.u8[1] = uart_data_udup_v4[12];

   uint16_t all_packet_length_calculated = payload_length.u16 + 13 + 2; //Добавляем длину преамбулы(13) и crc(2)
   uint16_t all_packet_length_really = udup_v4_data_iterator + 1; //Сравниваем счетчик(от нуля) с количеством(от единицы)

   if (all_packet_length_really != all_packet_length_calculated)
   {
      printf("UDM: Non-correct packet send command length\n");
      printf("UDM: Hex payload length([0], [1]): 0x%"PRIXX8" 0x%"PRIXX8", payload length: %"PRIu16"\n", payload_length.u8[0], payload_length.u8[1], payload_length.u16);
      printf("UDM: packet length calc(payload+13+2): %"PRId16", really: %"PRId16"\n", all_packet_length_calculated, all_packet_length_really);
      return;
   }

   crc16.u8[0] = uart_data_udup_v4[payload_length.u16 + 13];
   crc16.u8[1] = uart_data_udup_v4[payload_length.u16 + 14];
   if (false)
   {
      printf("UDM: Non-correct crc: %"PRIXX16"\n", crc16.u16);
      return;
   }

   address.addr[0] = uart_data_udup_v4[3];
   address.addr[1] = uart_data_udup_v4[4];
   address.addr[2] = uart_data_udup_v4[5];
   address.addr[3] = uart_data_udup_v4[6];
   address.addr[4] = uart_data_udup_v4[7];
   address.addr[5] = uart_data_udup_v4[8];
   address.addr[6] = uart_data_udup_v4[9];
   address.addr[7] = uart_data_udup_v4[10];

   static uip_ipaddr_t node_ipaddr;
   uip_ip6addr_u8(&node_ipaddr, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, address.addr[0], address.addr[1], address.addr[2], address.addr[3], address.addr[4], address.addr[5], address.addr[6], address.addr[7]);

   uart_packet_processed(node_ipaddr);

   printf("UDM: adress: ");
   uip_debug_ipaddr_print(&node_ipaddr);
   printf(", crc: %"PRIXX16"", crc16.u16);
   printf(", payload length: %"PRIXX16"", payload_length.u16);
   printf(", data_iter: %"PRId16"", udup_v4_data_iterator);
   printf("\n");
}

/*---------------------------------------------------------------------------*/
int uart_data_receiver_udup_v4(unsigned char uart_char)
{
   led_blink(LED_A);
   if (udup_v4_data_iterator < MAX_UART_PACKET_LENGTH_UDUP_V4)
   {
      uart_data_udup_v4[udup_v4_data_iterator] = uart_char;
      udup_v4_data_iterator++;
      timer_restart(&udup_v4_timeout_timer);
   }
   return 1;
}

/*---------------------------------------------------------------------------*/
void udup_v4_data_process()
{
   printf("UDM: UDUP packet: ");
   for (uint16_t i = 0; i <= udup_v4_data_iterator; i++)
      printf("%"PRIXX8" ", uart_data_udup_v4[i]);
   printf("\n");

   /* Проверяем символ начала пакета */
   if (udup_v4_data_iterator < 4)
   {
      printf("UDM: Non-correct packet length(length<4b): %"PRId16"\n", udup_v4_data_iterator + 1);
      return;
   }

   uint8_t mq = uart_data_udup_v4[0];
   uint8_t version = uart_data_udup_v4[1];
   uint8_t command = uart_data_udup_v4[2];

   /* Проверяем символ начала пакета */
   if (mq != MAGIC_SEQUENCE_UDUP_V4)
   {
      printf("UDM: Non-correct MQ: 0x%"PRIXX8"\n", mq);
      return;
   }

   /* Проверяем версию протокола */
   if (version != UART_PROTOCOL_VERSION_V4)
   {
      printf("UDM: Non-correct protocol version: 0x%"PRIXX8"\n", version);
      return;
   }

    /* Считаем CRC и выполняем короткие команды */
   if (command == UDUP_V4_COMMAND_TYPE_REBOOT ||
      command == UDUP_V4_COMMAND_TYPE_BOOTLOADER_ACTIVATE ||
      command == UDUP_V4_COMMAND_TYPE_BINARY_CR_MODE ||
      command == UDUP_V4_COMMAND_TYPE_ASCII_CR_MODE)
      udup_v4_short_command_process();

   /* Считаем CRC и выполняем команду времени */
   else if (command == UDUP_V4_COMMAND_TYPE_ROOT_TIME_SET)
      udup_v4_time_command_process();

   /* Считаем CRC и выполняем команду отправки пакета */
   else if (command == UDUP_V4_COMMAND_TYPE_PACKET_SEND)
      udup_v4_packet_send_command_process();

   else
      printf("UDM: Non-correct command: 0x%"PRIXX8"\n", command);

   return;
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_root_process, ev, data)
{
   PROCESS_BEGIN();
   timer_set(&udup_v4_timeout_timer, 2);

   static struct etimer main_root_process_timer;
   PROCESS_PAUSE();

   while (1)
   {
      etimer_set(&main_root_process_timer, 1);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&main_root_process_timer));

      if (timer_expired(&udup_v4_timeout_timer) && udup_v4_data_iterator > 0)
      {
         disable_interrupts();
         udup_v4_data_iterator--;
         udup_v4_data_process();
         udup_v4_data_iterator = 0;
         enable_interrupts();
      }

      if (command_message.ready_to_send != 0)
      {
         disable_interrupts();
         send_command_packet(&command_message);
         command_message.ready_to_send = 0;
         enable_interrupts();
      }

   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/
