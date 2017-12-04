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
#include "crypto-common.h"
#include "rtc-common.h"

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5 //in main timer ticks, one tick ~8ms

/*---------------------------------------------------------------------------*/

void send_time_sync_resp_packet(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

uint8_t udup_v5_rc_uart_rx_buffer[UDUP_V5_RC_MAX_LENGTH];
uint8_t udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_MAX_LENGTH];

static uint16_t udup_v5_data_iterator = 0;
root_udup_message_type_t udup_v5_bin_hex_type = ROOT_UDUP_V5_HEX_MSG;
static struct timer udup_v5_timeout_timer;

PROCESS(main_root_process, "main root process");


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

void udup_v5_dag_root_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
   if (addr == NULL || data == NULL)
      return;

   u8_u16_t payload_length;
   u8_u16_t crc16_calculated;
   uint8_t payload_offset = 0;
   uint8_t udbp_version = 0x04;

   if (version == UDBP_PROTOCOL_VERSION_V5)
   {
      payload_offset = UDBP_V5_HEADER_LENGTH;
      udbp_version = 0x05;
   }

   udup_v5_cr_uart_tx_buffer[0] = UDUP_V5_MAGIC_BYTE;
   udup_v5_cr_uart_tx_buffer[1] = udbp_version;
   udup_v5_cr_uart_tx_buffer[2] = UDUP_V5_COMMAND_TYPE_NET_PACKET;

   udup_v5_cr_uart_tx_buffer[3] = ((uint8_t *)addr)[8];
   udup_v5_cr_uart_tx_buffer[4] = ((uint8_t *)addr)[9];
   udup_v5_cr_uart_tx_buffer[5] = ((uint8_t *)addr)[10];
   udup_v5_cr_uart_tx_buffer[6] = ((uint8_t *)addr)[11];
   udup_v5_cr_uart_tx_buffer[7] = ((uint8_t *)addr)[12];
   udup_v5_cr_uart_tx_buffer[8] = ((uint8_t *)addr)[13];
   udup_v5_cr_uart_tx_buffer[9] = ((uint8_t *)addr)[14];
   udup_v5_cr_uart_tx_buffer[10] = ((uint8_t *)addr)[15];


   if (version == UDBP_PROTOCOL_VERSION_V5)
   {
      udup_v5_cr_uart_tx_buffer[11] = data[5]; //voltage
      udup_v5_cr_uart_tx_buffer[12] = data[3]; //rssi
   }
   else
   {
      /* Призрак будущих полей */
      udup_v5_cr_uart_tx_buffer[11] = 0xDE; //voltage
      udup_v5_cr_uart_tx_buffer[12] = 0xAD; //rssi
   }

   /* Считаем длину пакета */
   if (length - payload_offset > UDUP_V5_CR_MAX_PAYLOAD_LENGTH)
      payload_length.u16 = UDUP_V5_CR_MAX_PAYLOAD_LENGTH;
   else
      payload_length.u16 = length - payload_offset;
   udup_v5_cr_uart_tx_buffer[13] = payload_length.u8[0];
   udup_v5_cr_uart_tx_buffer[14] = payload_length.u8[1];

   /* Переносим данные */
   for (uint16_t i = 0; i < payload_length.u16; i++) {
      udup_v5_cr_uart_tx_buffer[i + UDUP_V5_CR_PAYLOAD_OFFSET] = data[i + payload_offset];
      //printf("%" PRIXX8 " ", data[i + payload_offset]);
   }
   //printf("\n");

   /* Считаем контрольную сумму */
   crc16_calculated.u16 = crc16_arc(udup_v5_cr_uart_tx_buffer, UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16);
   udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+0] = crc16_calculated.u8[1]; //Меняем порядок байт на MSB-First
   udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+1] = crc16_calculated.u8[0];


   /* Выводим весь пакет в UART в зависимости от настроек */
   if (udup_v5_bin_hex_type == ROOT_UDUP_V5_HEX_MSG)
   {
      for (uint16_t i = 0; i < UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16 + UDUP_V5_CRC_LENGTH; i++)
         printf("%"PRIXX8, udup_v5_cr_uart_tx_buffer[i]);
      printf("\n");
   }
   else if (udup_v5_bin_hex_type == ROOT_UDUP_V5_BIN_MSG)
   {
      for (uint16_t i = 0; i < UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16 + UDUP_V5_CRC_LENGTH; i++)
          cc26xx_uart_write_byte(udup_v5_cr_uart_tx_buffer[i]);
   }

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
/*
   printf("Raw net packet: ");
   for (uint16_t i = 0; i < datalen; i++) {
      printf("%" PRIXX8 " ", data[i]);
   }
   printf("\n");
*/
   uip_ip6addr_t node_addr;
   uip_ip6addr_copy(&node_addr, sender_addr);

   uint8_t protocol_version = data[0];

   if (protocol_version == PROTOCOL_VERSION_V1)
   {
      uint8_t v_1_packet_type = data[2];
      uint8_t v_1_packet_subtype = data[3];

      if (v_1_packet_type == DATA_TYPE_SENSOR_DATA)
      {
         send_pong_packet(&node_addr);
      }

      else if (v_1_packet_type == DATA_TYPE_SET_TIME && v_1_packet_subtype == DATA_TYPE_SET_TIME_REQUEST)
      {
         send_time_sync_resp_packet(&node_addr, data, datalen);
         return;
      }

      //udup_v5_dag_root_print(&node_addr, data, datalen, PROTOCOL_VERSION_V1);
   }
   else if (protocol_version == UDBP_PROTOCOL_VERSION_V5)
   {
      uint8_t v_5_module_id = data[UDBP_V5_HEADER_LENGTH + 0];
      uint8_t v_5_packet_type = data[UDBP_V5_HEADER_LENGTH + 1];

      if (v_5_module_id == UNWDS_CONFIG_MODULE_ID)
      {
         if (v_5_packet_type == DATA_TYPE_JOIN)
         {
            send_confirmation_packet(&node_addr);
            led_off(LED_A);
            return;
         }
         if (v_5_packet_type == DATA_TYPE_STATUS)
         {
            send_pong_packet(&node_addr);
         }
      }

      udup_v5_dag_root_print(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
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
   cc26xx_uart_set_input(&uart_data_receiver_udup_v5);

   /* blink-blink LED */
   led_blink(LED_A);
   led_blink(LED_A);

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

void udup_v5_to_net_packet_processed(uip_ipaddr_t node_ipaddr, uint16_t payload_length)
{
   if (udp_connection.udp_conn == NULL)
      return;

   uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
   udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
   udp_buffer[1] = 0xFF; //Счетчик, 1 байт
   udp_buffer[2] = 0xFF; //Счетчик, 2 байт
   udp_buffer[3] = 0xFF; //rssi
   udp_buffer[4] = 0xFF; //temp
   udp_buffer[5] = 0xFF; //voltage

   for (uint16_t i = 0; i < payload_length; i++)
      udp_buffer[i + UDBP_V5_HEADER_LENGTH] = udup_v5_rc_uart_rx_buffer[UDUP_V5_RC_PAYLOAD_OFFSET+i];

   simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &node_ipaddr);
}
/*---------------------------------------------------------------------------*/
void udup_v5_short_command_process()
{
   u8_u16_t crc16_in_packet;
   uint8_t command = udup_v5_rc_uart_rx_buffer[2];

   /* Считаем CRC для коротких команд */
   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, 3);
   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[4]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[3];

   printf("UDM: crc: 0x%"PRIXX16"\n", crc16_in_packet.u16);

   if (crc16_calculated != crc16_in_packet.u16)
   {
      printf("UDM: Non-correct crc, packet: 0x%"PRIXX16", calculated: 0x%"PRIXX16"\n", crc16_in_packet.u16, crc16_calculated);
      return;
   }

   /* Выполянем короткие команды */
   if (command == UDUP_V5_COMMAND_TYPE_REBOOT)
   {
      printf("UDM: reboot\n");
      watchdog_reboot();
   }

   if (command == UDUP_V5_COMMAND_TYPE_BOOTLOADER_ACTIVATE)
   {
      printf("UDM: bootloader activate\n");
      ti_lib_flash_sector_erase(0x0001F000);
   }

   if (command == UDUP_V5_COMMAND_TYPE_BINARY_CR_MODE)
   {
      udup_v5_bin_hex_type = ROOT_UDUP_V5_BIN_MSG;
      printf("UDM: CR mode is binary\n");
   }

   if (command == UDUP_V5_COMMAND_TYPE_ASCII_CR_MODE)
   {
      udup_v5_bin_hex_type = ROOT_UDUP_V5_HEX_MSG;
      printf("UDM: CR mode is ascii-hex\n");
   }

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v5_time_command_process()
{
   u8_u16_t crc16_in_packet;

   if (udup_v5_data_iterator != 18)
   {
      printf("UDM: Non-correct time set command length(iter<18b): %"PRId16"\n", udup_v5_data_iterator);
      return;
   }
   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, 17);

   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[18]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[17];

   if (crc16_calculated != crc16_in_packet.u16)
   {
      printf("UDM: Non-correct crc, packet: 0x%"PRIXX16", calculated: 0x%"PRIXX16"\n", crc16_in_packet.u16, crc16_calculated);
      return;
   }

   u8_u32_t epoch;
   epoch.u8[0] = udup_v5_rc_uart_rx_buffer[13];
   epoch.u8[1] = udup_v5_rc_uart_rx_buffer[14];
   epoch.u8[2] = udup_v5_rc_uart_rx_buffer[15];
   epoch.u8[3] = udup_v5_rc_uart_rx_buffer[16];

   time_data_t new_time;
   new_time.seconds = epoch.u32;
   new_time.milliseconds = 0;

   set_epoch_time(new_time);

   printf("UDM: epoch time: %"PRIu32"(%"PRIXX8"%"PRIXX8"%"PRIXX8"%"PRIXX8")\n", epoch.u32, epoch.u8[0], epoch.u8[1], epoch.u8[2], epoch.u8[3]);

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v5_packet_net_packet_process()
{
   u8_u16_t crc16_in_packet;
   u8_u16_t payload_length;

   if (udup_v5_data_iterator < 15)
   {
      printf("UDM: Non-correct packet send command length(iter<15b): %"PRId16"\n", udup_v5_data_iterator);
      return;
   }
   payload_length.u8[0] = udup_v5_rc_uart_rx_buffer[12]; //Меняем порядок байт на MSB-First
   payload_length.u8[1] = udup_v5_rc_uart_rx_buffer[11];

   uint16_t all_packet_length_calculated = payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + UDUP_V5_CRC_LENGTH;
   uint16_t all_packet_length_really = udup_v5_data_iterator + 1; //Сравниваем счетчик(от нуля) с количеством(от единицы)

   if (all_packet_length_really != all_packet_length_calculated)
   {
      printf("UDM: Non-correct packet send command length\n");
      printf("UDM: Hex payload length([0], [1]): 0x%"PRIXX8" 0x%"PRIXX8", payload length: %"PRIu16"\n", payload_length.u8[0], payload_length.u8[1], payload_length.u16);
      printf("UDM: packet length calc(payload+13+2): %"PRId16", really: %"PRId16"\n", all_packet_length_calculated, all_packet_length_really);
      return;
   }

   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, all_packet_length_calculated - UDUP_V5_CRC_LENGTH);

   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + 1]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + 0];

   if (crc16_calculated != crc16_in_packet.u16)
   {
      printf("UDM: Non-correct crc, packet: 0x%"PRIXX16", calculated: 0x%"PRIXX16"\n", crc16_in_packet.u16, crc16_calculated);
      return;
   }

   static uip_ipaddr_t node_ipaddr;
   uip_ip6addr_u8(&node_ipaddr, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      udup_v5_rc_uart_rx_buffer[3],
      udup_v5_rc_uart_rx_buffer[4],
      udup_v5_rc_uart_rx_buffer[5],
      udup_v5_rc_uart_rx_buffer[6],
      udup_v5_rc_uart_rx_buffer[7],
      udup_v5_rc_uart_rx_buffer[8],
      udup_v5_rc_uart_rx_buffer[9],
      udup_v5_rc_uart_rx_buffer[10]);



   udup_v5_to_net_packet_processed(node_ipaddr, payload_length.u16);

   printf("UDM: adress: ");
   uip_debug_ipaddr_print(&node_ipaddr);
   printf(", payload length: %"PRIXX16"", payload_length.u16);
   printf(", data_iter: %"PRId16"", udup_v5_data_iterator);
   printf("\n");
}

/*---------------------------------------------------------------------------*/
int uart_data_receiver_udup_v5(unsigned char uart_char)
{
   led_blink(LED_A);
   if (udup_v5_data_iterator < UDUP_V5_RC_MAX_LENGTH)
   {
      udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator] = uart_char;
      udup_v5_data_iterator++;
      timer_restart(&udup_v5_timeout_timer);
   }
   return 1;
}

/*---------------------------------------------------------------------------*/
void udup_v5_data_process()
{
   printf("UDM: UDUP packet: ");
   for (uint16_t i = 0; i <= udup_v5_data_iterator; i++)
      printf("%"PRIXX8" ", udup_v5_rc_uart_rx_buffer[i]);
   printf("\n");

   /* Проверяем символ начала пакета */
   if (udup_v5_data_iterator < 4)
   {
      printf("UDM: Non-correct packet length(length<4b): %"PRId16"\n", udup_v5_data_iterator + 1);
      return;
   }

   uint8_t mq = udup_v5_rc_uart_rx_buffer[0];
   uint8_t version = udup_v5_rc_uart_rx_buffer[1];
   uint8_t command = udup_v5_rc_uart_rx_buffer[2];

   /* Проверяем символ начала пакета */
   if (mq != UDUP_V5_MAGIC_BYTE)
   {
      printf("UDM: Non-correct MQ: 0x%"PRIXX8"\n", mq);
      return;
   }

   /* Проверяем версию протокола */
   if (version == UDUP_V5_PROTOCOL_VERSION)
   {
      /* Считаем CRC и обрабатываем сетевой пакет */
      if (command == UDUP_V5_COMMAND_TYPE_NET_PACKET)
         udup_v5_packet_net_packet_process();

      /* Считаем CRC и выполняем короткие команды */
      else if (command == UDUP_V5_COMMAND_TYPE_REBOOT ||
               command == UDUP_V5_COMMAND_TYPE_BOOTLOADER_ACTIVATE ||
               command == UDUP_V5_COMMAND_TYPE_BINARY_CR_MODE ||
               command == UDUP_V5_COMMAND_TYPE_ASCII_CR_MODE)
         udup_v5_short_command_process();

      /* Считаем CRC и выполняем команду времени */
      else if (command == UDUP_V5_COMMAND_TYPE_ROOT_TIME_SET)
         udup_v5_time_command_process();

      else
         printf("UDM: Non-correct command: 0x%"PRIXX8"\n", command);
   }
   else
   {
      printf("UDM: Non-correct protocol version: 0x%"PRIXX8"\n", version);
      return;
   }

   return;
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_root_process, ev, data)
{
   PROCESS_BEGIN();
   timer_set(&udup_v5_timeout_timer, 2);

   static struct etimer main_root_process_timer;
   PROCESS_PAUSE();

   while (1)
   {
      etimer_set(&main_root_process_timer, 1);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&main_root_process_timer));

      if (timer_expired(&udup_v5_timeout_timer) && udup_v5_data_iterator > 0)
      {
         disable_interrupts();
         udup_v5_data_iterator--;
         udup_v5_data_process();
         udup_v5_data_iterator = 0;
         enable_interrupts();
      }

   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/
