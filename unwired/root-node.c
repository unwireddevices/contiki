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

#include "dev/serial-line.h"
#include "../cpu/cc26xx-cc13xx/dev/cc26xx-uart.h"

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

#define MAX_ROUTE_TABLE			10  //Максимальное количество счетчиков в сети
#define WAIT_RESPONSE 			3 	//Максимальное время ожидания ответа от счетчика в секундах
#define AES128_PACKAGE_LENGTH	16	//Длина пакета AES-128

/*---------------------------------------------------------------------------*/

void send_time_sync_resp_packet(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length);

uint8_t udup_v5_rc_uart_rx_buffer[UDUP_V5_RC_MAX_LENGTH+1]; //+1 for \n(0x0A)
uint8_t udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_MAX_LENGTH];

uint8_t aes_key[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
uint8_t aes_buffer[128];
uint8_t nonce_key[16];


static uint16_t udup_v5_data_iterator = 0;
static struct timer udup_v5_timeout_timer;
static struct ctimer wait_response;

static bool uart = 0;
static bool wait_response_slave = 0;

struct route 
{
    uint32_t serial;
    uip_ip6addr_t addr;
	uint16_t nonce;
	uint16_t counter;
};

struct route route_table[MAX_ROUTE_TABLE];
uint8_t route_table_ptr = 0;

volatile union { uint16_t u16; uint8_t u8[2]; } packet_counter_root;

PROCESS(main_root_process, "main root process");


/*---------------------------------------------------------------------------*/
static void wait_response_reset(void *ptr)
{
   wait_response_slave = 0;
}
 
/*---------------------------------------------------------------------------*/ 
void add_route(uint32_t serial, uip_ip6addr_t addr, uint16_t nonce)
{
	if(route_table_ptr >= MAX_ROUTE_TABLE) //Проверка на макс размер таблицы
		return;
	
	for(uint8_t i = 0; i < route_table_ptr; i++) //Проверка есть ли такой серийник
	{
		if(route_table[i].serial == serial)
		{
			route_table[i].addr = addr;
			route_table[i].nonce = nonce;
			route_table[i].counter = 0xFFFF;
			//printf("Dont add serial: %lu\n", serial);
			//uip_debug_ipaddr_print(&addr);
			//printf("route_table_ptr: %i\n", route_table_ptr);
			return;
		}
	}
	
	//printf("Add serial: %lu\n", serial);
	//uip_debug_ipaddr_print(&addr);
	//printf("route_table_ptr: %i\n", route_table_ptr);
	route_table[route_table_ptr].serial = serial; //Добавляем в таблицу
	route_table[route_table_ptr].addr = addr;
	route_table[route_table_ptr].nonce = nonce;
	route_table[route_table_ptr].counter = 0xFFFF; //Добавляется в таблицу, но не будет работать, пока счетчик не обнулится
	route_table_ptr++;
}

/*---------------------------------------------------------------------------*/
uip_ip6addr_t find_addr(uint32_t serial)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		//printf("%lu == %lu\n", route_table[i].serial, serial);
		if(route_table[i].serial == serial)
		{
			//printf("Find serial: %lu\n", serial);
			//uip_debug_ipaddr_print(&route_table[i].addr);
			//printf("route_table_ptr: %i\n", i);
			if(route_table[i].counter != 0xFFFF) //Проверка на активность.
				return route_table[i].addr;
		}
	}
	
	uip_ip6addr_t addr_not_found;
	uip_ip6addr(&addr_not_found, 0, 0, 0, 0, 0, 0, 0, 0);
	return addr_not_found;
	
	//printf("Dont find:  %lu\n", serial);
	//return 0; //route_table[0].addr; //ВОЗВРАЩАЕТ ЧТО ТАКОГО АДРЕССА НЕТ, А ПОКА ЧТО ХУЕТУ ВОЗВРАЩАЕТ
}
/*---------------------------------------------------------------------------*/
uint16_t get_nonce(uint32_t serial)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			return route_table[i].nonce;
		}
	}
	return 0;
	
	//printf("Dont find:  %lu\n", serial);
	//return 0; //route_table[0].addr; //ВОЗВРАЩАЕТ ЧТО ТАКОГО АДРЕССА НЕТ, А ПОКА ЧТО ХУЕТУ ВОЗВРАЩАЕТ
}
/*---------------------------------------------------------------------------*/
static void unlock_addr(uint32_t serial)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		//printf("%lu == %lu\n", route_table[i].serial, serial);
		if(route_table[i].serial == serial)
		{
			//printf("Find serial: %lu\n", serial);
			//uip_debug_ipaddr_print(&route_table[i].addr);
			//printf("route_table_ptr: %i\n", i);
			if(route_table[i].counter == 0xFFFF) //Проверка на активность.
				route_table[i].counter = 0x0000;
		}
	}
}
/*---------------------------------------------------------------------------*/
static void udbp_v5_rx_uart_from_air_to_tx_handler(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	if(wait_response_slave == 0)
		return;
	
	//Дешифрово4kа
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[UDBP_V5_HEADER_LENGTH], (uint32_t*)(aes_buffer), (length - UDBP_V5_HEADER_LENGTH));
	
	for(uint16_t i = 0; i < aes_buffer[2]; i++)
		cc26xx_uart_write_byte(aes_buffer[i + 3]);
	
	//for (uint16_t i = 0; i < length - 6; i++) 
	//{
	//	cc26xx_uart_write_byte(data[i + 6]);
	//}
}

/*---------------------------------------------------------------------------*/

void udbp_v5_join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	printf("udbp_v5_join_stage_2_sender\n");
	if (dest_addr == NULL)
		return;
   
	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	uint8_t payload_length = 18; //2 HEADER + 16 AES
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_root.u8[0];
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_2;//packet_counter_root.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_root.u8[1];//DATA_TYPE_JOIN_V5_STAGE_2;
	
	uint16_t nonce = random_rand();
		
	add_route((uint32_t)((data[UDBP_V5_HEADER_LENGTH + 8] << 24) |
						 (data[UDBP_V5_HEADER_LENGTH + 9] << 16) |
						 (data[UDBP_V5_HEADER_LENGTH + 10] << 8) |
						 (data[UDBP_V5_HEADER_LENGTH + 11] )), 
						 addr,
						 nonce);
						 
	aes_buffer[0] = (uint8_t)((nonce >> 8) & 0xFF);
	aes_buffer[1] = (uint8_t)(nonce & 0xFF);
	
	printf(" %"PRIXX8, aes_buffer[0]);
	printf(" %"PRIXX8, aes_buffer[1]);
	printf("\n");
	
	for(uint8_t i = 2; i < 16; i++)
		aes_buffer[i] = 0x00;
	
	aes_ecb_encrypt((uint32_t*)aes_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[8]));
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, aes_buffer[i]);
	}
	printf("\n");
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, udp_buffer[i+8]);
	}
	printf("\n");
	
	//udp_buffer[8] = DATA_RESERVED;
	//udp_buffer[9] = DATA_RESERVED;
	//udp_buffer[10] = DATA_RESERVED;
	//udp_buffer[11] = DATA_RESERVED;
	//udp_buffer[12] = DATA_RESERVED;
	//udp_buffer[13] = DATA_RESERVED;
	//udp_buffer[14] = DATA_RESERVED;
	//udp_buffer[15] = DATA_RESERVED;
	//udp_buffer[16] = DATA_RESERVED;
	//udp_buffer[17] = DATA_RESERVED;
	//udp_buffer[18] = DATA_RESERVED;
	//udp_buffer[19] = DATA_RESERVED;
	//udp_buffer[20] = DATA_RESERVED;
	//udp_buffer[21] = DATA_RESERVED;
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/
void udbp_v5_join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	printf("udbp_v5_join_stage_4_sender\n");
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	uint16_t nonce = get_nonce((uint32_t) ( (data[UDBP_V5_HEADER_LENGTH + 2] << 24) |
											(data[UDBP_V5_HEADER_LENGTH + 3] << 16) |
											(data[UDBP_V5_HEADER_LENGTH + 4] << 8)  |
											(data[UDBP_V5_HEADER_LENGTH + 5] )));
	
	nonce_key[0] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[1] = (uint8_t)(nonce & 0xFF);
	nonce_key[2] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[3] = (uint8_t)(nonce & 0xFF);
	nonce_key[4] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[5] = (uint8_t)(nonce & 0xFF);
	nonce_key[6] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[7] = (uint8_t)(nonce & 0xFF);
	nonce_key[8] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[9] = (uint8_t)(nonce & 0xFF);
	nonce_key[10] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[11] = (uint8_t)(nonce & 0xFF);
	nonce_key[12] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[13] = (uint8_t)(nonce & 0xFF);
	nonce_key[14] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[15] = (uint8_t)(nonce & 0xFF);
	
	//uint16_t nonce = 0xDEAD;
	//aes_buffer[0] = (uint8_t)((nonce >> 8) & 0xFF);
	//aes_buffer[1] = (uint8_t)(nonce & 0xFF);
	//for(uint8_t i = 2; i < 16; i++)
		//aes_buffer[i] = 0x00;
	
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[12], (uint32_t*)(aes_buffer), 16);
								
	if(aes_buffer[0] == nonce_key[0] && aes_buffer[1] == nonce_key[1])
	{		
		unlock_addr((uint32_t) ((data[UDBP_V5_HEADER_LENGTH + 2] << 24) |
								(data[UDBP_V5_HEADER_LENGTH + 3] << 16) |
								(data[UDBP_V5_HEADER_LENGTH + 4] << 8) |
								(data[UDBP_V5_HEADER_LENGTH + 5] )));
	}
						 
	uint8_t payload_length = 18; //2 HEADER + 16 AES
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID; //packet_counter_root.u8[0];
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_4; //packet_counter_root.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0]; //UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_root.u8[1]; //DATA_TYPE_JOIN_V5_STAGE_4;
	
	for(uint8_t i = 0; i < 16; i++)
		aes_buffer[i] = 0x00;
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[8]), 16);
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, nonce_key[i]);
	}
	printf("\n");
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, aes_buffer[i]);
	}
	printf("\n");
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, udp_buffer[i+8]);
	}
	printf("\n");
	
	//udp_buffer[8] = DATA_RESERVED;
	//udp_buffer[9] = DATA_RESERVED;
	//udp_buffer[10] = DATA_RESERVED;
	//udp_buffer[11] = DATA_RESERVED;
	//udp_buffer[12] = DATA_RESERVED;
	//udp_buffer[13] = DATA_RESERVED;
	//udp_buffer[14] = DATA_RESERVED;
	//udp_buffer[15] = DATA_RESERVED;
	//udp_buffer[16] = DATA_RESERVED;
	//udp_buffer[17] = DATA_RESERVED;
	//udp_buffer[18] = DATA_RESERVED;
	//udp_buffer[19] = DATA_RESERVED;
	//udp_buffer[20] = DATA_RESERVED;
	//udp_buffer[21] = DATA_RESERVED;
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/
void udbp_v5_ack_packet_sender(const uip_ip6addr_t *dest_addr)
{
	//printf("udbp_v5_ack_packet_sender\n");
   if (dest_addr == NULL)
      return;

   uip_ipaddr_t addr;
   uip_ip6addr_copy(&addr, dest_addr);

   uint8_t payload_length = 16;
   uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
   udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
   udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_root.u8[0];
   udp_buffer[2] = DATA_TYPE_ACK;//packet_counter_root.u8[1];
   udp_buffer[3] = get_parent_rssi();
   udp_buffer[4] = get_temperature();
   udp_buffer[5] = get_voltage();

   udp_buffer[6] = packet_counter_root.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
   udp_buffer[7] = packet_counter_root.u8[1];//DATA_TYPE_ACK;
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
   simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
   packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/

void udup_v5_dag_root_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	
	//printf("udup_v5_dag_root_print\n");
	if(uart == 1)
		return;
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
   udup_v5_cr_uart_tx_buffer[13] = payload_length.u8[1]; //Меняем порядок байт на MSB-First
   udup_v5_cr_uart_tx_buffer[14] = payload_length.u8[0];

   /* Переносим данные */
   for (uint16_t i = 0; i < payload_length.u16; i++) {
      udup_v5_cr_uart_tx_buffer[i + UDUP_V5_CR_PAYLOAD_OFFSET] = data[i + payload_offset];
      //printf("%" PRIXX8 " ", data[i + payload_offset]);
   }
   //printf("\n");
   //cc26xx_uart_write_byte(0x55);

   /* Считаем контрольную сумму */
   crc16_calculated.u16 = crc16_arc(udup_v5_cr_uart_tx_buffer, UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16);
   udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+0] = crc16_calculated.u8[1]; //Меняем порядок байт на MSB-First
   udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+1] = crc16_calculated.u8[0];

   /*for (uint16_t i = 0; i < payload_length.u16; i++) {
	  cc26xx_uart_write_byte(data[i + payload_offset]);
      //printf("%" PRIXX8 " ", data[i + payload_offset]);
   }*/

   /* Выводим весь пакет в UART в зависимости от настроек */
   if(uart_status_r() == 0)
   {
	   for (uint16_t i = 0; i < UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16 + UDUP_V5_CRC_LENGTH; i++)
		  printf("%"PRIXX8, udup_v5_cr_uart_tx_buffer[i]);
	   printf("\n");
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
	//printf("udp_data_receiver\n");
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

   if (protocol_version == UDBP_PROTOCOL_VERSION_V5)
   {
      uint8_t v_5_module_id = data[UDUP_V5_MODULE_ID];
      uint8_t v_5_packet_type = data[UDUP_V5_PACKET_TYPE];

      if (v_5_module_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
      {
         if (v_5_packet_type == DATA_TYPE_JOIN_V5_STAGE_1)
         {
            udbp_v5_join_stage_2_sender(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
            led_off(LED_A);
            return;
         }
		 else if(v_5_packet_type == DATA_TYPE_JOIN_V5_STAGE_3)
		 {
			udbp_v5_join_stage_4_sender(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
			led_off(LED_A);
            return;
		 }
		 else if(v_5_packet_type == UART_FROM_RX_TO_AIR)
		 {
			udbp_v5_rx_uart_from_air_to_tx_handler(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
			led_off(LED_A);
            return;
		 }
      }
      udbp_v5_ack_packet_sender(&node_addr);
      udup_v5_dag_root_print(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
   }

   led_off(LED_A);
}

/*---------------------------------------------------------------------------*/

void rpl_initialize()
{
	//printf("rpl_initialize\n");
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

   if(uart_status_r() == 0)
	 printf("UDM: Created a new RPL DAG, i'm root!\n");
 
   if(uart_status_r() == 0)
     printf("UDM: Time sync needed\n");
}

/*---------------------------------------------------------------------------*/

void root_node_initialize()
{
	//printf("root_node_initialize\n");
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
	//printf("send_time_sync_resp_packet\n");
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

   if(uart_status_r() == 0)
     printf("UDM: time sync responce sended: %" PRIu32 " sec, %" PRIu16 " ms\n", root_time.seconds, root_time.milliseconds);

   simple_udp_sendto(&udp_connection, udp_buffer, PROTOCOL_VERSION_V2_16BYTE, dest_addr);
}


/*---------------------------------------------------------------------------*/

void udup_v5_to_net_packet_processed(uip_ipaddr_t node_ipaddr, uint16_t payload_length)
{
	//printf("udup_v5_to_net_packet_processed\n");
   if (udp_connection.udp_conn == NULL)
      return;

   uint16_t addition_length = ((((payload_length-1)/MINIMAL_BLOCK)+1)*MINIMAL_BLOCK)-payload_length;

   //printf("addition_length ff: %"PRIu8" + %"PRIu8" = %"PRIu8"\n", payload_length, addition_length, payload_length + addition_length);

   uint16_t packet_length = payload_length + addition_length + UDBP_V5_HEADER_LENGTH;
   uint8_t udp_buffer[packet_length];
   udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
   udp_buffer[1] = 0xDE; //Счетчик, 1 байт
   udp_buffer[2] = 0xAD; //Счетчик, 2 байт
   udp_buffer[3] = 0xBE; //rssi
   udp_buffer[4] = 0xEE; //temp
   udp_buffer[5] = 0xEF; //voltage

   for (uint16_t i = 0; i < payload_length; i++)
      udp_buffer[i + UDBP_V5_HEADER_LENGTH] = udup_v5_rc_uart_rx_buffer[UDUP_V5_RC_PAYLOAD_OFFSET+i];

   for (uint8_t i = 0; i < addition_length; i++)
      udp_buffer[i + payload_length + UDBP_V5_HEADER_LENGTH] = 0xFF;

/*
   printf("Payload(%"PRIu16"): ", payload_length);
   for (uint16_t i = 0; i < payload_length; i++)
   {
      printf("%"PRIXX8, udp_buffer[i + UDBP_V5_HEADER_LENGTH]);
   }
   printf("\n");

   printf("Packet(%"PRIu16" + %"PRIu16" + %"PRIu16" = %"PRIu16"): ", payload_length, addition_length, UDBP_V5_HEADER_LENGTH, packet_length);
   for (uint16_t i = 0; i < packet_length; i++)
   {
      printf("%"PRIXX8, udp_buffer[i]);
   }
   printf("\n");
*/

   simple_udp_sendto(&udp_connection, udp_buffer, packet_length, &node_ipaddr);
}
/*---------------------------------------------------------------------------*/
void udup_v5_short_command_process()
{
	//printf("udup_v5_short_command_process\n");
   u8_u16_t crc16_in_packet;
   uint8_t command = udup_v5_rc_uart_rx_buffer[2];

   /* Считаем CRC для коротких команд */
   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, 3);
   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[4]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[3];

   //printf("UDM: crc: 0x%"PRIXX16"\n", crc16_in_packet.u16);

   if (crc16_calculated != crc16_in_packet.u16)
   {
	  if(uart_status_r() == 0)
        printf("UDM: Non-correct crc, packet: 0x%"PRIXX16", calculated: 0x%"PRIXX16"\n", crc16_in_packet.u16, crc16_calculated);
      return;
   }

   /* Выполянем короткие команды */
   if (command == UDUP_V5_COMMAND_TYPE_REBOOT)
   {
	  if(uart_status_r() == 0)
        printf("UDM: reboot\n");
      watchdog_reboot();
   }

   if (command == UDUP_V5_COMMAND_TYPE_BOOTLOADER_ACTIVATE)
   {
	  if(uart_status_r() == 0)
        printf("UDM: bootloader activate\n");
      ti_lib_flash_sector_erase(0x0001F000);
   }

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v5_time_command_process()
{
	//printf("udup_v5_time_command_process\n");
   u8_u16_t crc16_in_packet;

   if (udup_v5_data_iterator != 18)
   {
	  if(uart_status_r() == 0)
        printf("UDM: Non-correct time set command length(iter<18b): %"PRId16"\n", udup_v5_data_iterator);
      return;
   }
   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, 17);

   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[18]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[17];

   if (crc16_calculated != crc16_in_packet.u16)
   {
	  if(uart_status_r() == 0)
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

   if(uart_status_r() == 0)
     printf("UDM: epoch time: %"PRIu32"(%"PRIXX8"%"PRIXX8"%"PRIXX8"%"PRIXX8")\n", epoch.u32, epoch.u8[0], epoch.u8[1], epoch.u8[2], epoch.u8[3]);

   return;
}

/*---------------------------------------------------------------------------*/
void udup_v5_packet_net_packet_process()
{
	//printf("udup_v5_packet_net_packet_process\n");
   u8_u16_t crc16_in_packet;
   u8_u16_t payload_length;

   if (udup_v5_data_iterator < 15)
   {
	  if(uart_status_r() == 0)
        printf("UDM: Non-correct packet send command length(iter<15b): %"PRId16"\n", udup_v5_data_iterator);
      return;
   }
   payload_length.u8[0] = udup_v5_rc_uart_rx_buffer[12]; //Меняем порядок байт на MSB-First
   payload_length.u8[1] = udup_v5_rc_uart_rx_buffer[11];

   uint16_t all_packet_length_calculated = payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + UDUP_V5_CRC_LENGTH;
   uint16_t all_packet_length_really = udup_v5_data_iterator + 1; //Сравниваем счетчик(от нуля) с количеством(от единицы)

   if (all_packet_length_really != all_packet_length_calculated)
   {
	  if(uart_status_r() == 0)
	  {
        printf("UDM: Non-correct packet send command length\n");
        printf("UDM: Hex payload length([0], [1]): 0x%"PRIXX8" 0x%"PRIXX8", payload length: %"PRIu16"\n", payload_length.u8[0], payload_length.u8[1], payload_length.u16);
        printf("UDM: packet length calc(payload+13+2): %"PRId16", really: %"PRId16"\n", all_packet_length_calculated, all_packet_length_really);
	  }
      return;
   }

   uint16_t crc16_calculated = crc16_arc(udup_v5_rc_uart_rx_buffer, all_packet_length_calculated - UDUP_V5_CRC_LENGTH);

   crc16_in_packet.u8[0] = udup_v5_rc_uart_rx_buffer[payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + 1]; //Меняем порядок байт на MSB-First
   crc16_in_packet.u8[1] = udup_v5_rc_uart_rx_buffer[payload_length.u16 + UDUP_V5_RC_PAYLOAD_OFFSET + 0];

   if (crc16_calculated != crc16_in_packet.u16)
   {
	  if(uart_status_r() == 0)
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

   //printf("UDM: adress: ");
   //uip_debug_ipaddr_print(&node_ipaddr);
   //printf(", payload length: %"PRIXX16"", payload_length.u16);
   //printf(", data_iter: %"PRId16"", udup_v5_data_iterator);
   //printf("\n");
}

/*---------------------------------------------------------------------------*/
int uart_data_receiver_udup_v5(unsigned char uart_char)
{
	//printf("uart_data_receiver_udup_v5\n");
   led_blink(LED_A);
   //printf("UART char: %"PRIXX8"\n", uart_char);
   if (udup_v5_data_iterator < UDUP_V5_RC_MAX_LENGTH)
   {
      udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator] = uart_char;
      udup_v5_data_iterator++;
      timer_restart(&udup_v5_timeout_timer);
   }
   //if (uart_char == LF_BYTE)
   //{
   //   timer_set(&udup_v5_timeout_timer, 0);
   //   printf("Timer reset!");
   //}
   return 1;
}

/*---------------------------------------------------------------------------*/
hex_to_bin_errno_t convert_hex_to_bin()
{
	//printf("convert_hex_to_bin\n");
   for (uint16_t i = 0; i < udup_v5_data_iterator + 1; i = i + 2)
   {
      if (udup_v5_rc_uart_rx_buffer[i] == LF_BYTE && i > 0)
         return HEX2BIN_SUCCESS;

      else if (udup_v5_rc_uart_rx_buffer[i+1] == LF_BYTE && i+1 > 1)
         return HEX2BIN_NONPARITY;

      uint8_t convert_result;
      char hex_string[2] = { udup_v5_rc_uart_rx_buffer[i], udup_v5_rc_uart_rx_buffer[i+1] };
      str2int_errno_t convert_status = hex_str2uint8(&convert_result, hex_string);
      if (convert_status == STR2INT_SUCCESS)
      {
         //printf("HEX2BIN: converted %"CHAR"%"CHAR": dec: %"PRId8", hex: %"PRIXX8"\n", udup_v5_rc_uart_rx_buffer[i], udup_v5_rc_uart_rx_buffer[i+1], convert_result, convert_result);
         udup_v5_rc_uart_rx_buffer[i/2] = convert_result;
      }
      else
         break;
   }
   return HEX2BIN_INCONVERTIBLE;
}

/*---------------------------------------------------------------------------*/
void udup_v5_data_process()
{
	//printf("udup_v5_data_process\n");
/*
   printf("UDM: UDUP packet: ");
   for (uint16_t i = 0; i <= udup_v5_data_iterator; i++)
      printf("%"PRIXX8" ", udup_v5_rc_uart_rx_buffer[i]);
   printf("\n");
*/
   /* Проверяем символ начала пакета */
   if (udup_v5_data_iterator + 1 < 5)
   {
	  if(uart_status_r() == 0)
        printf("UDM: Non-correct packet length(length<5b): %"PRId16"\n", udup_v5_data_iterator + 1);
      return;
   }

   uint8_t mq = udup_v5_rc_uart_rx_buffer[0];
   uint8_t version = udup_v5_rc_uart_rx_buffer[1];
   uint8_t command = udup_v5_rc_uart_rx_buffer[2];

   /* Проверяем символ начала пакета */
   if (mq != UDUP_V5_MAGIC_BYTE)
   {
	  if(uart_status_r() == 0)
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
               command == UDUP_V5_COMMAND_TYPE_BOOTLOADER_ACTIVATE)
         udup_v5_short_command_process();

      /* Считаем CRC и выполняем команду времени */
      else if (command == UDUP_V5_COMMAND_TYPE_ROOT_TIME_SET)
         udup_v5_time_command_process();

      else
		 if(uart_status_r() == 0)
           printf("UDM: Non-correct command: 0x%"PRIXX8"\n", command);
   }
   else
   {
	  if(uart_status_r() == 0)
        printf("UDM: Non-correct protocol version: 0x%"PRIXX8"\n", version);
      return;
   }

   return;
}

/*---------------------------------------------------------------------------*/
void set_uart_r(void)
{
	uart = 1;
	//udup_v5_data_iterator = 1;
}
/*---------------------------------------------------------------------------*/
void unset_uart_r(void)
{
	uart = 0;
	//udup_v5_data_iterator = 0;
}
/*---------------------------------------------------------------------------*/
uint8_t uart_status_r(void)
{
	if(uart == 1)
		return 1;
	else
		return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t iterator_to_byte(uint8_t iterator)
{
	if(iterator <= 16)
		return 16;
	if((iterator > 16) && (iterator <= 32))
		return 32;
	if((iterator > 32) && (iterator <= 48))
		return 48;
	if((iterator > 48) && (iterator <= 64))
		return 64;
	if((iterator > 64) && (iterator <= 80))
		return 80;
	if((iterator > 80) && (iterator <= 96))
		return 96;
	if((iterator > 96) && (iterator <= 112))
		return 112;
	if((iterator > 112) && (iterator <= 128))
		return 128;
	return 0;
}
/*---------------------------------------------------------------------------*/
static void uart_to_air()
{
	//printf("uart_to_air\n");
	//if (dest_addr == NULL)
		//return;


	//fd00:0000:0000:0000:0212:4b00:0c46:7a01
	//uip_ipaddr_t addr = { 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4b, 0x00, 0x0c, 0x46, 0x7a, 0x01 };
	
	uip_ipaddr_t addr = find_addr((uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
											 (udup_v5_rc_uart_rx_buffer[1] << 16) |
											 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
											  udup_v5_rc_uart_rx_buffer[3]));
											  
	uip_ip6addr_t addr_not_found;
	uip_ip6addr(&addr_not_found, 0, 0, 0, 0, 0, 0, 0, 0);
	
	if((((&addr)->u16[0])  == 0x00)  &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00))	
	{	
		return; //Нету такого адреса
	}
	
	uint16_t nonce = get_nonce((uint32_t) ( (udup_v5_rc_uart_rx_buffer[0] << 24) |
											(udup_v5_rc_uart_rx_buffer[1] << 16) |
											(udup_v5_rc_uart_rx_buffer[2] << 8)  |
											(udup_v5_rc_uart_rx_buffer[3] )));
	
	nonce_key[0] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[1] = (uint8_t)(nonce & 0xFF);
	nonce_key[2] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[3] = (uint8_t)(nonce & 0xFF);
	nonce_key[4] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[5] = (uint8_t)(nonce & 0xFF);
	nonce_key[6] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[7] = (uint8_t)(nonce & 0xFF);
	nonce_key[8] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[9] = (uint8_t)(nonce & 0xFF);
	nonce_key[10] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[11] = (uint8_t)(nonce & 0xFF);
	nonce_key[12] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[13] = (uint8_t)(nonce & 0xFF);
	nonce_key[14] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[15] = (uint8_t)(nonce & 0xFF);
	
	
	//if(addr == addr_not_found)
		//return;
											  
	//printf("\nSerial: %lu\n", (uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
	//								     (udup_v5_rc_uart_rx_buffer[1] << 16) |
	//									 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
	//									  udup_v5_rc_uart_rx_buffer[3]));
	//uip_debug_ipaddr_print(&addr);
	//printf("\n");
	//printf("route_table_ptr: %i\n", route_table_ptr);

	uint8_t payload_length = iterator_to_byte(udup_v5_data_iterator + 3);
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_root.u8[0];
	udp_buffer[2] = UART_FROM_AIR_TO_TX;//packet_counter_root.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_root.u8[1];//UART_FROM_AIR_TO_TX;
	udp_buffer[8] = udup_v5_data_iterator;//Длина пакета
	
	aes_buffer[0] = packet_counter_root.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	aes_buffer[1] = packet_counter_root.u8[1];//UART_FROM_AIR_TO_TX;
	aes_buffer[2] = udup_v5_data_iterator;//Длина пакета
	
	for(uint8_t i = 3; i < payload_length; i++)
	{
		if(i < (udup_v5_data_iterator + 3))
			aes_buffer[i] = udup_v5_rc_uart_rx_buffer[i-3];
		else
			aes_buffer[i] = 0x00;
	}
	
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[UDBP_V5_HEADER_LENGTH]), payload_length);

	
	//for (uint16_t i = 0; i < payload_length + UDBP_V5_HEADER_LENGTH; i++) 
	//{
	//	cc26xx_uart_write_byte(udp_buffer[i]);
	//}
	
	
	//for(uint8_t i = 0; i < udup_v5_data_iterator; i++) /*Копирование из буфера приема UART*/
	//	udp_buffer[i+8] = udup_v5_rc_uart_rx_buffer[i];
	
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_root_process, ev, data)
{
   PROCESS_BEGIN();
   timer_set(&udup_v5_timeout_timer, 10);

   static struct etimer main_root_process_timer;
   packet_counter_root.u16 = 0;
   PROCESS_PAUSE();
   
   uint16_t crc_uart;

   while (1)
   {
      etimer_set(&main_root_process_timer, 1);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&main_root_process_timer));

      if (timer_expired(&udup_v5_timeout_timer) && udup_v5_data_iterator > 0)
      {
		 if(uart == 1)
		 {
			 if(udup_v5_data_iterator > 6)
			 {
				crc_uart = crc16_modbus(udup_v5_rc_uart_rx_buffer, udup_v5_data_iterator-2);
				if(crc_uart == (uint16_t)((udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator-1] << 8) | 
										   udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator-2]))
				{
					wait_response_slave = 1;
					ctimer_set(&wait_response, WAIT_RESPONSE * CLOCK_SECOND, wait_response_reset, NULL);
					uart_to_air();
				}
			 }
			 
			 //udup_v5_data_iterator--;
			 //printf("udup_v5_data_iterator: %i\n", udup_v5_data_iterator);
			 //uart_to_air();
			 //udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator] = uart_char;
			 
			 udup_v5_data_iterator = 0;
		 }
		 else
		 {
			 disable_interrupts();
			 udup_v5_data_iterator--;
			 /*
			 printf("UDM: HEX bytes(%"PRIu8"):", udup_v5_data_iterator);
			 for (uint16_t i = 0; i < udup_v5_data_iterator; i++)
				printf("%"PRIXX8" ", udup_v5_rc_uart_rx_buffer[i]);
			 printf("\n");
			 */
			 hex_to_bin_errno_t convert_status = convert_hex_to_bin();
			 if (convert_status == HEX2BIN_SUCCESS)
			 {
				udup_v5_data_iterator = udup_v5_data_iterator/2;
				udup_v5_data_iterator--;
				udup_v5_data_process();
			 }
			 else if (convert_status == HEX2BIN_NONPARITY)
			 {
				if(uart_status_r() == 0)
				  printf("UDM: ANSCII-HEX converting failed, NONPARITY\n");
			 }
			 else if (convert_status == HEX2BIN_INCONVERTIBLE)
			 {
				if(uart_status_r() == 0)
				  printf("UDM: ANSCII-HEX converting failed, INCONVERTIBLE\n");
			 }

			 udup_v5_data_iterator = 0;
			 enable_interrupts();
		 }
      }

   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/
