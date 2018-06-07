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
#include "net/ipv6/uip-ds6-route.h"
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
#include "int-flash-common.h"

#include "dev/serial-line.h"
#include "../cpu/cc26xx-cc13xx/dev/cc26xx-uart.h"
#include "uart/root.h"

/*---------------------------------------------------------------------------*/

#define UART_DATA_POLL_INTERVAL 5	//in main timer ticks, one tick ~8ms

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

volatile union { uint16_t u16; uint8_t u8[2]; } packet_counter_root;

static eeprom_t eeprom_root;

PROCESS(settings_root_init, "Initializing settings");
PROCESS(main_root_process, "main root process");

/*---------------------------------------------------------------------------*/
/*PROTOTYPES OF FUNCTIONS*/
static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version);
static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version);
static void ack_packet_sender(const uip_ip6addr_t *dest_addr);
static void uart_to_air();
static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version);
static void udup_v5_dag_root_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version);

static uint8_t iterator_to_byte(uint8_t iterator);
static void wait_response_reset(void *ptr);


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

	uint8_t protocol_version = data[0];

	if (protocol_version == UDBP_PROTOCOL_VERSION_V5)
	{
		uint8_t v_5_module_id = data[UDUP_V5_MODULE_ID];
		uint8_t v_5_packet_type = data[UDUP_V5_PACKET_TYPE];

		if (v_5_module_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
		{
			if (v_5_packet_type == DATA_TYPE_JOIN_V5_STAGE_1)
			{
				join_stage_2_sender(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
				led_off(LED_A);
				return;
			}
			else if(v_5_packet_type == DATA_TYPE_JOIN_V5_STAGE_3)
			{
				join_stage_4_sender(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
				led_off(LED_A);
				return;
			}
			else if(v_5_packet_type == UART_FROM_RX_TO_AIR)
			{
				uart_from_air(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
				led_off(LED_A);
				return;
			}
		}
		ack_packet_sender(&node_addr);
		udup_v5_dag_root_print(&node_addr, data, datalen, UDBP_PROTOCOL_VERSION_V5);
	}

	led_off(LED_A);
}

/*---------------------------------------------------------------------------*/

static void join_stage_2_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	if (dest_addr == NULL)
		return;
   
	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	uint8_t payload_length = 18; //2 HEADER + 16 AES
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_2;
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0];
	udp_buffer[7] = packet_counter_root.u8[1];
	
	uint16_t nonce = random_rand();
		
	add_route((uint32_t)((data[UDBP_V5_HEADER_LENGTH + 8] << 24) |
						 (data[UDBP_V5_HEADER_LENGTH + 9] << 16) |
						 (data[UDBP_V5_HEADER_LENGTH + 10] << 8) |
						 (data[UDBP_V5_HEADER_LENGTH + 11] )), 
						  &addr,
						  nonce);
						 
	aes_buffer[0] = (uint8_t)((nonce >> 8) & 0xFF);
	aes_buffer[1] = (uint8_t)(nonce & 0xFF);
	
	for(uint8_t i = 2; i < 16; i++)
		aes_buffer[i] = 0x00;
	
	aes_ecb_encrypt((uint32_t*)aes_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[8]));

	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/

static void join_stage_4_sender(const uip_ip6addr_t *dest_addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	uint16_t nonce = get_nonce( (uint32_t) ((data[UDBP_V5_HEADER_LENGTH + 2] << 24) |
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
	
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[12], (uint32_t*)(aes_buffer), 16);
								
	if(aes_buffer[0] == nonce_key[0] && aes_buffer[1] == nonce_key[1])
	{		
		unlock_addr((uint32_t) ((data[UDBP_V5_HEADER_LENGTH + 2] << 24) |
								(data[UDBP_V5_HEADER_LENGTH + 3] << 16) |
								(data[UDBP_V5_HEADER_LENGTH + 4] << 8)  |
								(data[UDBP_V5_HEADER_LENGTH + 5] )));				
	}
					 
	uint8_t payload_length = 18; //2 HEADER + 16 AES
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID; 
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_4; 
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0]; 
	udp_buffer[7] = packet_counter_root.u8[1]; 
	
	for(uint8_t i = 0; i < 16; i++)
		aes_buffer[i] = 0x00;
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[8]), 16);
	
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/

static void ack_packet_sender(const uip_ip6addr_t *dest_addr)
{
	if (dest_addr == NULL)
		return;

	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);

	uint8_t payload_length = 16;
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[2] = DATA_TYPE_ACK;
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_root.u8[0];
	udp_buffer[7] = packet_counter_root.u8[1];
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
static void uart_to_air() 
{
	uip_ipaddr_t addr;
	uint16_t nonce;
	
	if(udup_v5_data_iterator < 7) //Если команда меньше 7 байт, то она для однобайтно адресуемых счетчиков
	{
		addr = find_addr((uint32_t)(udup_v5_rc_uart_rx_buffer[0]));
		
		if((((&addr)->u16[0])  == 0x00) && 
		(((&addr)->u16[1])  == 0x00) && 
		(((&addr)->u16[2])  == 0x00) && 
		(((&addr)->u16[3])  == 0x00) &&
		(((&addr)->u16[4])  == 0x00) &&
		(((&addr)->u16[5])  == 0x00) && 
		(((&addr)->u16[6])  == 0x00) && 
		(((&addr)->u16[7])  == 0x00))
		{
			return;
		}
		
		nonce = get_nonce((uint32_t)(udup_v5_rc_uart_rx_buffer[0]));
	}
	else //В первую очередь проверяем по четырехбайтному адресу, а потом по однобайтному
	{
		addr = find_addr((uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
												 (udup_v5_rc_uart_rx_buffer[1] << 16) |
												 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
												  udup_v5_rc_uart_rx_buffer[3]));
		if((((&addr)->u16[0])  == 0x00) && 
		(((&addr)->u16[1])  == 0x00) && 
		(((&addr)->u16[2])  == 0x00) && 
		(((&addr)->u16[3])  == 0x00) &&
		(((&addr)->u16[4])  == 0x00) &&
		(((&addr)->u16[5])  == 0x00) && 
		(((&addr)->u16[6])  == 0x00) && 
		(((&addr)->u16[7])  == 0x00))	
		{	
					addr = find_addr((uint32_t)(udup_v5_rc_uart_rx_buffer[0]));
		
					if((((&addr)->u16[0])  == 0x00) && 
					(((&addr)->u16[1])  == 0x00) && 
					(((&addr)->u16[2])  == 0x00) && 
					(((&addr)->u16[3])  == 0x00) &&
					(((&addr)->u16[4])  == 0x00) &&
					(((&addr)->u16[5])  == 0x00) && 
					(((&addr)->u16[6])  == 0x00) && 
					(((&addr)->u16[7])  == 0x00))
					{
						return;
					}
					
					nonce = get_nonce((uint32_t)(udup_v5_rc_uart_rx_buffer[0]));
		}
		else
		{
			nonce = get_nonce((uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
										 (udup_v5_rc_uart_rx_buffer[1] << 16) |
										 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
										 (udup_v5_rc_uart_rx_buffer[3] )));
		}
	}
	
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

	uint8_t payload_length = iterator_to_byte(udup_v5_data_iterator + 3);
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[2] = UART_FROM_AIR_TO_TX;
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();
	
	aes_buffer[0] = packet_counter_root.u8[0];
	aes_buffer[1] = packet_counter_root.u8[1];
	aes_buffer[2] = udup_v5_data_iterator; //Длина пакета
	
	for(uint8_t i = 3; i < payload_length; i++)
	{
		if(i < (udup_v5_data_iterator + 3))
			aes_buffer[i] = udup_v5_rc_uart_rx_buffer[i-3];
		else
			aes_buffer[i] = 0x00;
	}
	
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[UDBP_V5_HEADER_LENGTH]), payload_length);
	
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}

/*---------------------------------------------------------------------------*/

static void uart_from_air(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
	if(wait_response_slave == 0)
		return;
	
	//Дешифрово4kа
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[UDBP_V5_HEADER_LENGTH], (uint32_t*)(aes_buffer), (length - UDBP_V5_HEADER_LENGTH));
	
	uint32_t serial = ( (aes_buffer[3] << 24) |
						(aes_buffer[4] << 16) |
						(aes_buffer[5] << 8)  |
						(aes_buffer[6] ));
	uint16_t counter = ((aes_buffer[1] << 8)  |
						 aes_buffer[0]);
	
	if(valid_counter(serial, counter) || valid_counter(aes_buffer[3], counter))//|| valid_counter(aes_buffer[3], counter)
	{
		for(uint16_t i = 0; i < aes_buffer[2]; i++)
			cc26xx_uart_write_byte(aes_buffer[i + 3]);
	}
}

/*---------------------------------------------------------------------------*/

static void udup_v5_dag_root_print(const uip_ip6addr_t *addr, const uint8_t *data, const uint16_t length, uint8_t version)
{
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
	for (uint16_t i = 0; i < payload_length.u16; i++) 
	{
		udup_v5_cr_uart_tx_buffer[i + UDUP_V5_CR_PAYLOAD_OFFSET] = data[i + payload_offset];
		//printf("%" PRIXX8 " ", data[i + payload_offset]);
	}

	/* Считаем контрольную сумму */
	crc16_calculated.u16 = crc16_arc(udup_v5_cr_uart_tx_buffer, UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16);
	udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+0] = crc16_calculated.u8[1]; //Меняем порядок байт на MSB-First
	udup_v5_cr_uart_tx_buffer[UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16+1] = crc16_calculated.u8[0];

	/* Выводим весь пакет в UART в зависимости от настроек */
	if(uart_status_r() == 0)
	{
		for (uint16_t i = 0; i < UDUP_V5_CR_PAYLOAD_OFFSET + payload_length.u16 + UDUP_V5_CRC_LENGTH; i++)
			printf("%"PRIXX8, udup_v5_cr_uart_tx_buffer[i]);
		printf("\n");
	}

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
	cc26xx_uart_set_input(&uart_data_receiver);

	/* blink-blink LED */
	led_blink(LED_A);
	led_blink(LED_A);

	/* start main root process */
	process_start(&main_root_process, NULL);
}

/*---------------------------------------------------------------------------*/

int uart_data_receiver(unsigned char uart_char)
{
	led_blink(LED_A);
	if(udup_v5_data_iterator < UDUP_V5_RC_MAX_LENGTH)
	{
		udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator] = uart_char;
		udup_v5_data_iterator++;
		timer_restart(&udup_v5_timeout_timer);
	}
	return 1;
}

/*---------------------------------------------------------------------------*/
void set_uart_r(void)
{
	uart = 1;
}
/*---------------------------------------------------------------------------*/
void unset_uart_r(void)
{
	uart = 0;
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

static void wait_response_reset(void *ptr)
{
	wait_response_slave = 0;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(settings_root_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	read_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
	
	if(eeprom_root.aes_key_configured == true) //При первом включении забивает нормальные настройки сети
	{
		if((eeprom_root.channel != 26) && (eeprom_root.panid != 0xAABB))
		{
			eeprom_root.channel = 26;
			eeprom_root.panid = 0xAABB;
			write_eeprom((uint8_t*)&eeprom_root, sizeof(eeprom_root));
		}
	}
	
	if(!eeprom_root.aes_key_configured) 
	{
		// printf("AES-128 key:");
		// for (uint8_t i = 0; i < 16; i++)
		// {
			// aes_key[i] = eeprom_root.aes_key[i];
			// printf(" %"PRIXX8, aes_key[i]);
		// }
		// printf("\n");
		;
	}
	else
	{
		printf("AES-128 key not declared\n******************************\n******PLEASE SET AES KEY******\n******************************\n");
		// led_mode_set(LED_FAST_BLINK);
		while(eeprom_root.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	// if(!eeprom_root.interface_configured) 
	// {
		// interface = eeprom_root.interface;
	
		// if(interface == INTERFACE_RS485)
			// printf("Installed interface RS485\n");
		// else if(interface == INTERFACE_CAN)
			// printf("Installed interface CAN\n");
		// else
			// printf("Unknown interface\n");
	// }
	// else
	// {
		// printf("Interface not declared\n******************************\n*****PLEASE SET INTERFACE*****\n******************************\n");
		// led_mode_set(LED_FAST_BLINK);
		
		// while(eeprom_root.interface_configured)
		// {
			// PROCESS_YIELD();
		// }	
	// }
	
	radio_value_t channel = 0;
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	if(channel != eeprom_root.channel)
	{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_root.channel);
		
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_root.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_root.channel, freq_mhz);
		}

		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_root.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_root.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		if(panid != eeprom_root.panid)
		{
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_root.panid);
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_root.panid);
		}
	}
	
	process_post(&rpl_root_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
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
				if(udup_v5_data_iterator > 3)
				{
					crc_uart = crc16_modbus(udup_v5_rc_uart_rx_buffer, udup_v5_data_iterator-2);
					if(crc_uart == (uint16_t) ((udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator-1] << 8) | 
												udup_v5_rc_uart_rx_buffer[udup_v5_data_iterator-2]))
					{
						wait_response_slave = 1;
						ctimer_set(&wait_response, WAIT_RESPONSE * CLOCK_SECOND, wait_response_reset, NULL);
						uart_to_air();
					}
				}
				
				udup_v5_data_iterator = 0;
			}
			else
			{
				// disable_interrupts();
				// udup_v5_data_iterator--;

				udup_v5_data_iterator = 0;
				// enable_interrupts();
			}
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
