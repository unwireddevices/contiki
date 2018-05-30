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
 *         DAG-node service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 */
/*---------------------------------------------------------------------------*/
#define DPRINT printf(">dag_node.c:%"PRIu16"\n", __LINE__);watchdog_periodic();

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "clock.h"

#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ip/uip-debug.h"
#include "net/ip/simple-udp.h"
#include "net/link-stats.h"

#include "dev/leds.h"
#include "sys/clock.h"
#include "shell.h"
#include "serial-shell.h"
#include "dev/serial-line.h" //
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "radio_power.h"
#include "dev/watchdog.h"
#include "dev/cc26xx-uart.h"
#include "board-peripherals.h"
#include "board.h"
#include "ti-lib.h"

#include "ota-main.h"
#include "ota-common.h"
#include "crypto-common.h"
#include "rtc-common.h"
#include "system-common.h"
#include "int-flash-common.h"

#include "xxf_types_helper.h"
#include "ud_binary_protocol.h"

#include "dag_node.h"

#ifdef IF_UD_BUTTON
#  include "smarthome/button.h"
#endif

#ifdef IF_UD_RELAY
#  include "smarthome/relay.h"
#endif

#ifdef IF_UD_DIMMER
#  include "smarthome/dimmer.h"
#endif

#ifdef IF_UD_MOTIONSENSOR
#  include "smarthome/motionsensor.h"
#endif


#define MAINTENANCE_INTERVAL            (10 * 60 * CLOCK_SECOND)
#define SHORT_STATUS_INTERVAL           (10 * 60 * CLOCK_SECOND)
#define LONG_STATUS_INTERVAL            (20 * 60 * CLOCK_SECOND)
#define ROOT_FIND_INTERVAL                    (2 * CLOCK_SECOND)
#define ROOT_FIND_LIMIT_TIME             (2 * 60 * CLOCK_SECOND)
#define FW_DELAY                              (2 * CLOCK_SECOND)
#define FW_MAX_ERROR_COUNTER                    5

#define FALSE                                   0x00
#define TRUE                                    0x01

#define MAX_NON_ANSWERED_PINGS	3

#define WAIT_RESPONSE			0.150 	//Максимальное время ожидания ответа от счетчика в секундах


#define AES128_PACKAGE_LENGTH	16	//Длина пакета AES-128

#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
   UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/

/* struct for simple_udp_send */
simple_udp_connection_t udp_connection;

volatile uint8_t spi_status;

volatile uint8_t led_mode;

volatile uint8_t non_answered_packet = 0;
volatile uip_ipaddr_t root_addr;
static struct interpocess_message message_for_main_process;

static struct etimer maintenance_timer;
static struct etimer fw_timer;

volatile u8_u16_t fw_chunk_quantity;
volatile uint16_t fw_ext_flash_address = 0;
volatile uint8_t fw_error_counter = 0;

uint32_t ota_image_current_offset = 0;

extern uint32_t serial;

static struct ctimer wait_response;
static bool wait_response_slave = 0;

rpl_dag_t *rpl_probing_dag;

volatile uint8_t process_message = 0;

static volatile union { uint16_t u16; uint8_t u8[2]; } packet_counter_root;

static uint8_t aes_buffer[128];
static uint8_t aes_key[16];
static uint8_t nonce_key[16];

//static struct eeprom eeprom_settings;
static eeprom_t eeprom_dag;

static uint8_t interface; 
/*---------------------------------------------------------------------------*/

PROCESS(settings_dag_init, "Initializing settings");
PROCESS(dag_node_process, "DAG-node process");
PROCESS(dag_node_button_process, "DAG-node button process");
PROCESS(root_find_process, "Root find process");
PROCESS(status_send_process, "Status send process");
PROCESS(maintenance_process, "Maintenance process");
PROCESS(led_process, "Led process");
PROCESS(fw_update_process, "FW OTA update process");

/*---------------------------------------------------------------------------*/
uint8_t get_interface(void)
{
	return interface;
}
/*---------------------------------------------------------------------------*/
void interface_update(uint8_t interface_new)
{
	eeprom_dag.interface_configured = false;
	eeprom_dag.interface = interface_new;
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
void aes128_key_update(const uint8_t *aes_key_new)
{	
	eeprom_dag.aes_key_configured = false;
	
	for(uint8_t i = 0; i < 16; i++)
		eeprom_dag.aes_key[i] = aes_key_new[i];
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
uint8_t *get_aes128_key(void)
{
	return aes_key;
}
/*---------------------------------------------------------------------------*/
void serial_update(uint32_t serial_new)
{
	eeprom_dag.serial_configured = false;
	eeprom_dag.serial = serial_new;
	
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
	
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
uint32_t get_serial(void)
{
	return serial;
}
/*---------------------------------------------------------------------------*/
void channel_update(uint8_t channel_new)
{
	eeprom_dag.channel = channel_new;
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
}
/*---------------------------------------------------------------------------*/
void panid_update(uint16_t panid_new)
{
	eeprom_dag.panid = panid_new;
	write_eeprom(((uint8_t*)&eeprom_dag), sizeof(eeprom_dag));
}
/*---------------------------------------------------------------------------*/

static void wait_response_reset(void *ptr)
{
   wait_response_slave = 0;
}

/*---------------------------------------------------------------------------*/

bool wait_response_status(void)
{
   return wait_response_slave;
}

/*---------------------------------------------------------------------------*/

static void udbp_v5_uart_from_air_to_tx_handler(const uip_ipaddr_t *sender_addr,
												const uint8_t *data,
												uint16_t datalen)
{
	//Дешифрово4kа
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[UDBP_V5_HEADER_LENGTH], (uint32_t*)(aes_buffer), (datalen - UDBP_V5_HEADER_LENGTH));
	
	
	if(packet_counter_root.u16 < ((uint16_t)(aes_buffer[1] << 8) | aes_buffer[0]))
	{		
		packet_counter_root.u8[0] = aes_buffer[0];
		packet_counter_root.u8[1] = aes_buffer[1];
		
		if(get_interface() == INTERFACE_RS485)
		{
			ti_lib_gpio_set_dio(RS485_DE);
			ti_lib_gpio_set_dio(RS485_RE);
		}
		
		
		ti_lib_uart_int_disable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		
		for(uint16_t i = 0; i < aes_buffer[2]; i++)
			cc26xx_uart_write_byte(aes_buffer[i + 3]);
		
		while(ti_lib_uart_busy(UART0_BASE));
		
		if(get_interface() == INTERFACE_RS485)
		{
			ti_lib_gpio_clear_dio(RS485_DE);
			ti_lib_gpio_clear_dio(RS485_RE);
		}
		
		//ti_lib_uart_fifo_disable(UART0_BASE);
		//ti_lib_uart_fifo_enable(UART0_BASE);
		while(ti_lib_uart_chars_avail(UART0_BASE))
		{
			UARTCharGetNonBlocking(UART0_BASE);
		}
		ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		reset_uart();
		wait_response_slave = 1;
		ctimer_set(&wait_response, (WAIT_RESPONSE * CLOCK_SECOND), wait_response_reset, NULL);
		
		
		//ti_lib_gpio_clear_dio(RS485_DE);
		//ti_lib_gpio_clear_dio(RS485_RE);
	}
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
void udbp_v5_uart_to_root_sender(char* data)
{
	if (node_mode == 2) //MODE_NOTROOT_SLEEP
	{
		watchdog_reboot();
	}

	if (node_mode == MODE_NORMAL)
	{
		uint16_t crc_uart;
		crc_uart = crc16_modbus((uint8_t*)&data[1], (data[0] - 2));
		
		if(data[0] > 6)
		{
			if(crc_uart != (uint16_t)((data[((uint8_t)(data[0]))] << 8) | data[(data[0]-1)]))
			{
				return; //CRC16 не совпала
			}
		}
		else
		{
			return; //Слишком маленькая длина фрейма 4 байта адреса и 2 CRC16
		}
		
		uip_ipaddr_t addr;
		uip_ip6addr_copy(&addr, &root_addr);

		uint8_t payload_length = iterator_to_byte(data[0] + 3);//data[0] + 2;
		uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
		udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
		udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_node.u8[0];
		udp_buffer[2] = UART_FROM_RX_TO_AIR;//packet_counter_node.u8[1];
		udp_buffer[3] = get_parent_rssi();
		udp_buffer[4] = get_temperature();
		udp_buffer[5] = get_voltage();

		aes_buffer[0] = packet_counter_node.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
		aes_buffer[1] = packet_counter_node.u8[1];//UART_FROM_RX_TO_AIR;
		aes_buffer[2] = data[0];
		
		for(uint8_t i = 3; i < payload_length; i++)
		{
		if(i < (data[0] + 3))
			aes_buffer[i] = data[i-2];
		else
			aes_buffer[i] = 0x00;
		}
	
		aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[UDBP_V5_HEADER_LENGTH]), payload_length);


		//for(uint8_t i = 1; i < (data[0] + 1); i++) /*Копирование из буфера приема UART*/
		//	udp_buffer[i+7] = data[i];
		  
		  
		//udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH - 2] = 0x12;//(uint8_t)(crc_uart >> 8);//CRC16
		//udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH - 1] = 0x34;//(uint8_t)(crc_uart & 0xFF);//CRC16
		
		//udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH - 2] = (uint8_t)((uint16_t)((data[(data[0])] << 8) | data[(data[0]-1)]) >> 8);//CRC16
		//udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH - 1] = (uint8_t)((uint16_t)((data[(data[0])] << 8) | data[(data[0]-1)]) & 0xFF);//CRC16
/*
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
 */

		net_on(RADIO_ON_TIMER_OFF);
		simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
		packet_counter_node.u16++;
		led_mode_set(LED_FLASH);
   }
}
/*---------------------------------------------------------------------------*/

void udbp_v5_join_stage_1_sender(const uip_ipaddr_t *dest_addr)
{
	//printf("udbp_v5_join_stage_1_sender\n");
	if (dest_addr == NULL)
		return;

	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	if(uart_status() == 0)
	{
		printf("DAG Node: Send join packet to DAG-root node: ");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}

	uint8_t payload_length = 8 + 4; //4 serial
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_node.u8[0];
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_1;//packet_counter_node.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_node.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_node.u8[1];//DATA_TYPE_JOIN_V5_STAGE_1;
	udp_buffer[8] = CURRENT_DEVICE_GROUP;
	udp_buffer[9] = CURRENT_DEVICE_SLEEP_TYPE;
	udp_buffer[10] = CURRENT_ABILITY_1BYTE;
	udp_buffer[11] = CURRENT_ABILITY_2BYTE;
	udp_buffer[12] = CURRENT_ABILITY_3BYTE;
	udp_buffer[13] = CURRENT_ABILITY_4BYTE;
	
	udp_buffer[14] = (uint8_t)((serial >> 24) & 0xFF);	// SERIAL 00  00  92  F6
	udp_buffer[15] = (uint8_t)((serial >> 16) & 0xFF); 	// SERIAL
	udp_buffer[16] = (uint8_t)((serial >> 8) & 0xFF);  	// SERIAL
	udp_buffer[17] = (uint8_t)(serial & 0xFF);			// SERIAL
/*
	udp_buffer[18] = DATA_RESERVED;
	udp_buffer[19] = DATA_RESERVED;
	udp_buffer[20] = DATA_RESERVED;
	udp_buffer[21] = DATA_RESERVED;
 */

	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
}
/*---------------------------------------------------------------------------*/
//udbp_v5_join_stage_2_handler
static void udbp_v5_join_stage_3_sender(const uip_ipaddr_t *dest_addr,
										const uint8_t *data,
										uint16_t datalen)
{
	//printf("udbp_v5_join_stage_3_sender\n");
	if (dest_addr == NULL)
		return;
	
	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, dest_addr);
	
	if(uart_status() == 0)
	{
		printf("DAG Node: Send join packet stage 3 to DAG-root node:");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}
	
	uint8_t payload_length = 2 + 4 + 16; //2 header + 4 serial + 16 AES
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_node.u8[0];
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_3;//packet_counter_node.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_node.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_node.u8[1];//DATA_TYPE_JOIN_V5_STAGE_3;
	
	udp_buffer[8] = (uint8_t)((serial >> 24) & 0xFF);	// SERIAL 00  00  92  F6
	udp_buffer[9] = (uint8_t)((serial >> 16) & 0xFF); 	// SERIAL
	udp_buffer[10] = (uint8_t)((serial >> 8) & 0xFF);  	// SERIAL
	udp_buffer[11] = (uint8_t)(serial & 0xFF);			// SERIAL
	
	aes_ecb_decrypt((uint32_t*)aes_key, (uint32_t*)&data[8], (uint32_t*)aes_buffer);
	nonce_key[0] = aes_buffer[0];
	nonce_key[1] = aes_buffer[1];
	nonce_key[2] = aes_buffer[0];
	nonce_key[3] = aes_buffer[1];
	nonce_key[4] = aes_buffer[0];
	nonce_key[5] = aes_buffer[1];
	nonce_key[6] = aes_buffer[0];
	nonce_key[7] = aes_buffer[1];
	nonce_key[8] = aes_buffer[0];
	nonce_key[9] = aes_buffer[1];
	nonce_key[10] = aes_buffer[0];
	nonce_key[11] = aes_buffer[1];
	nonce_key[12] = aes_buffer[0];
	nonce_key[13] = aes_buffer[1];
	nonce_key[14] = aes_buffer[0];
	nonce_key[15] = aes_buffer[1];
	
	/*
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
		printf(" %"PRIXX8, udp_buffer[i+12]);
	}
	printf("\n");
	*/
	
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[12]), 16);
	
/*	
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
 */

	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);

	//uip_ipaddr_copy(&root_addr, sender_addr);
	//process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);
	//etimer_set(&maintenance_timer, 0);
	//packet_counter_node.u16 = 0;
}
/*---------------------------------------------------------------------------*/
static void udbp_v5_join_stage_4_handler(const uip_ipaddr_t *sender_addr,
										const uint8_t *data,
										uint16_t datalen)
{
	//printf("udbp_v5_join_stage_4_handler\n");
	
	if (sender_addr == NULL)
		return;
			
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[8], (uint32_t*)(aes_buffer), 16);
	
	/*
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, nonce_key[i]);
	}
	printf("\n");
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, data[i+8]);
	}
	printf("\n");
	
	for (uint16_t i = 0; i < 16; i++)
	{
		printf(" %"PRIXX8, aes_buffer[i]);
	}
	printf("\n");
	*/
	
	if( (aes_buffer[0] == 0x00)  &&
		(aes_buffer[1] == 0x00)  &&
		(aes_buffer[2] == 0x00)  &&
		(aes_buffer[3] == 0x00)  &&
		(aes_buffer[4] == 0x00)  &&
		(aes_buffer[5] == 0x00)  &&
		(aes_buffer[6] == 0x00)  &&
		(aes_buffer[7] == 0x00)  &&
		(aes_buffer[8] == 0x00)  &&
		(aes_buffer[9] == 0x00)  &&
		(aes_buffer[10] == 0x00) &&
		(aes_buffer[11] == 0x00) &&
		(aes_buffer[12] == 0x00) &&
		(aes_buffer[13] == 0x00) &&
		(aes_buffer[14] == 0x00) &&
		(aes_buffer[15] == 0x00))
	{
		packet_counter_root.u8[0] = data[6];
		packet_counter_root.u8[1] = data[7];
		//printf("%i\n", packet_counter_root.u16);////////////////////////////////////////
		uip_ipaddr_copy(&root_addr, sender_addr); //Авторизован
		process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);
		etimer_set(&maintenance_timer, 0);
		packet_counter_node.u16 = 0;
		return;
	}
	
	printf("Authorisation Error\n");
	//Не авторизован
	
}
/*---------------------------------------------------------------------------*/
/*
static void command_settings_handler(const uip_ipaddr_t *sender_addr,
                                    const uint8_t *data,
                                    uint16_t datalen)
{
      message_for_main_process.data_type = data[2];
      message_for_main_process.ability_target = data[3];
      message_for_main_process.ability_number = data[4];
      message_for_main_process.ability_state = data[5];
      printf("DAG Node: Command/settings packet received(target, number, state): %"PRIXX8",%"PRIXX8",%"PRIXX8"\n", message_for_main_process.ability_target, message_for_main_process.ability_number, message_for_main_process.ability_state);
      process_post(&main_process, PROCESS_EVENT_CONTINUE, &message_for_main_process);
}
*/

/*---------------------------------------------------------------------------*/

static void udbp_v5_ack_handler(const uip_ipaddr_t *sender_addr,
								const uint8_t *data,
								uint16_t datalen)
{
	non_answered_packet = 0;
	if(uart_status() == 0)
		printf("DAG Node: ACK packet received, non-answered packet counter: %"PRId8" \n", non_answered_packet);
	net_off(RADIO_OFF_NOW);
	process_message = PT_MESSAGE_ACK_RECIEVED;
	process_post_synch(&main_process, PROCESS_EVENT_MSG, (process_data_t)&process_message);
}

/*---------------------------------------------------------------------------*/

/*
static void firmware_data_handler(const uip_ipaddr_t *sender_addr,
                                    const uint8_t *data,
                                    uint16_t datalen)
{
      printf(" Firmware packet received(%"PRIu16" bytes)", datalen - FIRMWARE_PAYLOAD_OFFSET);

      uint8_t flash_write_buffer[FIRMWARE_PAYLOAD_LENGTH];

      for (uint16_t i = 0; i < FIRMWARE_PAYLOAD_LENGTH; i++)
      {
            flash_write_buffer[i] = data[i + FIRMWARE_PAYLOAD_OFFSET];
      }

      fw_error_counter = 0;
      uint32_t current_ota_ext_flash_address = (ota_images[1-1] << 12) + ota_image_current_offset;
      while(store_firmware_data(current_ota_ext_flash_address, flash_write_buffer, FIRMWARE_PAYLOAD_LENGTH));
      ota_image_current_offset = ota_image_current_offset + FIRMWARE_PAYLOAD_LENGTH;

      etimer_set( &fw_timer, 0 );
}
*/

/*---------------------------------------------------------------------------*/

/*
static void firmware_cmd_new_fw_handler(const uip_ipaddr_t *sender_addr,
                                          const uint8_t *data,
                                          uint16_t datalen)
{
      fw_chunk_quantity.u8[0] = data[5];
      fw_chunk_quantity.u8[1] = data[4];

      ota_image_current_offset = 0;

      printf("DAG Node: DATA_TYPE_FIRMWARE_COMMAND_NEW_FW command received, %"PRIu16"(0x%"PRIXX8" 0x%"PRIXX8") chunks\n", fw_chunk_quantity.u16, data[5], data[4]);

      if (spi_status == SPI_EXT_FLASH_ACTIVE)
      {
            printf("DAG Node: OTA update process start\n");
            process_start(&fw_update_process, NULL);
      }
      else
      {
            udbp_v5_message_sender(DEVICE_MESSAGE_OTA_SPI_NOTACTIVE, DATA_NONE, DATA_NONE);
            printf("DAG Node: OTA update not processed, spi flash not-active\n");
      }
}
*/

/*---------------------------------------------------------------------------*/

static void udp_receiver(struct simple_udp_connection *c,
                         const uip_ipaddr_t *sender_addr,
                         uint16_t sender_port,
                         const uip_ipaddr_t *receiver_addr,
                         uint16_t receiver_port,
                         const uint8_t *data, //TODO: make "parse" function(data[0] -> data.protocol_version)
                         uint16_t datalen)
{
      uint8_t protocol_version = data[0]; //deprecated
      uint8_t device_version = data[1]; //deprecated
      //uint8_t packet_type = data[2]; //deprecated
      //uint8_t packet_subtype = data[3]; //deprecated

	  if(uart_status() == 0)
	  {
		printf("DAG Node: UDP packet received(%"PRIu8"): ", datalen);
			for (uint16_t i = 0; i < datalen; i++)
				printf("%"PRIXX8, data[i]);
			printf("\n");
	  }

     if (protocol_version == PROTOCOL_VERSION_V1 && device_version == CURRENT_DEVICE_VERSION)
      {
/* 
            if (packet_type == DATA_TYPE_COMMAND || data[2] == DATA_TYPE_SETTINGS)
                  command_settings_handler(sender_addr, data, datalen);

            else if (packet_type == DATA_TYPE_FIRMWARE)
                  firmware_data_handler(sender_addr, data, datalen);

            else if (packet_type == DATA_TYPE_SET_TIME && packet_subtype == DATA_TYPE_SET_TIME_RESPONSE)
                  time_data_handler(data, datalen);

            else if (packet_type == DATA_TYPE_SET_TIME && packet_subtype == DATA_TYPE_SET_TIME_COMMAND_SYNC)
                  send_time_sync_req_packet(data, datalen);

            else if (packet_type == DATA_TYPE_FIRMWARE_CMD)
            {
                  if (packet_subtype == DATA_TYPE_FIRMWARE_COMMAND_NEW_FW)
                        firmware_cmd_new_fw_handler(sender_addr, data, datalen);

                  else if (packet_subtype == DATA_TYPE_FIRMWARE_COMMAND_REBOOT)
                        watchdog_reboot();

                  else if (packet_subtype == DATA_TYPE_FIRMWARE_COMMAND_CLEAN_GI)
                        erase_ota_image(0);

                  else if (packet_subtype == DATA_TYPE_FIRMWARE_COMMAND_FLASH_GI)
                  {
                     write_fw_flag(FW_FLAG_NEW_IMG_INT);
                     watchdog_reboot();
                  }

                  else
                  {
                        printf("DAG Node: Incompatible FW CMD command from ");
                        uip_debug_ipaddr_print(sender_addr);
                        printf(", command: 0x%02x\n", data[3]);
                  }
            }

            else
            {
                  printf("DAG Node: Incompatible data type UDP packer from ");
                  uip_debug_ipaddr_print(sender_addr);
                  printf(", data type: 0x%02x\n", data[2]);
            }
*/
      }
      else if (protocol_version == UDBP_PROTOCOL_VERSION_V5)
      {
         uint8_t endpoint = data[UDUP_V5_MODULE_ID];
         if (endpoint == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
         {
            uint8_t packet_type = data[UDUP_V5_PACKET_TYPE];
            if (packet_type == DATA_TYPE_JOIN_V5_STAGE_2)
            {
				udbp_v5_join_stage_3_sender(sender_addr, data, datalen);
            }
			else if (packet_type == DATA_TYPE_JOIN_V5_STAGE_4)
            {
				udbp_v5_join_stage_4_handler(sender_addr, data, datalen);
            }
            else if (packet_type == DATA_TYPE_ACK)
            {
				udbp_v5_ack_handler(sender_addr, data, datalen);
            }
			else if (packet_type == UART_FROM_AIR_TO_TX)
            {
				udbp_v5_uart_from_air_to_tx_handler(sender_addr, data, datalen);
            }
            else
            {
				if(uart_status() == 0)
					printf("DAG Node: Incompatible packet type(endpoint UNWDS_6LOWPAN_SYSTEM_MODULE_ID): %"PRIXX8"\n", packet_type);
            }
         }
         else
         {
            for (uint16_t i = 0; i < datalen-6; i++)
               message_for_main_process.payload[i] = data[i+6];
			
			if(uart_status() == 0)
			{
				printf("DAG Node: Message for module received(%"PRIu8"): ", datalen-6);
				for (uint16_t i = 0; i < datalen-6; i++)
					printf("%"PRIXX8, message_for_main_process.payload[i]);
				printf("\n");
			}
            process_post(&main_process, PROCESS_EVENT_CONTINUE, &message_for_main_process);
         }

      }
      else
      {
		if(uart_status() == 0)
			printf("DAG Node: Incompatible protocol version: %"PRIXX8"\n", protocol_version);
      }

   led_mode_set(LED_FLASH);
}

/*---------------------------------------------------------------------------*/

/*
void send_time_sync_req_packet()
{
   if (node_mode != MODE_NORMAL)
      return;

   uip_ipaddr_t addr;
   uip_ip6addr_copy(&addr, &root_addr);

   time_data_t local_time = get_epoch_time();

   uint8_t udp_buffer[PROTOCOL_VERSION_V2_16BYTE];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_SET_TIME;
   udp_buffer[3] = DATA_TYPE_SET_TIME_REQUEST;
   udp_buffer[4] = DATA_NONE;
   udp_buffer[5] = DATA_NONE;
   udp_buffer[6] = DATA_NONE;
   udp_buffer[7] = DATA_NONE;
   udp_buffer[8] = DATA_NONE;
   udp_buffer[9] = DATA_NONE;
   udp_buffer[10] = local_time.seconds_u8[0];
   udp_buffer[11] = local_time.seconds_u8[1];
   udp_buffer[12] = local_time.seconds_u8[2];
   udp_buffer[13] = local_time.seconds_u8[3];
   udp_buffer[14] = local_time.milliseconds_u8[0];
   udp_buffer[15] = local_time.milliseconds_u8[1]; // << 16-byte packet, ready to encrypt v2 protocol

   net_on(RADIO_ON_TIMER_OFF);
   simple_udp_sendto(&udp_connection, udp_buffer, PROTOCOL_VERSION_V2_16BYTE, &addr);
}
*/

/*---------------------------------------------------------------------------*/

void udbp_v5_message_sender(uint8_t message_type, uint8_t data_1, uint8_t data_2)
{
	if (node_mode != MODE_NORMAL)
		return;

	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, &root_addr);
		
	if(uart_status() == 0)
		printf("DAG Node: Send message packet to DAG-root node\n");

	uint8_t payload_length = 5;
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_node.u8[0];
	udp_buffer[2] = DATA_TYPE_MESSAGE;//packet_counter_node.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_node.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_node.u8[1];//DATA_TYPE_MESSAGE;
	udp_buffer[8] = message_type;
	udp_buffer[9] = data_1;
	udp_buffer[10] = data_2;
/*
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
 */

	net_on(RADIO_ON_TIMER_OFF);
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_node.u16++;
	led_mode_set(LED_FLASH);
}

/*---------------------------------------------------------------------------*/

void udbp_v5_status_packet_sender(const uip_ipaddr_t *parent_addr,
                        uint32_t uptime_raw,
                        int16_t rssi_parent_raw,
                        uint8_t temp)
{
	if (parent_addr == NULL)
		return;

	u8_u32_t uptime;
	uptime.u32 = uptime_raw;

	uip_ipaddr_t addr;
	uip_ip6addr_copy(&addr, &root_addr);
   
	if(uart_status() == 0)
	{
		printf("DAG Node: Send status packet to DAG-root node: ");
		uip_debug_ipaddr_print(&addr);
		printf("\n");
	}

	uint8_t payload_length = 18;
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;//packet_counter_node.u8[0];
	udp_buffer[2] = DATA_TYPE_STATUS;//packet_counter_node.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_node.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = packet_counter_node.u8[1];//DATA_TYPE_STATUS;
	udp_buffer[8] = parent_addr->u8[8];
	udp_buffer[9] = parent_addr->u8[9];
	udp_buffer[10] = parent_addr->u8[10];
	udp_buffer[11] = parent_addr->u8[11];
	udp_buffer[12] = parent_addr->u8[12];
	udp_buffer[13] = parent_addr->u8[13];
	udp_buffer[14] = parent_addr->u8[14];
	udp_buffer[15] = parent_addr->u8[15];
	udp_buffer[16] = uptime.u8[0];
	udp_buffer[17] = uptime.u8[1];
	udp_buffer[18] = uptime.u8[2];
	udp_buffer[19] = uptime.u8[3];
	udp_buffer[20] = temp;
	udp_buffer[21] = BIG_VERSION;

	udp_buffer[22] = LITTLE_VERSION;
	udp_buffer[23] = spi_status;
/*
	udp_buffer[24] = DATA_RESERVED;
	udp_buffer[25] = DATA_RESERVED;
	udp_buffer[26] = DATA_RESERVED;
	udp_buffer[27] = DATA_RESERVED;
	udp_buffer[28] = DATA_RESERVED;
	udp_buffer[29] = DATA_RESERVED;
	udp_buffer[30] = DATA_RESERVED;
	udp_buffer[31] = DATA_RESERVED;
	udp_buffer[32] = DATA_RESERVED;
	udp_buffer[33] = DATA_RESERVED;
	udp_buffer[34] = DATA_RESERVED;
	udp_buffer[35] = DATA_RESERVED;
	udp_buffer[36] = DATA_RESERVED;
	udp_buffer[37] = DATA_RESERVED;
 */

	net_on(RADIO_ON_TIMER_OFF);
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	led_mode_set(LED_FLASH);
}

/*---------------------------------------------------------------------------*/

/*
void send_fw_chunk_req_packet(uint16_t chunk_num_raw)
{
   uip_ipaddr_t addr;
   uip_ip6addr_copy(&addr, &root_addr);

   u8_u16_t chunk_num;
   chunk_num.u16 = chunk_num_raw;

   uint8_t length = 10;
   uint8_t udp_buffer[length];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = CURRENT_DEVICE_VERSION;
   udp_buffer[2] = DATA_TYPE_FIRMWARE_CMD;
   udp_buffer[3] = DATA_TYPE_FIRMWARE_COMMAND_CHANK_REQ;
   udp_buffer[4] = chunk_num.u8[0];
   udp_buffer[5] = chunk_num.u8[1];
   udp_buffer[6] = DATA_RESERVED;
   udp_buffer[7] = DATA_RESERVED;
   udp_buffer[8] = DATA_RESERVED;
   udp_buffer[9] = DATA_RESERVED;
   simple_udp_sendto(&udp_connection, udp_buffer, length, &addr);
}
*/

/*---------------------------------------------------------------------------*/

void led_mode_set(uint8_t mode)
{
	led_mode = mode;
	if (led_mode == LED_OFF)
		led_off(LED_A);

	if (led_mode == LED_ON)
		led_on(LED_A);

	if (led_mode == LED_SLOW_BLINK || led_mode == LED_FAST_BLINK || led_mode == LED_FLASH)
		process_start(&led_process, NULL);
	else
		process_exit(&led_process);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(settings_dag_init, ev, data)
{
	PROCESS_BEGIN();
	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	read_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
	
	if(eeprom_dag.serial_configured == true) //При первом включении забивает нормальные настройки сети
	{
		if(eeprom_dag.aes_key_configured == true)
		{
			if((eeprom_dag.channel != 26) && (eeprom_dag.panid != 0xAABB))
			{
				eeprom_dag.channel = 26;
				eeprom_dag.panid = 0xAABB;
				write_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
			}
		}
	}

	if(!eeprom_dag.serial_configured) 
	{
		serial = eeprom_dag.serial;
		printf("Serial: %lu\n", serial);
	}
	else
	{
		printf("Serial number not declared\n******************************\n***PLEASE SET SERIAL NUMBER***\n******************************\n");
		led_mode_set(LED_FAST_BLINK);
		
		while(eeprom_dag.serial_configured)
		{
			PROCESS_YIELD();
		}	
	}
	
	if(!eeprom_dag.aes_key_configured) 
	{
		printf("AES-128 key:");
		for (uint8_t i = 0; i < 16; i++)
		{
			aes_key[i] = eeprom_dag.aes_key[i];
			printf(" %"PRIXX8, aes_key[i]);
		}
		printf("\n");
	}
	else
	{
		printf("AES-128 key not declared\n******************************\n******PLEASE SET AES KEY******\n******************************\n");
		led_mode_set(LED_FAST_BLINK);
		while(eeprom_dag.aes_key_configured)
		{
			PROCESS_YIELD();
		}		
	}
	
	if(!eeprom_dag.interface_configured) 
	{
		interface = eeprom_dag.interface;
	
		if(interface == INTERFACE_RS485)
			printf("Installed interface RS485\n");
		else if(interface == INTERFACE_CAN)
			printf("Installed interface CAN\n");
		else
			printf("Unknown interface\n");
	}
	else
	{
		printf("Interface not declared\n******************************\n*****PLEASE SET INTERFACE*****\n******************************\n");
		led_mode_set(LED_FAST_BLINK);
		
		while(eeprom_dag.interface_configured)
		{
			PROCESS_YIELD();
		}	
	}
	
	radio_value_t channel = 0;
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	if(channel != eeprom_dag.channel)
	{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_dag.channel);
		
		if (ti_lib_chipinfo_chip_family_is_cc26xx())
		{
			uint32_t freq_mhz = (2405 + 5 * (eeprom_dag.channel - 11));
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_dag.channel, freq_mhz);
		}

		if (ti_lib_chipinfo_chip_family_is_cc13xx())
		{
			uint32_t freq_khz = 863125 + (eeprom_dag.channel * 200);
			printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_dag.channel, freq_khz);
		}
	}
	
	if (ti_lib_chipinfo_chip_family_is_cc26xx())
	{
		radio_value_t panid = 0;
		NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		if(panid != eeprom_dag.panid)
		{
			NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_dag.panid);
			printf("PAN ID changed to: %"PRIXX16"\n", eeprom_dag.panid);
		}
	}
	
	hexraw_print(sizeof(eeprom_dag), (uint8_t*)(&eeprom_dag));
	
	process_post(&main_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(led_process, ev, data)
{
   PROCESS_BEGIN();
   if (ev == PROCESS_EVENT_EXIT)
      return 1;
   static struct etimer led_mode_timer;

   while (led_mode == LED_SLOW_BLINK || led_mode == LED_FAST_BLINK || led_mode == LED_FLASH)
   {
      if (led_mode == LED_FAST_BLINK)
         etimer_set( &led_mode_timer, CLOCK_SECOND/10);

      if (led_mode == LED_SLOW_BLINK)
         etimer_set( &led_mode_timer, CLOCK_SECOND/2);

      if (led_mode == LED_FLASH)
         etimer_set( &led_mode_timer, 1);

      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&led_mode_timer) );

      led_on(LED_A);

      if (led_mode == LED_FAST_BLINK)
         etimer_set( &led_mode_timer, CLOCK_SECOND/32);

      if (led_mode == LED_SLOW_BLINK)
         etimer_set( &led_mode_timer, CLOCK_SECOND/32);

      if (led_mode == LED_FLASH)
         etimer_set( &led_mode_timer, CLOCK_SECOND/16);

      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&led_mode_timer) );

      led_off(LED_A);

      if (led_mode == LED_FLASH)
         led_mode = LED_OFF;
   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(dag_node_button_process, ev, data)
{
   PROCESS_BEGIN();
   if (ev == PROCESS_EVENT_EXIT)
      return 1;

   PROCESS_PAUSE();

   while (1)
   {
      PROCESS_YIELD();

      if (ev == sensors_event)
      {
         if (data == &button_e_sensor_long_click)
         {
            led_mode_set(LED_ON);
			if(uart_status() == 0)
				printf("SYSTEM: Button E long click, reboot\n");
            watchdog_reboot();
         }
      }
   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(maintenance_process, ev, data)
{
   PROCESS_BEGIN();
   if (ev == PROCESS_EVENT_EXIT)
      return 1;

   PROCESS_PAUSE();

   while (1)
   {
      if (node_mode == MODE_NEED_REBOOT)
      {
            static struct etimer maintenance_reboot_timer;
            etimer_set( &maintenance_reboot_timer, (5 * CLOCK_SECOND));
            PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&maintenance_reboot_timer) );
            watchdog_reboot();
      }

      if (node_mode == MODE_NORMAL)
      {
         led_mode_set(LED_OFF);
         if (process_is_running(&status_send_process) == 0)
            process_start(&status_send_process, NULL);

         if (process_is_running(&root_find_process) == 1)
            process_exit(&root_find_process);

         if (non_answered_packet > MAX_NON_ANSWERED_PINGS)
         {
			if(uart_status() == 0)
				printf("DAG Node: Root not available, reboot\n");
            watchdog_reboot();
         }
      }

      if (node_mode == MODE_NOTROOT)
      {
         if (CLASS == CLASS_B)
         {
            led_mode_set(LED_OFF);
			if(uart_status() == 0)
				printf("DAG Node: Root not found, sleep\n");
            if (process_is_running(&dag_node_button_process) == 1)
               process_exit(&dag_node_button_process);

            if (process_is_running(&root_find_process) == 1)
               process_exit(&root_find_process);

            if (process_is_running(&status_send_process) == 1)
               process_exit(&status_send_process);

            if (process_is_running(&maintenance_process) == 1)
               process_exit(&maintenance_process);
            net_mode(RADIO_FREEDOM);
            net_off(RADIO_OFF_NOW);
            net_mode(RADIO_HOLD);

            etimer_set( &maintenance_timer, (5 * 60 * 60 * CLOCK_SECOND));
            PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&maintenance_timer) );
            watchdog_reboot();
         }

         if (CLASS == CLASS_C)
         {
            led_mode_set(LED_FAST_BLINK);
			if(uart_status() == 0)
				printf("DAG Node: Root not found, reboot\n"); //почему-то не перезагружается!
            watchdog_reboot();
         }
      }

      if (node_mode == MODE_JOIN_PROGRESS)
      {
         net_on(RADIO_ON_NORMAL);
         net_mode(RADIO_HOLD);
         led_mode_set(LED_SLOW_BLINK);

         if (process_is_running(&root_find_process) == 0)
            process_start(&root_find_process, NULL);

         if (process_is_running(&status_send_process) == 1)
            process_exit(&status_send_process);
      }

      etimer_set( &maintenance_timer, MAINTENANCE_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&maintenance_timer) );
   }
   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(status_send_process, ev, data)
{
   PROCESS_BEGIN();
   if (ev == PROCESS_EVENT_EXIT)
      return 1;

   static struct etimer status_send_timer;
   const rpl_dag_t *dag = NULL;
   PROCESS_PAUSE();

   while (1)
   {
      dag = rpl_get_any_dag();

      if (dag != NULL && node_mode == MODE_NORMAL)
      {

         if (rpl_parent_is_reachable(dag->preferred_parent) == 0)
         {
			if(uart_status() == 0)
				printf("DAG Node: Parent is not reachable\n");
            watchdog_reboot();
         }

         const uip_ipaddr_t *ipaddr_parent = rpl_get_parent_ipaddr(dag->preferred_parent);
         const struct link_stats *stat_parent = rpl_get_parent_link_stats(dag->preferred_parent);
         uint8_t temp = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
         if (ipaddr_parent != NULL && stat_parent != NULL)
         {
            udbp_v5_status_packet_sender(ipaddr_parent, rtc_s(), stat_parent->rssi, temp);
         }
         non_answered_packet++;
         if (non_answered_packet != 1)
         {
			if(uart_status() == 0)
				printf("DAG Node: Non-answered packet counter increase(status message): %"PRId8" \n", non_answered_packet);
         }
      }

      if (CLASS == CLASS_B)
      {
		if(uart_status() == 0)
			printf("DAG Node: Next status message planned on long interval(%"PRId8" min)\n", LONG_STATUS_INTERVAL/CLOCK_SECOND/60);
        etimer_set( &status_send_timer, LONG_STATUS_INTERVAL + (random_rand() % LONG_STATUS_INTERVAL) );
      }
      if (CLASS == CLASS_C)
      {
		 if(uart_status() == 0)
			printf("DAG Node: Next status message planned on short interval(%"PRId8" min)\n", SHORT_STATUS_INTERVAL/CLOCK_SECOND/60);
         etimer_set( &status_send_timer, SHORT_STATUS_INTERVAL + (random_rand() % SHORT_STATUS_INTERVAL) );
      }

      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&status_send_timer) && node_mode == MODE_NORMAL );
   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(fw_update_process, ev, data)
{
	PROCESS_BEGIN();

	if (ev == PROCESS_EVENT_EXIT)
		return 1;

	static uint16_t chunk_num = 0;
	static struct etimer fw_timer_deadline;
	static struct etimer fw_timer_delay_chunk;
	static struct etimer ota_image_erase_timer;
	static uint32_t page;

	/* Стираем память */

	if(uart_status() == 0)
		printf("[OTA]: Erasing OTA slot 1 [%#x, %#x)...\n", (ota_images[0]<<12), ((ota_images[0]+25)<<12));
	for (page=0; page<25; page++)
	{
		if(uart_status() == 0)
			printf("\r[OTA]: Erasing page %"PRIu32" at 0x%"PRIX32"..", page, (( ota_images[0] + page ) << 12));
		while( erase_extflash_page( (( ota_images[0] + page ) << 12) ) );

		udbp_v5_message_sender(DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS, page, DATA_NONE);
		etimer_set( &ota_image_erase_timer, (CLOCK_SECOND/20) );
		PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&ota_image_erase_timer) );
	}
	if(uart_status() == 0)
		printf("\r[OTA]: OTA slot 1 erased                        \n");


   /* Начинаем процесс обновления */

   while (1)
   {

      if (chunk_num < fw_chunk_quantity.u16) //Если остались незапрошенные пакеты
      {
         //send_fw_chunk_req_packet(chunk_num);
		 if(uart_status() == 0)
			printf("\r[OTA]: Request %"PRId16"/%"PRId16" chunk... ", chunk_num + 1, fw_chunk_quantity.u16);
         chunk_num++;
      }
      else //Если все пакеты запрошены
      {
		 if(uart_status() == 0)
			printf("\n[OTA]: End chunks\n");
         chunk_num = 0;
         fw_error_counter = 0;
         int crc_status_ota_slot = verify_ota_slot(1);
         OTAMetadata_t current_firmware;
         OTAMetadata_t ota_slot_1_firmware;
         get_current_metadata( &current_firmware );
         get_ota_slot_metadata(1, &ota_slot_1_firmware);

         if (crc_status_ota_slot == CORRECT_CRC)
         {
			if(uart_status() == 0)
				printf("[OTA]: New FW in OTA slot 1 correct CRC\n");
            if (current_firmware.uuid == ota_slot_1_firmware.uuid) //TODO: add univeral uuid(0xFFFFFFFF)
            {
			   if(uart_status() == 0)
					printf("[OTA]: New FW in OTA slot 1 correct UUID, set FW_FLAG_NEW_IMG_EXT, reboot\n");
               write_fw_flag(FW_FLAG_NEW_IMG_EXT);
               ti_lib_sys_ctrl_system_reset();
            }
            else
            {
			   if(uart_status() == 0)
					printf("[OTA]: New FW in OTA slot 1 non-correct firmware UUID\n");
               udbp_v5_message_sender(DEVICE_MESSAGE_OTA_NONCORRECT_UUID, DATA_NONE, DATA_NONE);
            }

         }
         else
         {
			if(uart_status() == 0)
				printf("[OTA]: New FW in OTA slot 1 non-correct CRC\n");
            udbp_v5_message_sender(DEVICE_MESSAGE_OTA_NONCORRECT_CRC, DATA_NONE, DATA_NONE);
         }
         process_exit(&fw_update_process);
         return 0;
      }

      etimer_set( &fw_timer, FW_DELAY + 1); //Таймер, который сбрасывается при получении пакета
      etimer_set( &fw_timer_deadline, FW_DELAY); //Таймер максимального ожидания

      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&fw_timer) ); //Ждем сброса таймера после получения пакета или истечения времени ожидания пакета

      if (etimer_expired(&fw_timer_deadline) && (chunk_num < fw_chunk_quantity.u16)) //Если истек таймер максимального ожидания(fw_timer_deadline)
      {
         if (fw_error_counter > FW_MAX_ERROR_COUNTER)
         {
			if(uart_status() == 0)
				printf("[OTA]: Not delivered chunk(>%"PRId8" errors), exit\n", FW_MAX_ERROR_COUNTER);
            udbp_v5_message_sender(DEVICE_MESSAGE_OTA_NOT_DELIVERED_CHUNK, DATA_NONE, DATA_NONE);
            process_exit(&fw_update_process);
            chunk_num = 0;
            fw_error_counter = 0;
            return 0;
         }
         else
         {
            fw_error_counter++;
            chunk_num--;
			if(uart_status() == 0)
				printf("[OTA]: Request %"PRId16"/%"PRId16" chunk again(%"PRId8" errors)\n", chunk_num + 1, fw_chunk_quantity.u16, fw_error_counter);
         }
      }
      etimer_set( &fw_timer_delay_chunk, (CLOCK_SECOND/20) ); //Таймер задержки перед запросом следующего чанка
      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&fw_timer_delay_chunk) );
   }

   PROCESS_END();
}


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(root_find_process, ev, data)
{
   PROCESS_BEGIN();

   if (ev == PROCESS_EVENT_EXIT)
      return 1;

   static struct etimer find_root_timer;
   static struct etimer find_root_limit_timer;
   static rpl_dag_t *root_find_dag = NULL;
   static uip_ds6_addr_t *ds6_addr = NULL;
   PROCESS_PAUSE();

   etimer_set( &find_root_limit_timer, ROOT_FIND_LIMIT_TIME);

   while (1)
   {
      if (node_mode == MODE_JOIN_PROGRESS)
      {
         if (!etimer_expired(&find_root_limit_timer))
         {
            ds6_addr = uip_ds6_get_global(ADDR_PREFERRED);
            if (ds6_addr != NULL)
            {
               root_find_dag = rpl_get_dag(&ds6_addr->ipaddr);
               if (root_find_dag != NULL)
               {
                  if ( led_mode != LED_FAST_BLINK)
                     led_mode_set(LED_FAST_BLINK);

                  udbp_v5_join_stage_1_sender(&root_find_dag->dag_id);
               }
            }
         }
         else
         {
            node_mode = MODE_NOTROOT;
			if(uart_status() == 0)
				printf("DAG Node: mode set to MODE_NOTROOT\n");
            process_exit(&maintenance_process);
            process_start(&maintenance_process, NULL);
         }

      }
      etimer_set( &find_root_timer, ROOT_FIND_INTERVAL + (random_rand() % ROOT_FIND_INTERVAL) );
      PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&find_root_timer) );
   }

   PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(dag_node_process, ev, data)
{
   PROCESS_BEGIN();

   PROCESS_PAUSE();

   simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_receiver);

   if (CLASS == CLASS_B)
      rpl_set_mode(RPL_MODE_LEAF);
   else
      rpl_set_mode(RPL_MODE_MESH);

   node_mode = MODE_JOIN_PROGRESS;

   spi_status = spi_test();

   packet_counter_node.u16 = 0;

   if(uart_status() == 0)
	   printf("Node started, %s mode, %s class, SPI %s, version %"PRIu8".%"PRIu8"\n",
					rpl_get_mode() == RPL_MODE_LEAF ? "leaf" : "no-leaf",
					CLASS == CLASS_B ? "B(sleep)" : "C(non-sleep)",
					spi_status == SPI_EXT_FLASH_ACTIVE ? "active" : "non-active",
					BIG_VERSION, LITTLE_VERSION);

   process_start(&dag_node_button_process, NULL);
   process_start(&maintenance_process, NULL);

   /*if (BOARD_IOID_UART_RX == IOID_UNUSED)
   {
	  if(uart_status() == 0)
		 printf("DAG Node: Shell not active, uart RX set to IOID_UNUSED\n");
      cc26xx_uart_set_input(NULL);
   }
   else
   {
      serial_shell_init();
      shell_reboot_init();
      shell_time_init();
      unwired_shell_init();
	  if(uart_status() == 0)
		printf("DAG Node: Shell activated, type \"help\" for command list\n");
   }*/


   SENSORS_ACTIVATE(batmon_sensor);

   PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);

   if(uart_status() == 0)
		printf("DAG Node: DAG active, join stage 4 packet received, mode set to MODE_NORMAL\n");
	
   led_mode_set(LED_SLOW_BLINK);
   node_mode = MODE_NORMAL;
   //udbp_v5_message_sender(DEVICE_MESSAGE_JOIN_SUCCESSFUL, DATA_NONE, DATA_NONE);
   net_mode(RADIO_FREEDOM);
   net_off(RADIO_OFF_NOW);
   //process_start(&time_sync_process, NULL); //Start timesync in main process, if necessary

   /*
   if (spi_status == SPI_EXT_FLASH_ACTIVE)
   {
      if (verify_ota_slot(0) == VERIFY_SLOT_CRC_ERROR)
      {
		 if(uart_status() == 0)
			printf("[OTA]: bad golden image, write current FW\n");
         udbp_v5_message_sender(DEVICE_MESSAGE_OTA_BAD_GOLDEN_IMAGE, DATA_NONE, DATA_NONE);
         backup_golden_image();
         watchdog_reboot();
      }
   }

   uint8_t current_ota_flag_status = read_fw_flag();
   if (current_ota_flag_status == FW_FLAG_NEW_IMG_INT)
   {
      write_fw_flag(FW_FLAG_PING_OK);
	  if(uart_status() == 0)
		printf("DAG Node: OTA flag changed to FW_FLAG_PING_OK\n");
      udbp_v5_message_sender(DEVICE_MESSAGE_OTA_UPDATE_SUCCESS, DATA_NONE, DATA_NONE);
      node_mode = MODE_NEED_REBOOT;
	  if(uart_status() == 0)
		printf("DAG Node: mode set to MODE_NEED_REBOOT(reboot after ota-update)\n");
      process_exit(&maintenance_process);
      process_start(&maintenance_process, NULL);
   }
	*/
	
   PROCESS_END();
}
