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
*         Header file for DAG-node service
* \author
*         Manchenko Oleg man4enkoos@gmail.com
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
#include "uart/uart.h"

#define MAINTENANCE_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define SHORT_STATUS_INTERVAL			(10 * 60 * CLOCK_SECOND)
#define LONG_STATUS_INTERVAL			(20 * 60 * CLOCK_SECOND)
#define ROOT_FIND_INTERVAL				(2 * CLOCK_SECOND)
#define ROOT_FIND_LIMIT_TIME			(2 * 60 * CLOCK_SECOND)
#define FW_DELAY						(2 * CLOCK_SECOND)
#define FW_MAX_ERROR_COUNTER			5

#define CURRENT_DEVICE_SLEEP_TYPE             DEVICE_SLEEP_TYPE_LEAF
#define CURRENT_DEVICE_GROUP                  DEVICE_GROUP_BUTTON_SWITCH
#define CURRENT_DEVICE_VERSION                DEVICE_VERSION_V1
#define CURRENT_PROTOCOL_VERSION              PROTOCOL_VERSION_V1
#define CURRENT_ABILITY_1BYTE                 0b10000000
#define CURRENT_ABILITY_2BYTE                 0b00000000
#define CURRENT_ABILITY_3BYTE                 0b00000000
#define CURRENT_ABILITY_4BYTE                 0b00000000

#define FALSE							0x00
#define TRUE							0x01

#define MAX_NON_ANSWERED_PINGS			3

#define WAIT_RESPONSE					0.150 	//Максимальное время ожидания ответа от счетчика в секундах


#define AES128_PACKAGE_LENGTH			16	//Длина пакета AES-128

#define CC26XX_UART_INTERRUPT_ALL ( UART_INT_OE | UART_INT_BE | UART_INT_PE | \
									UART_INT_FE | UART_INT_RT | UART_INT_TX | \
									UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/

/* struct for simple_udp_send */
simple_udp_connection_t udp_connection;

volatile uint8_t spi_status;

volatile uint8_t led_mode;

volatile uint8_t non_answered_packet = 0;
volatile uip_ipaddr_t root_addr;

static struct etimer maintenance_timer;

volatile u8_u16_t fw_chunk_quantity;
volatile uint16_t fw_ext_flash_address = 0;
volatile uint8_t fw_error_counter = 0;

uint32_t ota_image_current_offset = 0;

static struct ctimer wait_response;
static bool wait_response_slave = 0;

rpl_dag_t *rpl_probing_dag;

volatile uint8_t process_message = 0;

static volatile union { uint16_t u16; uint8_t u8[2]; } packet_counter_root;

static uint8_t aes_buffer[128];
static uint8_t aes_key[16];
static uint8_t nonce_key[16];

static eeprom_t eeprom_dag;
static uint8_t interface; 
extern uint32_t serial;

/*---------------------------------------------------------------------------*/

PROCESS(settings_dag_init, "Initializing settings of DAG");
PROCESS(dag_node_process, "DAG-node process");
PROCESS(dag_node_button_process, "DAG-node button process");
PROCESS(root_find_process, "Root find process");
PROCESS(maintenance_process, "Maintenance process");
PROCESS(led_process, "Led process");

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

static void uart_from_air ( const uip_ipaddr_t *sender_addr,
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
		
		while(ti_lib_uart_chars_avail(UART0_BASE))
		{
			UARTCharGetNonBlocking(UART0_BASE);
		}
		
		ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
		reset_uart();
		wait_response_slave = 1;
		ctimer_set(&wait_response, (WAIT_RESPONSE * CLOCK_SECOND), wait_response_reset, NULL);
	}
}

/*---------------------------------------------------------------------------*/

void uart_to_air(char* data)
{
	if (node_mode == 2) //MODE_NOTROOT_SLEEP
	{
		watchdog_reboot();
	}

	if (node_mode == MODE_NORMAL)
	{
		uint16_t crc_uart;
		crc_uart = crc16_modbus((uint8_t*)&data[1], (data[0] - 2));
		
		if(data[0] > 3)
		{
			if(crc_uart != (uint16_t)((data[((uint8_t)(data[0]))] << 8) | data[(data[0]-1)]))
			{
				return; //CRC16 не совпала
			}
		}
		else
		{
			return; //Слишком маленькая длина фрейма
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

		net_on(RADIO_ON_TIMER_OFF);
		simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
		packet_counter_node.u16++;
		led_mode_set(LED_FLASH);
	}
}
/*---------------------------------------------------------------------------*/

void join_stage_1_sender(const uip_ipaddr_t *dest_addr)
{
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

	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
}
/*---------------------------------------------------------------------------*/
static void join_stage_3_sender(const uip_ipaddr_t *dest_addr,
										const uint8_t *data,
										uint16_t datalen)
{
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
	
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[12]), 16);

	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
}
/*---------------------------------------------------------------------------*/
static void join_stage_4_handler(const uip_ipaddr_t *sender_addr,
										const uint8_t *data,
										uint16_t datalen)
{	
	if (sender_addr == NULL)
		return;
			
	aes_cbc_decrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&data[8], (uint32_t*)(aes_buffer), 16);
	
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
		uip_ipaddr_copy(&root_addr, sender_addr); //Авторизован
		process_post(&dag_node_process, PROCESS_EVENT_CONTINUE, NULL);
		etimer_set(&maintenance_timer, 0);
		packet_counter_node.u16 = 1;
		return;
	}
	
	printf("Authorisation Error\n");
	//Не авторизован
}

/*---------------------------------------------------------------------------*/

static void ack_handler(const uip_ipaddr_t *sender_addr,
								const uint8_t *data,
								uint16_t datalen)
{
	non_answered_packet = 0;
	if(uart_status() == 0)
		printf("DAG Node: ACK packet received, non-answered packet counter: %"PRId8" \n", non_answered_packet);
	net_off(RADIO_OFF_NOW);
	process_message = PT_MESSAGE_ACK_RECIEVED;
	//process_post_synch(&main_process, PROCESS_EVENT_MSG, (process_data_t)&process_message);
}

/*---------------------------------------------------------------------------*/

static void udp_receiver(struct simple_udp_connection *c,
						const uip_ipaddr_t *sender_addr,
						uint16_t sender_port,
						const uip_ipaddr_t *receiver_addr,
						uint16_t receiver_port,
						const uint8_t *data, 
						uint16_t datalen)
{
	uint8_t protocol_version = data[0];

	if(uart_status() == 0)
	{
		printf("DAG Node: UDP packet received(%"PRIu8"): ", datalen);
		for (uint16_t i = 0; i < datalen; i++)
			printf("%"PRIXX8, data[i]);
		printf("\n");
	}

	if (protocol_version == UDBP_PROTOCOL_VERSION_V5)
	{
		uint8_t endpoint = data[UDUP_V5_MODULE_ID];
		if (endpoint == UNWDS_6LOWPAN_SYSTEM_MODULE_ID)
		{
			uint8_t packet_type = data[UDUP_V5_PACKET_TYPE];
			
			if (packet_type == DATA_TYPE_JOIN_V5_STAGE_2)
			{
				join_stage_3_sender(sender_addr, data, datalen);
			}
			
			else if (packet_type == DATA_TYPE_JOIN_V5_STAGE_4)
			{
				join_stage_4_handler(sender_addr, data, datalen);
			}
			
			else if (packet_type == DATA_TYPE_ACK)
			{
				ack_handler(sender_addr, data, datalen);
			}
			
			else if (packet_type == UART_FROM_AIR_TO_TX)
			{
				uart_from_air(sender_addr, data, datalen);
			}
			
			else
			{
				if(uart_status() == 0)
					printf("DAG Node: Incompatible packet type(endpoint UNWDS_6LOWPAN_SYSTEM_MODULE_ID): %"PRIXX8"\n", packet_type);
			}
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
		;
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
	
	process_post(&main_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(led_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;
	
	static struct etimer led_mode_timer;

	while((led_mode == LED_SLOW_BLINK) || (led_mode == LED_FAST_BLINK) || (led_mode == LED_FLASH))
	{
		if(led_mode == LED_FAST_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/10);

		if(led_mode == LED_SLOW_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/2);

		if(led_mode == LED_FLASH)
			etimer_set(&led_mode_timer, 1);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&led_mode_timer));

		led_on(LED_A);

		if(led_mode == LED_FAST_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/32);

		if(led_mode == LED_SLOW_BLINK)
			etimer_set(&led_mode_timer, CLOCK_SECOND/32);

		if(led_mode == LED_FLASH)
			etimer_set(&led_mode_timer, CLOCK_SECOND/16);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&led_mode_timer) );

		led_off(LED_A);

		if(led_mode == LED_FLASH)
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
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;

	PROCESS_PAUSE();

	while(1)
	{
		if(node_mode == MODE_NEED_REBOOT)
		{
			static struct etimer maintenance_reboot_timer;
			etimer_set(&maintenance_reboot_timer, (5*CLOCK_SECOND));
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&maintenance_reboot_timer) );
			watchdog_reboot();
		}

		if(node_mode == MODE_NORMAL)
		{
			led_mode_set(LED_OFF);

			if(process_is_running(&root_find_process) == 1)
				process_exit(&root_find_process);

			if(non_answered_packet > MAX_NON_ANSWERED_PINGS)
			{
				if(uart_status() == 0)
					printf("DAG Node: Root not available, reboot\n");
				watchdog_reboot();
			}
		}

		if(node_mode == MODE_NOTROOT)
		{
			if(CLASS == CLASS_B)
			{
				led_mode_set(LED_OFF);
				
				if(uart_status() == 0)
					printf("DAG Node: Root not found, sleep\n");
				
				if(process_is_running(&dag_node_button_process) == 1)
					process_exit(&dag_node_button_process);

				if(process_is_running(&root_find_process) == 1)
					process_exit(&root_find_process);

				if(process_is_running(&maintenance_process) == 1)
					process_exit(&maintenance_process);
				
				net_mode(RADIO_FREEDOM);
				net_off(RADIO_OFF_NOW);
				net_mode(RADIO_HOLD);

				etimer_set(&maintenance_timer, (5 * 60 * 60 * CLOCK_SECOND));
				PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&maintenance_timer) );
				watchdog_reboot();
			}

			if(CLASS == CLASS_C)
			{
				led_mode_set(LED_FAST_BLINK);
				if(uart_status() == 0)
					printf("DAG Node: Root not found, reboot\n"); //почему-то не перезагружается!
				watchdog_reboot();
			}
		}

		if(node_mode == MODE_JOIN_PROGRESS)
		{
			net_on(RADIO_ON_NORMAL);
			net_mode(RADIO_HOLD);
			led_mode_set(LED_SLOW_BLINK);

			if(process_is_running(&root_find_process) == 0)
				process_start(&root_find_process, NULL);
		}

		etimer_set(&maintenance_timer, MAINTENANCE_INTERVAL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&maintenance_timer) );
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
		if(node_mode == MODE_JOIN_PROGRESS)
		{
			if(!etimer_expired(&find_root_limit_timer))
			{
				ds6_addr = uip_ds6_get_global(ADDR_PREFERRED);
				if(ds6_addr != NULL)
				{
					root_find_dag = rpl_get_dag(&ds6_addr->ipaddr);
					if(root_find_dag != NULL)
					{
						if(led_mode != LED_FAST_BLINK)
						led_mode_set(LED_FAST_BLINK);
						join_stage_1_sender(&root_find_dag->dag_id);
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
	packet_counter_node.u16 = 1;

	if(uart_status() == 0)
		printf("Node started, %s mode, %s class, SPI %s, version %"PRIu8".%"PRIu8"\n",
				rpl_get_mode() == RPL_MODE_LEAF ? "leaf" : "no-leaf",
				CLASS == CLASS_B ? "B(sleep)" : "C(non-sleep)",
				spi_status == SPI_EXT_FLASH_ACTIVE ? "active" : "non-active",
				BIG_VERSION, LITTLE_VERSION);

	process_start(&dag_node_button_process, NULL);
	process_start(&maintenance_process, NULL);

	SENSORS_ACTIVATE(batmon_sensor);

	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);

	if(uart_status() == 0)
		printf("DAG Node: DAG active, join stage 4 packet received, mode set to MODE_NORMAL\n");
	
	led_mode_set(LED_SLOW_BLINK);
	node_mode = MODE_NORMAL;
	net_mode(RADIO_FREEDOM);
	net_off(RADIO_OFF_NOW);

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/