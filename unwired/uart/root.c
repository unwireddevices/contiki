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

#include "ti-lib.h"
#include "dev/cc26xx-uart.h"

#include "../ud_binary_protocol.h"
#include "xxf_types_helper.h"
#include "dev/watchdog.h"
#include "root.h"
#include "../root-node.h"
#include "../system-common.h"//
#include "../ecc.h"//

#include "sys/etimer.h"


#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
   UART_INT_RX | UART_INT_CTS)

/*---------------------------------------------------------------------------*/

/* Buttons on DIO 1 */
SENSORS(&button_e_sensor_click, &button_e_sensor_long_click);

PROCESS(rpl_root_process, "Unwired RPL root and udp data receiver");

AUTOSTART_PROCESSES(&rpl_root_process);

/*---------------------------------------------------------------------------*/


static void off_uart(uint32_t rx_dio, uint32_t tx_dio)
{
   ti_lib_ioc_port_configure_set(tx_dio, IOC_PORT_GPIO, IOC_STD_OUTPUT);
   ti_lib_ioc_port_configure_set(rx_dio, IOC_PORT_GPIO, IOC_STD_INPUT);
   ti_lib_gpio_set_output_enable_dio(tx_dio, GPIO_OUTPUT_ENABLE);
   ti_lib_gpio_set_dio(tx_dio);

   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_disable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_fifo_disable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_disable(UART0_BASE);
}

static void on_uart(uint32_t rx_dio, uint32_t tx_dio, uint32_t baud_rate)
{
   if(baud_rate == 9600) //Не обновляется скорость UART
   {
	  (*(unsigned long*)(0x40001024)) = 312;
	  (*(unsigned long*)(0x40001028)) = 32;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   else 
   {
	  (*(unsigned long*)(0x40001024)) = 312;
	  (*(unsigned long*)(0x40001028)) = 32;
	  (*(unsigned long*)(0x4000102C)) = 112; //без обновления регистра LCRH скорость не обновляется (FEN && WLEN)
   }
   
   ti_lib_ioc_pin_type_gpio_output(tx_dio);
   ti_lib_gpio_set_dio(tx_dio);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_ioc_pin_type_uart(UART0_BASE, rx_dio, tx_dio, IOID_UNUSED, IOID_UNUSED);
   ti_lib_uart_config_set_exp_clk(UART0_BASE, ti_lib_sys_ctrl_clock_get(), baud_rate, 
                  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   ti_lib_uart_fifo_level_set(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
   ti_lib_uart_fifo_enable(UART0_BASE);
   ti_lib_uart_int_enable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_enable(UART0_BASE);
   while(ti_lib_uart_busy(UART0_BASE));
   ti_lib_uart_int_clear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);
}

void getRandomBytes(void *p_dest, unsigned p_size)
{
    // if(read(randfd, p_dest, p_size) != (int)p_size)
    // {
        // printf("Failed to get random bytes.\n");
    // }
}

PROCESS_THREAD(rpl_root_process, ev, data)
{
	PROCESS_BEGIN();

	printf("Start Unwired RLP root.\n");
	
	printf("MAC: ");
	hexraw_print(8, ((uint8_t*)(0x500012F0)));
	printf("\n");
	
	EccPoint l_public;
	printf("EccPoint: ");
	
	//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
	l_public.x[0] = 0x2FDEBE6C;
	l_public.x[1] = 0xA47719F1;
	l_public.x[2] = 0x99034867;
	l_public.x[3] = 0xF3698B3E;
	l_public.x[4] = 0x3E5E1334;
	l_public.x[5] = 0x6D54D442;
	l_public.x[6] = 0x18D0D2DC;
	l_public.x[7] = 0x6FF51D33;
	
	//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
	l_public.y[0] = 0x0C36BF5C;
	l_public.y[1] = 0x06FCC1EB;
	l_public.y[2] = 0x14129CC5;
	l_public.y[3] = 0xCC769E3E;
	l_public.y[4] = 0xA4D8C08B;
	l_public.y[5] = 0x9B96F6A5;
	l_public.y[6] = 0x47374183;
	l_public.y[7] = 0xAEF2B7CD;
	
	hexraw_print(sizeof(EccPoint), ((uint8_t*)(&l_public)));
	printf("\n");
	
	printf("l_private: ");
    uint32_t l_private[NUM_ECC_DIGITS];
	
	//d = 24 B8 50 94 0A 9B 79 02 14 D7 7E 95 D4 0B 11 49 5A 30 D5 80 CF 8C 39 0B BC 09 F1 BC FA 10 88 06
	l_private[0] = 0xFA108806;
	l_private[1] = 0xBC09F1BC;
	l_private[2] = 0xCF8C390B;
	l_private[3] = 0x5A30D580;
	l_private[4] = 0xD40B1149;
	l_private[5] = 0x14D77E95;
	l_private[6] = 0x0A9B7902;
	l_private[7] = 0x24B85094;
	
	hexraw_print(sizeof(l_private), ((uint8_t*)(&l_private)));
	printf("\n");

    uint32_t l_hash[NUM_ECC_DIGITS];
	
	//hash = 15 7D 3F 09 0D 8A 09 10 C3 C1 A6 8C E4 10 21 BD 0E A4 15 C5 DF A8 C1 5C 3C D3 07 79 F5 BE 64 2A
	l_hash[0] = 0xF5BE642A;
	l_hash[1] = 0x3CD30779;
	l_hash[2] = 0xDFA8C15C;
	l_hash[3] = 0x0EA415C5;
	l_hash[4] = 0xE41021BD;
	l_hash[5] = 0xC3C1A68C;
	l_hash[6] = 0x0D8A0910;
	l_hash[7] = 0x157D3F09;
	
    uint32_t l_random[NUM_ECC_DIGITS];
	
	//random = 48 7A 0D 38 75 BD B8 12 9A 13 3F 00 5B 81 EC 32 F6 FB A3 06 5B C9 EE 44 06 0B 6E D5 73 53 83 2C
	l_random[0] = 0x7353832C;
	l_random[1] = 0x060B6ED5;
	l_random[2] = 0x5BC9EE44;
	l_random[3] = 0xF6FBA306;
	l_random[4] = 0x5B81EC32;
	l_random[5] = 0x9A133F00;
	l_random[6] = 0x75BDB812;
	l_random[7] = 0x487A0D38;
	
	printf("l_hash: ");
	hexraw_print(sizeof(l_hash), ((uint8_t*)(&l_hash)));
	printf("\n");
	
	printf("l_random: ");
	hexraw_print(sizeof(l_random), ((uint8_t*)(&l_random)));
	printf("\n");
    
    uint32_t r[NUM_ECC_DIGITS];
    uint32_t s[NUM_ECC_DIGITS];
	
	printf("r: ");
	hexraw_print(sizeof(r), ((uint8_t*)(&r)));
	printf("\n");
	
	printf("s: ");
	hexraw_print(sizeof(s), ((uint8_t*)(&s)));
	printf("\n");
    
    printf("Testing\n");
	
	/* ecc_make_key() function.
	Create a public/private key pair.

	You must use a new nonpredictable random number to generate each new key pair.

	Outputs:
		p_publicKey  - Will be filled in with the point representing the public key.
		p_privateKey - Will be filled in with the private key.

	Inputs:
		p_random - The random number to use to generate the key pair.

	Returns 1 if the key pair was generated successfully, 0 if an error occurred. If 0 is returned,
	try again with a different random number.
	*/
	
	//int ecc_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS]);
	
	// if(!ecc_make_key(&l_public, l_private, l_private)) 
		// printf("ecc_make_key() failed\n");
	// else
		// printf("ecc_make_key() ok\n");
	
	// printf("EccPoint: ");
	// hexraw_print(sizeof(EccPoint), ((uint8_t*)(&l_public)));
	// printf("\n");
	
	// printf("l_private: ");
	// hexraw_print(sizeof(l_private), ((uint8_t*)(&l_private)));
	// printf("\n");
	
	/* ecdsa_sign() function.
	Generate an ECDSA signature for a given hash value.

	Usage: Compute a hash of the data you wish to sign (SHA-2 is recommended) and pass it in to
	this function along with your private key and a random number.
	You must use a new nonpredictable random number to generate each new signature.

	Outputs:
	r, s - Will be filled in with the signature values.

	Inputs:
	p_privateKey - Your private key.
	p_random     - The random number to use to generate the signature.
	p_hash       - The message hash to sign.

	Returns 1 if the signature generated successfully, 0 if an error occurred. If 0 is returned,
	try again with a different random number.
	*/
	//int ecdsa_sign(uint32_t r[NUM_ECC_DIGITS],
				 //  uint32_t s[NUM_ECC_DIGITS], 
				 //  uint32_t p_privateKey[NUM_ECC_DIGITS],
				 //  uint32_t p_random[NUM_ECC_DIGITS], 
				 //  uint32_t p_hash[NUM_ECC_DIGITS]);
	
	if(!ecdsa_sign(r, s, l_private, l_random, l_hash))
		printf("ecdsa_sign() failed\n");
	else
		printf("ecdsa_sign() ok\n");
	
	printf("l_private: ");
	hexraw_print(sizeof(l_private), ((uint8_t*)(&l_private)));
	printf("\n");
	
	printf("l_hash: ");
	hexraw_print(sizeof(l_hash), ((uint8_t*)(&l_hash)));
	printf("\n");
	
	printf("l_random: ");
	hexraw_print(sizeof(l_random), ((uint8_t*)(&l_random)));
	printf("\n");
	
	printf("r: ");
	hexraw_print(sizeof(r), ((uint8_t*)(&r)));
	printf("\n");
	
	printf("s: ");
	hexraw_print(sizeof(s), ((uint8_t*)(&s)));
	printf("\n");
	
	printf("EccPoint: ");
	hexraw_print(sizeof(EccPoint), ((uint8_t*)(&l_public)));
	printf("\n");
	
	if(!ecc_valid_public_key(&l_public))
		printf("Not a valid public key!\n");
	else
		printf("Valid public key!\n");
	
	printf("EccPoint: ");
	hexraw_print(sizeof(EccPoint), ((uint8_t*)(&l_public)));
	printf("\n");
	
	if(!ecdsa_verify(&l_public, l_hash, r, s))
		printf("ecdsa_verify() failed\n");
	else
		printf("ecdsa_verify() ok\n");
    
    printf("\n");

	/* if you do not execute "cleanall" target, rpl-root can build in "leaf" configuration. Diagnostic message */
	if (RPL_CONF_LEAF_ONLY == 1)
		printf("\nWARNING: leaf mode on rpl-root!\n");

	rpl_initialize();

	root_node_initialize();

	static struct etimer shell_off;
	etimer_set(&shell_off, CLOCK_SECOND * 15);

	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&shell_off));
   
	if (BOARD_IOID_UART_TX != BOARD_IOID_ALT_UART_TX || BOARD_IOID_UART_RX != BOARD_IOID_ALT_UART_RX)
	{
		if(uart_status_r() == 0)
		{
			//printf("\nUART_IBRD: %lu \nUART_FBRD: %lu \nLHCR: %lu \n ", (*(unsigned long*)(0x40001024)), (*(unsigned long*)(0x40001028)), (*(unsigned long*)(0x4000102C)) );
			printf("UDM: UART change to alt(RX: %"PRIu16", TX: %"PRIu16")\n", BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX);
		}
		off_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX);
		on_uart(BOARD_IOID_ALT_UART_RX, BOARD_IOID_ALT_UART_TX, 9600);
		set_uart_r();
		//on_uart(BOARD_IOID_UART_RX, BOARD_IOID_UART_TX, 9600);
		//clock_delay((CLOCK_SECOND / 9600) * 1);
		//printf("UDM: Alt UART active\n");
		//printf("\nUART_IBRD: %lu \nUART_FBRD: %lu \nLHCR: %lu \n ", (*(unsigned long*)(0x40001024)), (*(unsigned long*)(0x40001028)), (*(unsigned long*)(0x4000102C)) );
	}

   while (1)
   {
      PROCESS_WAIT_EVENT();
      if (ev == sensors_event && data == &button_e_sensor_long_click)
      {
         led_on(LED_A);
		 if(uart_status_r() == 0)
			printf("UDM: Button E long click, reboot\n");
         watchdog_reboot();
      }
   }

   PROCESS_END();
}
/*---------------------------------------------------------------------------*/