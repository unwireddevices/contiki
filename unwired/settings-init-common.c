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
 *         Internal flash write-read functions for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"

#include <string.h>
#include <stdio.h>
#include "ti-lib.h"

#include "xxf_types_helper.h"

#include "int-flash-common.h"
#include "settings-init-common.h"
#include "root-node.h" 
#include "uart/root.h"
#include "uart/uart.h"

PROCESS(settings_root_init, "Initializing settings of ROOT");
PROCESS(settings_dag_init, "Initializing settings of DAG");

static eeprom_t eeprom_root;
static eeprom_t eeprom_dag;

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

PROCESS_THREAD(settings_dag_init, ev, data)
{
	PROCESS_BEGIN();
	// if (ev == PROCESS_EVENT_EXIT)
		// return 1;

	// read_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
	
	// if(eeprom_dag.serial_configured == true) //При первом включении забивает нормальные настройки сети
	// {
		// if(eeprom_dag.aes_key_configured == true)
		// {
			// if((eeprom_dag.channel != 26) && (eeprom_dag.panid != 0xAABB))
			// {
				// eeprom_dag.channel = 26;
				// eeprom_dag.panid = 0xAABB;
				// write_eeprom((uint8_t*)&eeprom_dag, sizeof(eeprom_dag));
			// }
		// }
	// }

	// if(!eeprom_dag.serial_configured) 
	// {
		// serial = eeprom_dag.serial;
		// printf("Serial: %lu\n", serial);
	// }
	// else
	// {
		// printf("Serial number not declared\n******************************\n***PLEASE SET SERIAL NUMBER***\n******************************\n");
		// led_mode_set(LED_FAST_BLINK);
		
		// while(eeprom_dag.serial_configured)
		// {
			// PROCESS_YIELD();
		// }	
	// }
	
	// if(!eeprom_dag.aes_key_configured) 
	// {
		// printf("AES-128 key:");
		// for (uint8_t i = 0; i < 16; i++)
		// {
			// aes_key[i] = eeprom_dag.aes_key[i];
			// printf(" %"PRIXX8, aes_key[i]);
		// }
		// printf("\n");
		;
	// }
	// else
	// {
		// printf("AES-128 key not declared\n******************************\n******PLEASE SET AES KEY******\n******************************\n");
		// led_mode_set(LED_FAST_BLINK);
		// while(eeprom_dag.aes_key_configured)
		// {
			// PROCESS_YIELD();
		// }		
	// }
	
	// if(!eeprom_dag.interface_configured) 
	// {
		// interface = eeprom_dag.interface;
	
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
		
		// while(eeprom_dag.interface_configured)
		// {
			// PROCESS_YIELD();
		// }	
	// }
	
	// radio_value_t channel = 0;
	// NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
	
	// if(channel != eeprom_dag.channel)
	// {
		// NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, eeprom_dag.channel);
		
		// if (ti_lib_chipinfo_chip_family_is_cc26xx())
		// {
			// uint32_t freq_mhz = (2405 + 5 * (eeprom_dag.channel - 11));
			// printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" MHz)\n", (int)eeprom_dag.channel, freq_mhz);
		// }

		// if (ti_lib_chipinfo_chip_family_is_cc13xx())
		// {
			// uint32_t freq_khz = 863125 + (eeprom_dag.channel * 200);
			// printf("Changed the radio-channel to: %"PRIint" (%"PRIu32" kHz)\n", (int)eeprom_dag.channel, freq_khz);
		// }
	// }
	
	// if (ti_lib_chipinfo_chip_family_is_cc26xx())
	// {
		// radio_value_t panid = 0;
		// NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &panid);
		
		// if(panid != eeprom_dag.panid)
		// {
			// NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, eeprom_dag.panid);
			// printf("PAN ID changed to: %"PRIXX16"\n", eeprom_dag.panid);
		// }
	// }
	
	// process_post(&main_process, PROCESS_EVENT_CONTINUE, NULL);
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/