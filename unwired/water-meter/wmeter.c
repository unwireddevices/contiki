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
*         water meter service for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
* \author
*         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
*/
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "uip-ds6-route.h"
#include "net/ip/uip-debug.h"
#include "dev/leds.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "button-sensor.h"
#include "board.h"
#include "board-peripherals.h"
#include "simple-udp.h"

#include "wmeter.h"
#include "radio_power.h"
#include "../dag_node.h"
#include "gpio-interrupt.h"
#include "dev/cc26xx-uart.h"
#include "aux-ctrl.h"

#include "xxf_types_helper.h"

#include "ti-lib.h"
#include "clock.h"
#include "../ud_binary_protocol.h"
#include "../int-flash-common.h"
#include "../rtc-common.h"
#include "../system-common.h"

#define L_OUT_DIO IOID_4
#define T_OUT_DIO IOID_11
#define T_IN_DIO IOID_12

#define SEND_TIME             (15 * 60 * CLOCK_SECOND)

#define OUT_GPIO_CFG            (IOC_CURRENT_8MA | IOC_STRENGTH_AUTO |      \
                                    IOC_NO_IOPULL | IOC_SLEW_DISABLE |         \
                                    IOC_HYST_DISABLE | IOC_NO_EDGE |           \
                                    IOC_INT_DISABLE | IOC_IOMODE_NORMAL |      \
                                    IOC_NO_WAKE_UP | IOC_INPUT_DISABLE )

/*---------------------------------------------------------------------------*/

static uint32_t water_counter = 0;

/*---------------------------------------------------------------------------*/

/* Register buttons sensors */
SENSORS(&button_a_sensor_click);

/* register dimmer process */
PROCESS(main_process, "water meter control process");
PROCESS(send_data_process, "Uart-data send process");

/* set autostart processes */
AUTOSTART_PROCESSES(&dag_node_process, &main_process, &send_data_process);

/*---------------------------------------------------------------------------*/


void send_wmeter_counter_packet(uint32_t water_counter_u32)
{
   if (node_mode != MODE_NORMAL)
      return;

   uip_ipaddr_t addr;
   uip_ip6addr_copy(&addr, &root_addr);

   u8_u32_t water_counter;

   water_counter.u32 = water_counter_u32;

   uint8_t udp_buffer[PROTOCOL_VERSION_V2_16BYTE];
   udp_buffer[0] = PROTOCOL_VERSION_V1;
   udp_buffer[1] = DEVICE_VERSION_V1;
   udp_buffer[2] = DATA_TYPE_MESSAGE;
   udp_buffer[3] = DEVICE_MESSAGE_FREE_DATA;
   udp_buffer[4] = water_counter.u8[0];
   udp_buffer[5] = water_counter.u8[1];
   udp_buffer[6] = water_counter.u8[2];
   udp_buffer[7] = water_counter.u8[3];
   udp_buffer[8] = DATA_NONE;
   udp_buffer[9] = DATA_NONE;
   udp_buffer[10] = DATA_NONE;
   udp_buffer[11] = DATA_NONE;
   udp_buffer[12] = DATA_NONE;
   udp_buffer[13] = DATA_NONE;
   udp_buffer[14] = DATA_NONE;
   udp_buffer[15] = DATA_NONE; // << 16-byte packet, ready to encrypt v2 protocol

   net_on(RADIO_ON_TIMER_OFF);
   simple_udp_sendto(&udp_connection, udp_buffer, PROTOCOL_VERSION_V2_16BYTE, &addr);
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(send_data_process, ev, data)
{
   PROCESS_BEGIN();

   static struct etimer send_data_process_timer;
   PROCESS_PAUSE();

   while (1)
   {
      send_wmeter_counter_packet(water_counter);
      printf("Send counter: %"PRIu32"\n", water_counter);
      etimer_set(&send_data_process_timer, SEND_TIME);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_data_process_timer));
   }

   PROCESS_END();
}



/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{
   PROCESS_BEGIN();

   static struct etimer water_meter_timer;
   static uint32_t current_value = 0;
   static uint32_t prev_measured = 0;
   static uint32_t prev_saved = 0;

   static aux_consumer_module_t adc_aux = {
      .clocks = AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK
    };

   PROCESS_PAUSE();

   printf("Unwired water meter device. HELL-IN-CODE free. I hope.\n");

   ti_lib_ioc_pin_type_gpio_output(L_OUT_DIO);
   ti_lib_ioc_pin_type_gpio_output(T_OUT_DIO);
   ti_lib_ioc_port_configure_set(L_OUT_DIO, IOC_PORT_GPIO, OUT_GPIO_CFG);
   ti_lib_ioc_port_configure_set(T_OUT_DIO, IOC_PORT_GPIO, OUT_GPIO_CFG);

   ti_lib_ioc_pin_type_gpio_input(T_IN_DIO);

   while (1)
   {
      /* On internal and external pull-up */
      ti_lib_gpio_write_dio(T_OUT_DIO, 1);
      ti_lib_gpio_write_dio(L_OUT_DIO, 0);
      ti_lib_ioc_io_port_pull_set(T_IN_DIO, IOC_IOPULL_UP);
      //clock_delay_usec(200);

      /* Init ADC */
      aux_ctrl_register_consumer(&adc_aux);
      ti_lib_aux_adc_select_input(ADC_COMPB_IN_AUXIO2);
      ti_lib_aux_adc_enable_sync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL); // AUXADC_REF_FIXED = nominally 4.3 V

      /* Off internal pull-up and on IR-LED */
      ti_lib_ioc_io_port_pull_set(T_IN_DIO, IOC_NO_IOPULL);
      ti_lib_gpio_write_dio(T_OUT_DIO, 1);
      ti_lib_gpio_write_dio(L_OUT_DIO, 1);

      clock_delay_usec(150);
      //current_value = ti_lib_gpio_read_dio(T_IN_DIO);

      /* Read ADC value */
      ti_lib_aux_adc_gen_manual_trigger();

      /* Off external pull-up and IR LED */
      ti_lib_gpio_write_dio(L_OUT_DIO, 0);
      ti_lib_gpio_write_dio(T_OUT_DIO, 0);

      /* Read ADC sample */
      uint16_t singleSample = ti_lib_aux_adc_read_fifo();
      ti_lib_aux_adc_flush_fifo();

      /* Disable ADC */
      ti_lib_aux_adc_disable();
      aux_ctrl_unregister_consumer(&adc_aux);

      if (singleSample > 700)
         current_value = 1;
      else
         current_value = 0;

      //printf("ADC: %d mv on ADC\r\n", singleSample);

      if (current_value == prev_measured)
      {
         if ((prev_saved == 0) && (current_value == 1))
         {
            water_counter++;
            //printf("METER: counter inc!\r\n");
         }
         prev_saved = current_value;
      }
      prev_measured = current_value;

      etimer_set(&water_meter_timer, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&water_meter_timer));
   }

   PROCESS_END();
}
