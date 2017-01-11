/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
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

#include "ud_binary_protocol.h"

#define DEBUG 1
#include "net/ip/uip-debug_UD.h"
//#include "../../core/contiki-net.h"

static struct simple_udp_connection udp_connection;
uip_ip6addr_t destination_address;
static uint8_t uart_command_buf[UART_DATA_LENGTH];
static uint8_t uart_iterator = 0;
static uint8_t uart_magic_sequence[UART_DATA_LENGTH] =
{0x01,0x16,0x16,0x16,0x16,0x10,
 0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
 0x01,0x01,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
 0x03,0x16,0x16,0x16,0x17,0x04};

SENSORS(&button_e_sensor);

/*---------------------------------------------------------------------------*/
PROCESS(rpl_root_process,"Unwired RPL root and udp data receiver");
AUTOSTART_PROCESSES(&rpl_root_process);
/*---------------------------------------------------------------------------*/
void send_confirmation_packet(const uip_ip6addr_t *dest_addr,
                              struct simple_udp_connection *connection)
{
    int lenght = 10;
    char buf[lenght];
    buf[0] = PROTOCOL_VERSION_V1;
    buf[1] = DEVICE_VERSION_V1;
    buf[2] = DATA_TYPE_CONFIRM;
    buf[3] = DATA_RESERVED;
    buf[4] = DATA_RESERVED;
    buf[5] = DATA_RESERVED;
    buf[6] = DATA_RESERVED;
    buf[7] = DATA_RESERVED;
    buf[8] = DATA_RESERVED;
    buf[9] = DATA_RESERVED;
    simple_udp_sendto(connection, buf, strlen(buf) + 1, dest_addr);
}
/*---------------------------------------------------------------------------*/
void send_command_packet(const uip_ip6addr_t *dest_addr,
                          struct simple_udp_connection *connection,
                          uint8_t target_ability,
                          uint8_t number_ability,
                          uint8_t data_to_ability)
{
    int lenght = 10;
    char buf[lenght];
    buf[0] = PROTOCOL_VERSION_V1;
    buf[1] = DEVICE_VERSION_V1;
    buf[2] = DATA_TYPE_COMMAND;
    buf[3] = target_ability;
    buf[4] = number_ability;
    buf[5] = data_to_ability;
    buf[6] = DATA_RESERVED;
    buf[7] = DATA_RESERVED;
    buf[8] = DATA_RESERVED;
    buf[9] = DATA_RESERVED;
    simple_udp_sendto(connection, buf, strlen(buf) + 1, dest_addr);
}

/*---------------------------------------------------------------------------*/
void dag_root_raw_print(const uip_ip6addr_t *addr, const uint8_t *data)
{
    printf("DAGROOTRAW1;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x;%02x"
           ";%02X;%02X;%02X;%02X;%02X;%02X;%02X;%02X;%02X;%02X;RAWEND"
            "   \n",
           ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2],
           ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5],
           ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8],
           ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11],
           ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15],
           data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9]);
}

/*---------------------------------------------------------------------------*/
void uart_packet_dump(uint8_t *uart_command_buf) {
    printf("\nOk packet: ");
    for (int i = 0; i < UART_DATA_LENGTH; i++)
    {
        printf("0x%02X ", uart_command_buf[i]);
    }
    printf("\n");
}

/*---------------------------------------------------------------------------*/
static int char_in(unsigned char c)
{
    led_blink(LED_A);
    if ((uart_iterator >= 0                                        && uart_iterator <= MAGIC_SEQUENCE_LENGTH - 1) ||
        (uart_iterator >= UART_DATA_LENGTH - MAGIC_SEQUENCE_LENGTH && uart_iterator <= UART_DATA_LENGTH - 1))
    {
        if (c != uart_magic_sequence[uart_iterator])
        {
            //printf(": BAD. iterator=%u\n", uart_iterator);
            uart_iterator = 0;
            return 1;
        }
        else
        {
            //printf(": OK. iterator=%u\n", uart_iterator);
        }
    }
    uart_command_buf[uart_iterator] = c;

    if (uart_iterator < UART_DATA_LENGTH-1)
    {
        uart_iterator++;
    }
    else
    {
        uart_iterator = 0;
        if (uart_command_buf[7] == UART_PROTOCOL_VERSION_V1 || true)
        {

            for (int i = 0; i <= 15; i++)
            {
               destination_address.u8[i] = uart_command_buf[i+7];
            }
            PRINT6ADDR(&destination_address);
            send_command_packet(&destination_address, &udp_connection, uart_command_buf[23], uart_command_buf[24], uart_command_buf[25]);
        }
        else
        {
            printf("USER: Incompatible protocol version!\n");
        }
        uart_packet_dump(uart_command_buf);
    }

    return 1;
}

/*---------------------------------------------------------------------------*/
static void udp_data_receiver(struct simple_udp_connection *connection,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
    led_on(LED_A);

    dag_root_raw_print(sender_addr, data);

  if (data[0] == PROTOCOL_VERSION_V1 && data[1] == DEVICE_VERSION_V1) {
      switch ( data[2] ) {
      case DATA_TYPE_JOIN:
          send_confirmation_packet(sender_addr, connection);
          printf("USER: DAG join packet from ");
          uip_debug_ipaddr_print(sender_addr);
          printf(" received, confirmation packet send\n");
          break;
      case DATA_TYPE_SENSOR_DATA: //data type(0x02 - data from sensors)
          printf("USER: data sensor from ");
          uip_debug_ipaddr_print(sender_addr);
          printf(", type: 0x%02X, event: 0x%02X, sensor: 0x%02X\n", data[3], data[6], data[5]);
          break;
      default:
          printf("USER: Incompatible data type!\n");
          break;
      }
  }
  else
  {
      printf("USER: Incompatible device or protocol version!\n");
  }
  led_off(LED_A);
}
/*---------------------------------------------------------------------------*/
static uip_ipaddr_t *set_global_address(void)
{
  static uip_ipaddr_t ipaddr;
  int i;
  uint8_t state;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  return &ipaddr;
}
/*---------------------------------------------------------------------------*/
static void create_rpl_dag(uip_ipaddr_t *ipaddr)
{
  struct uip_ds6_addr *root_if;
  root_if = uip_ds6_addr_lookup(ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    uip_ipaddr_t prefix;
    rpl_set_root(RPL_DEFAULT_INSTANCE, ipaddr);
    dag = rpl_get_any_dag();
    uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &prefix, 64);
    printf("Created a new RPL DAG, i'm root!\n");
  } else {
    printf("Failed to create a new RPL DAG :(\n");
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rpl_root_process, ev, data)
{
  uip_ipaddr_t *ipaddr;

  PROCESS_BEGIN();
  printf("Unwired RLP root and UDP data receiver. HELL-IN-CODE free. I hope.\n");

  ipaddr = set_global_address();
  create_rpl_dag(ipaddr);

  simple_udp_register(&udp_connection, UDP_DATA_PORT, NULL, UDP_DATA_PORT, udp_data_receiver);

  cc26xx_uart_set_input(&char_in);

  led_blink(LED_A);
  led_blink(LED_A);

  //ti_lib_ioc_pin_type_gpio_output(IOID_22);
  //ti_lib_gpio_set_dio(IOID_22);
  //ti_lib_gpio_set_output_enable_dio(IOID_3, GPIO_OUTPUT_ENABLE);

  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == sensors_event) {
        if(data == &button_e_sensor) {
            //PRINTF("Initiating global repair\n");
            //rpl_repair_root(RPL_DEFAULT_INSTANCE);
        }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
