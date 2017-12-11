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
 *         System functions for Unwired Devices mesh smart house system(UDMSHS %) <- this is smile
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "dev/watchdog.h"

#include <string.h>
#include <stdio.h>

#include "ud_binary_protocol.h"
#include "xxf_types_helper.h"

#include "batmon-sensor.h"
#include "core/lib/sensors.h"

#include "net/rpl/rpl.h"
#include "net/link-stats.h"

#include <ctype.h> // for str2xxx
#include <errno.h> // for str2xxx
#include <limits.h> // for str2xxx

#include "system-common.h"
#include "ota-main.h"
#include "ota-common.h"

/*---------------------------------------------------------------------------*/

void
flash_damp_hex(uint8_t mode)
{
   const uint32_t start_adress = (ota_images[1-1] << 12);
   const uint32_t read_length = 0x400;
   uint8_t flash_read_data_buffer[read_length];

   printf("SPIFLASH DAMP: \n");
   for (uint8_t page=0; page < 100; page++ )
   {
      watchdog_periodic();
      ext_flash_open();
      bool eeprom_access = ext_flash_read(start_adress+(read_length*page), read_length, flash_read_data_buffer);
      ext_flash_close();

      if(!eeprom_access)
      {
         printf("SPIFLASH: Error - Could not read EEPROM\n");
      }
      else
      {
         if (mode == HEXVIEW_MODE)
            hexview_print(read_length, flash_read_data_buffer, start_adress+(read_length*page));
         if (mode == HEXRAW_MODE)
            hexraw_print(read_length, flash_read_data_buffer);
      }
   }
   printf("\nSPIFLASH DAMP END \n");

}

/*---------------------------------------------------------------------------*/

void
hexraw_print(uint32_t flash_length, uint8_t *flash_read_data_buffer)
{
   for (uint32_t i = 0; i < flash_length; i++)
   {
         printf("%"PRIXX8, flash_read_data_buffer[i]);
   }
}

/*---------------------------------------------------------------------------*/

void
hexview_print(uint32_t flash_length, uint8_t *flash_read_data_buffer, uint32_t offset)
{

   for (uint32_t i = 0; i < flash_length; i = i + 16)
   {
      printf("0x%"PRIXX32": ", i + offset);
      for (int i2 = 0; i2 < 16; i2++)
      {
         printf("%"PRIXX8" ", flash_read_data_buffer[i2+i]);
         if (i2 == 7) { printf(" "); }
      }
      printf("\n");
   }

}

/*---------------------------------------------------------------------------*/

uint16_t crc16_arc(uint8_t *data, uint16_t len)
{
   uint16_t crc = 0x0000;
   uint16_t j;
   int i;
   // Note: 0xA001 is the reflection of 0x8005
   for (j = len; j > 0; j--)
   {
      crc ^= *data++;
      for (i = 0; i < 8; i++)
      {
         if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
         else
            crc >>= 1;
      }
   }
   return (crc);
}

/*---------------------------------------------------------------------------*/

uint8_t get_voltage()
{
   return (uint8_t)((((batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT) * 125) >> 5)-2000)/50);
}

/*---------------------------------------------------------------------------*/

uint8_t get_temperature()
{
   uint32_t temp_offset = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP)+50;
   uint8_t temp_offset_u8 = (uint8_t)temp_offset;
   return temp_offset_u8;
}

/*---------------------------------------------------------------------------*/

uint8_t get_parent_rssi()
{
   const rpl_dag_t *dag = NULL;
   const struct link_stats *stat_parent = NULL;
   dag = rpl_get_any_dag();
   if (dag != NULL)
   {
      stat_parent = rpl_get_parent_link_stats(dag->preferred_parent);
      if (stat_parent != NULL)
      {
         int16_t rssi_int = (stat_parent->rssi)+200;
         uint16_t rssi_uint = (uint16_t)rssi_int;
         uint8_t rssi_uint_8t = (uint8_t)rssi_uint;
         return rssi_uint_8t;
      }
   }

   return 0xFF;
}

/*---------------------------------------------------------------------------*/


str2int_errno_t hex_str2uint16(uint16_t *out, char *s) {
   char *end;
   if (s[0] == '\0' || isspace((unsigned char) s[0]))
       return STR2INT_INCONVERTIBLE;
   errno = 0;
   long l = strtol(s, &end, 16);
   /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
   if (l > 0xFFFF || (errno == ERANGE && l == LONG_MAX))
       return STR2INT_OVERFLOW;
   if (l < 0 || (errno == ERANGE && l == LONG_MIN))
       return STR2INT_UNDERFLOW;
   if (*end != '\0')
       return STR2INT_INCONVERTIBLE;
   *out = (uint16_t)l;
   return STR2INT_SUCCESS;
}

/*---------------------------------------------------------------------------*/


str2int_errno_t hex_str2uint8(uint8_t *out, char *s) {
   char *end;
   if (s[0] == '\0' || isspace((unsigned char) s[0]))
       return STR2INT_INCONVERTIBLE;
   errno = 0;
   long l = strtol(s, &end, 16);
   // Both checks are needed because INT_MAX == LONG_MAX is possible. //
   if (l > 255 || (errno == ERANGE && l == LONG_MAX))
       return STR2INT_OVERFLOW;
   if (l < 0 || (errno == ERANGE && l == LONG_MIN))
       return STR2INT_UNDERFLOW;
   if (*end != '\0')
       return STR2INT_INCONVERTIBLE;
   *out = (uint8_t)l;
   return STR2INT_SUCCESS;
}


/*---------------------------------------------------------------------------*/

str2int_errno_t dec_str2uint8(uint8_t *out, char *s) {
    char *end;
    if (s[0] == '\0' || isspace((unsigned char) s[0]))
        return STR2INT_INCONVERTIBLE;
    errno = 0;
    long l = strtol(s, &end, 10);
    /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
    if (l > 255 || (errno == ERANGE && l == LONG_MAX))
        return STR2INT_OVERFLOW;
    if (l < 0 || (errno == ERANGE && l == LONG_MIN))
        return STR2INT_UNDERFLOW;
    if (*end != '\0')
        return STR2INT_INCONVERTIBLE;
    *out = (uint8_t)l;
    return STR2INT_SUCCESS;
}
