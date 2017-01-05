/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Disable button shutdown functionality */
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN    0 //??????
/*---------------------------------------------------------------------------*/
#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID					0xABCD
#undef RF_CORE_CONF_CHANNEL
#define RF_CORE_CONF_CHANNEL					26

#undef RF_BLE_CONF_ENABLED
#define RF_BLE_CONF_ENABLED						0
#undef STARTUP_CONF_VERBOSE
#define STARTUP_CONF_VERBOSE					1

/* MAC tune option */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC						csma_driver //nullmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC						contikimac_driver //nullrc_driver
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER					framer_802154 //framer_nullmac
#undef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE	8 //in Hz, 2, 4, 8, 16, 32...

/* Encryption */
#undef LLSEC802154_CONF_ENABLED
#define LLSEC802154_CONF_ENABLED          		1
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER              		noncoresec_framer
#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC               		noncoresec_driver
#undef NONCORESEC_CONF_SEC_LVL
#define NONCORESEC_CONF_SEC_LVL           		0x05
#undef LLSEC802154_CONF_SECURITY_LEVEL
#define LLSEC802154_CONF_SECURITY_LEVEL 		0x05
#undef NONCORESEC_CONF_KEY
#define NONCORESEC_CONF_KEY						{0xF3,0x01,0x02,0x03,0x04,0x05,0x07,0x07,0x06,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}

/*---------------------------------------------------------------------------*/

#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/