/*
 * Copyright (c) 2014, OpenMote Technologies, S.L.
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
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup Ambient Light Sensor
 * @{
 *
 * \defgroup OPT3001 sensor
 * @{
 *
 * \file
 * Header file for the OPT3001 Ambient Light Sensor
 *
 * \author
 * Manchenko Oleg man4enkoos@gmail.com
 */
/*---------------------------------------------------------------------------*/
#ifndef OPT3001_H_
#define OPT3001_H_
/*---------------------------------------------------------------------------*/
/**
 * @brief Initial OPT3001 address on I2C bus
 */
#define OPT3001_ADDRESS 0x44 // ADDR = GND
/*
#define OPT3001_ADDRESS 0x45 // ADDR = VDD
#define OPT3001_ADDRESS 0x46 // ADDR = SDA
#define OPT3001_ADDRESS 0x47 // ADDR = SCL
*/

/* bytes are swapped */
#define OPT3001_CFG_FC_1    0x0000
#define OPT3001_CFG_FC_2    0x0100
#define OPT3001_CFG_FC_4    0x0200
#define OPT3001_CFG_FC_8    0x0300
#define OPT3001_CFG_MASK    0x0400
#define OPT3001_CFG_POLPOS  0x0800
#define OPT3001_CFG_LATCH   0x1000
#define OPT3001_CFG_FLAGL   0x2000
#define OPT3001_CFG_FLAGH   0x4000
#define OPT3001_CFG_CRF     0x8000
#define OPT3001_CFG_OVF     0x0001
#define OPT3001_CFG_SHDN    0x0000
#define OPT3001_CFG_SHOT    0x0002
#define OPT3001_CFG_CONT    0x0004
#define OPT3001_CFG_100MS   0x0000
#define OPT3001_CFG_800MS   0x0008
#define OPT3001_CFG_RNAUTO  0x00C0

#define OPT3001_CFG (OPT3001_CFG_FC_1 | OPT3001_CFG_SHOT | OPT3001_CFG_100MS | OPT3001_CFG_RNAUTO )
#define OPT3001_CFG_DEFAULT 0x10C8

/**
 * @brief OPT3001 registers
 */
#define OPT3001_REG_RESULT        0x00
#define OPT3001_REG_CONFIG        0x01
#define OPT3001_REG_ID            0x7E
#define OPT3001_CHIP_ID           0x4954
#define OPT3001_REG_CONFIG_MASK   0xFE1F

/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the OPT3001 Ambient Light Sensor
 */
bool opt3001_init(void);
/*---------------------------------------------------------------------------*/
/**
 * \brief Measure SHT21 OPT3001 Ambient Light Sensor
 */
uint32_t opt3001_measure(void);
/*---------------------------------------------------------------------------*/
#endif /* OPT3001_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
