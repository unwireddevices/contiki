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
 * \file
 * Driver for the OPT3001 Ambient Light Sensor
 *
 * \author
 * Manchenko Oleg man4enkoos@gmail.com
 */
/*---------------------------------------------------------------------------*/

#include "board-i2c.h"
#include "opt3001.h"

/*---------------------------------------------------------------------------*/

#define DEBUG 0
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/**
 * @brief OPT3001 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param OPT3001 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int opt3001_init(void)
{
	PRINTF("LIT: Init I2C.\n");
	board_i2c_select(BOARD_I2C_INTERFACE_0, OPT3001_ADDRESS);
	
	uint16_t chipid;
		
	if(!board_i2c_read_regs(OPT3001_REG_ID, (uint8_t*)&chipid, 2))
	{
        puts("[opt3001 driver] Error: sensor not found");
        board_i2c_shutdown();
        return -1;
    }
	
    if(chipid != OPT3001_CHIP_ID) 
	{
        puts("[opt3001 driver] Error: ID mismatch");
        board_i2c_shutdown();
        return -1;
    }
	
    //board_i2c_shutdown();
    
    return 0;
}

/*---------------------------------------------------------------------------*/
/**/
// static uint32_t read_sensor(void) {
	// printf("lum> 0\n");
    // uint16_t data;
    // data = OPT3001_CFG;
	
	// board_i2c_write_regs(OPT3001_REG_CONFIG, (uint8_t*)&data, 2);
	// printf("lum> 1\n");
	
    // /* wait till measurement is finished */
    // int i = 100;
    // do {
        // /* 10 ms delay */
		// printf("lum> 2.%i\n", i);
		// clock_delay(CLOCK_SECOND / 10);
        // printf("lum> 3.%i\n", i);
		
		// board_i2c_read_regs(OPT3001_REG_CONFIG, (uint8_t*)&data, 2);
		// printf("lum> 4.%i\n", i);
		
        // if (data & OPT3001_CFG_CRF) 
		// {
            // break;
        // }
        
        // i--;
    // } while (i);
    
    // if (i == 0) 
	// {
        // return 0;
    // }

    // /* read result */
	// board_i2c_read_regs(OPT3001_REG_RESULT, (uint8_t*)&data, 2);
    
    // /* swap bytes, as OPT3001 sends MSB first and I2C driver expects LSB first */
    // data = ((data >> 8) | (data << 8));
	
    // /* calculate lux per LSB */
    // uint8_t exp = (data >> 12) & 0x0F;
    // uint32_t lsb_size_x100 = (1 << exp);
    
    // /* remove 4-bit exponent and leave 12-bit mantissa only */
    // data &= 0x0FFF;
    
    // return (((uint32_t)data * lsb_size_x100) / 100);
// }

/*---------------------------------------------------------------------------*/
/**
 * @brief Gets OPT3001 measure.
 *
 * @param[in] dev pointer to the initialized OPT3001 device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * opt3001_measure_t measure;
 *
 * opt3001_init(opt3001);
 * opt3001_measure(opt3001, &measure)
 */
uint32_t opt3001_measure(void)
{
	// printf("lum> 0\n");
    uint16_t data;
    data = OPT3001_CFG;
	
	board_i2c_write_regs(OPT3001_REG_CONFIG, (uint8_t*)&data, 2);
	printf("lum> 1\n");
	
    /* wait till measurement is finished */
    int i = 100;
    do {
        /* 10 ms delay */
		printf("lum> 2.%i\n", i);
		clock_delay(CLOCK_SECOND / 10);
        printf("lum> 3.%i\n", i);
		
		board_i2c_read_regs(OPT3001_REG_CONFIG, (uint8_t*)&data, 2);
		printf("lum> 4.%i\n", i);
		
        if (data & OPT3001_CFG_CRF) 
		{
            break;
        }
        
        i--;
    } while (i);
    
    if (i == 0) 
	{
        return 0;
    }

    /* read result */
	board_i2c_read_regs(OPT3001_REG_RESULT, (uint8_t*)&data, 2);
    
    /* swap bytes, as OPT3001 sends MSB first and I2C driver expects LSB first */
    data = ((data >> 8) | (data << 8));
	
    /* calculate lux per LSB */
    uint8_t exp = (data >> 12) & 0x0F;
    uint32_t lsb_size_x100 = (1 << exp);
    
    /* remove 4-bit exponent and leave 12-bit mantissa only */
    data &= 0x0FFF;
    
    return (((uint32_t)data * lsb_size_x100) / 100);
	
	
    // uint32_t lum = read_sensor();
	// printf("lum: %lu\n", lum);
	
    // return lum;
}
/*---------------------------------------------------------------------------*/
/** @} */
