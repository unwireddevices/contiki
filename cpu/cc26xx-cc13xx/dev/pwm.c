/*
 * Copyright (c) 2015, Unwired Devices- http://www.unwireddevices.com
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
 *
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
/**
 * \addtogroup cc26xx-cc13xx
 * @{
 *
 * \file
 *  Driver for the CC26XX-CC13XX PWM
 *
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "driverlib/prcm.h"
#include "driverlib/ioc.h"
#include "pwm.h"
#include "ti-lib.h"
#include "lpm.h"
#include <stdio.h>
#include <stdlib.h>

/**/										 
typedef struct {		
	uint32_t pin;
	uint32_t frequency;
	uint8_t duty;
	uint8_t state;
} pwm_settings_t;

static pwm_settings_t pwm_channel[6];

/*---------------------------------------------------------------------------*/
LPM_MODULE(pwm_module, NULL, NULL, NULL, LPM_DOMAIN_PERIPH);

/*---------------------------------------------------------------------------*/
void pwm_config(uint8_t channel, uint32_t frequency, uint8_t duty)
{
	
	if((frequency < 100) || (frequency > 100000))
		return;
	
	uint32_t frequency_tick = 48000000 / frequency; //48000
	
	if(duty > 100)
		return;
	
	uint32_t duty_tick = frequency_tick / 100 * duty; //36000
	
	if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) 
	{
		ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
		while(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
	}
	
	uint32_t GPT_Base;
	
	if((channel == 0) || (channel == 1))
	{
		GPT_Base = GPT1_BASE;
		ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER1);
		ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER1);
		ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER1);
	}
	if((channel == 2) || (channel == 3))
	{
		GPT_Base = GPT2_BASE;
		ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER2);
		ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER2);
		ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER2);
	}
	if((channel == 4) || (channel == 5))
	{
		GPT_Base = GPT3_BASE;
		ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER3);
		ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER3);
		ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER3);
	}
	
	ti_lib_prcm_load_set();
	while(!ti_lib_prcm_load_get());
	
	/*PWM Mode*/
	/*Timer A*/
	if((channel == 0) || (channel == 2) || (channel == 4))
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TAEN_EN);
	
	/*Timer B*/
	if((channel == 1) || (channel == 3) || (channel == 5))
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TBEN_EN);
	
	HWREG(GPT_Base + GPT_O_CFG) = GPT_CFG_CFG_16BIT_TIMER;
	
	/*Timer A*/
	if((channel == 0) || (channel == 2) || (channel == 4))
	{
		HWREG(GPT_Base + GPT_O_TAMR) &= ~(GPT_TAMR_TACM);//3
		HWREG(GPT_Base + GPT_O_TAMR) |= (GPT_TAMR_TAAMS_PWM | GPT_TAMR_TAMR_PERIODIC);//3
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TAPWML_INVERTED);//4		
		HWREG(GPT_Base + GPT_O_TAPR) = ((frequency_tick & 0x00FF0000) >> 16);//5
		HWREG(GPT_Base + GPT_O_TAMR) &= ~(GPT_TAMR_TAPWMIE_EN);//6
		HWREG(GPT_Base + GPT_O_TAILR) = (frequency_tick & 0x0000FFFF);//7
		HWREG(GPT_Base + GPT_O_TAPMR) = ((duty_tick & 0x00FF0000) >> 16);//8
		HWREG(GPT_Base + GPT_O_TAMATCHR) = (duty_tick & 0x0000FFFF);//8
		
	}
	
	/*Timer B*/
	if((channel == 1) || (channel == 3) || (channel == 5))
	{
		HWREG(GPT_Base + GPT_O_TBMR) &= ~(GPT_TBMR_TBCM);//3
		HWREG(GPT_Base + GPT_O_TBMR) |= (GPT_TBMR_TBAMS_PWM | GPT_TBMR_TBMR_PERIODIC);//3
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TBPWML_INVERTED);//4
		HWREG(GPT_Base + GPT_O_TBPR) = ((frequency_tick & 0x00FF0000) >> 16);//5
		HWREG(GPT_Base + GPT_O_TBMR) &= ~(GPT_TBMR_TBPWMIE_EN);//6
		HWREG(GPT_Base + GPT_O_TBILR) = (frequency_tick & 0x0000FFFF);//7
		HWREG(GPT_Base + GPT_O_TBPMR) = ((duty_tick & 0x00FF0000) >> 16);//8
		HWREG(GPT_Base + GPT_O_TBMATCHR) = (duty_tick & 0x0000FFFF);//8
	}
	
	if(channel == 0)
		ti_lib_ioc_port_configure_set(IOID_5, IOC_PORT_MCU_PORT_EVENT2, IOC_OUTPUT);
	else if(channel == 1)
		ti_lib_ioc_port_configure_set(IOID_6, IOC_PORT_MCU_PORT_EVENT3, IOC_OUTPUT);
	else if(channel == 2)
		ti_lib_ioc_port_configure_set(IOID_7, IOC_PORT_MCU_PORT_EVENT4, IOC_OUTPUT);
	else if(channel == 3)
		ti_lib_ioc_port_configure_set(IOID_24, IOC_PORT_MCU_PORT_EVENT5, IOC_OUTPUT);
	else if(channel == 4)
		ti_lib_ioc_port_configure_set(IOID_25, IOC_PORT_MCU_PORT_EVENT6, IOC_OUTPUT);
	else if(channel == 5)
		ti_lib_ioc_port_configure_set(IOID_26, IOC_PORT_MCU_PORT_EVENT7, IOC_OUTPUT);
}

/*---------------------------------------------------------------------------*/
// void pwm_config_h(pwm_settings_t *pwm_settings)
// {
	// if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) 
	// {
		// ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
		// while(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
	// }

	// if(pwm_settings->ch == 0 || pwm_settings->ch == 1)
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER1);
	
	// else if(pwm_settings->ch == 2 || pwm_settings->ch == 3)
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER2);
	
	// else if(pwm_settings->ch == 4 || pwm_settings->ch == 5)
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER3);
	
	
	// ti_lib_prcm_load_set();
	// while(!ti_lib_prcm_load_get());
	
	
	// if(pwm_settings->ch == 0 || pwm_settings->ch == 1)
	// {
		// HWREG(GPT1_BASE + GPT_O_CTL) &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);
		// HWREG(GPT1_BASE + GPT_O_CFG) = TIMER_CFG_SPLIT_PAIR >> 24;
		// HWREG(GPT1_BASE + GPT_O_TBMR) = ((TIMER_CFG_B_ONE_SHOT >> 8) & 0xFF) | GPT_TBMR_TBPWMIE;
		// /* Enable GPT0 clocks under active, sleep, deep sleep */
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER1);
		// ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER1);
		// ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER1);
	// }
	
	// else if(pwm_settings->ch == 2 || pwm_settings->ch == 3)
	// {
		// HWREG(GPT2_BASE + GPT_O_CTL) &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);
		// HWREG(GPT2_BASE + GPT_O_CFG) = TIMER_CFG_SPLIT_PAIR >> 24;
		// HWREG(GPT1_BASE + GPT_O_TBMR) = ((TIMER_CFG_B_ONE_SHOT >> 8) & 0xFF) | GPT_TBMR_TBPWMIE;
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER2);
		// ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER2);
		// ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER2);
	// }
	
	// else if(pwm_settings->ch == 4 || pwm_settings->ch == 5)
	// {
		// HWREG(GPT3_BASE + GPT_O_CTL) &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);
		// HWREG(GPT3_BASE + GPT_O_CFG) = TIMER_CFG_SPLIT_PAIR >> 24;
		// HWREG(GPT1_BASE + GPT_O_TBMR) = ((TIMER_CFG_B_ONE_SHOT >> 8) & 0xFF) | GPT_TBMR_TBPWMIE;
		// ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER3);
		// ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER3);
		// ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER3);
	// }
	
	// frequency = pwm_settings->frequency;
    // ti_lib_prcm_load_set();
    // while(!ti_lib_prcm_load_get());
	

    // /* Drive the I/O ID with GPT0 / Timer A */
	// if(pwm_settings->ch == 0)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT2, IOC_OUTPUT);
		// /* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT1_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
		// ti_lib_timer_disable(GPT1_BASE, TIMER_A);
	// }
	
	// else if(pwm_settings->ch == 1)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT3, IOC_OUTPUT);
		/* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT1_BASE + GPT_O_TBMR) = (TIMER_CFG_B_PWM & 0xFF) | GPT_TBMR_TBPWMIE;
		// ti_lib_timer_disable(GPT1_BASE, TIMER_B);
	// }
	
	// else if(pwm_settings->ch == 2)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT4, IOC_OUTPUT);
		// /* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT2_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
		// ti_lib_timer_disable(GPT2_BASE, TIMER_A);
	// }
	
	// else if(pwm_settings->ch == 3)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT5, IOC_OUTPUT);
		/* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT2_BASE + GPT_O_TBMR) = (TIMER_CFG_B_PWM & 0xFF) | GPT_TBMR_TBPWMIE;
		// ti_lib_timer_disable(GPT2_BASE, TIMER_B);
	// }
	
	// else if(pwm_settings->ch == 4)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT6, IOC_OUTPUT);
		// /* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT3_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
		// ti_lib_timer_disable(GPT3_BASE, TIMER_A);
	// }
	
	// else if(pwm_settings->ch == 5)
	// {
		// ti_lib_ioc_port_configure_set(pwm_settings->pin, IOC_PORT_MCU_PORT_EVENT7, IOC_OUTPUT);
		/* GPT0 / Timer A: PWM, Interrupt Enable */
		// HWREG(GPT1_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
		// ti_lib_timer_disable(GPT1_BASE, TIMER_A);
	// }
	
	// uint32_t load = (GET_MCU_CLOCK / frequency);
    // uint32_t match;

    // if (pwm_settings->duty > 100)
        // return;

    // if (pwm_settings->duty == 100)
        // match = load - 1;
    // else
        // match = (load / 100) * pwm_settings->duty;

	
	
	// if(pwm_settings->ch == 0)
	// {
		// ti_lib_timer_load_set(GPT1_BASE, TIMER_A, load);
		// ti_lib_timer_match_set(GPT1_BASE, TIMER_A, match);
	// }
	
	// else if(pwm_settings->ch == 1)
	// {
		// ti_lib_timer_load_set(GPT1_BASE, TIMER_B, load);
		// ti_lib_timer_match_set(GPT1_BASE, TIMER_B, match);
	// }
	
	// else if(pwm_settings->ch == 2)
	// {
		// ti_lib_timer_load_set(GPT2_BASE, TIMER_A, load);
		// ti_lib_timer_match_set(GPT2_BASE, TIMER_A, match);
	// }
	
	// else if(pwm_settings->ch == 3)
	// {
		// ti_lib_timer_load_set(GPT2_BASE, TIMER_B, load);
		// ti_lib_timer_match_set(GPT2_BASE, TIMER_B, match);
	// }
	
	// else if(pwm_settings->ch == 4)
	// {
		// ti_lib_timer_load_set(GPT3_BASE, TIMER_A, load);
		// ti_lib_timer_match_set(GPT3_BASE, TIMER_A, match);
	// }
	
	// else if(pwm_settings->ch == 5)
	// {
		// ti_lib_timer_load_set(GPT3_BASE, TIMER_B, load);
		// ti_lib_timer_match_set(GPT3_BASE, TIMER_B, match);
	// }	
// }
/*---------------------------------------------------------------------------*/
/**/
void pwm_stop(uint8_t channel)
{
	if(channel == 0)
	{
		ti_lib_timer_disable(GPT1_BASE, TIMER_A);
	}
	
	else if(channel == 1)
	{
		ti_lib_timer_disable(GPT1_BASE, TIMER_B);
	}
	
	else if(channel == 2)
	{
		ti_lib_timer_disable(GPT2_BASE, TIMER_A);
	}
	
	else if(channel == 3)
	{
		ti_lib_timer_disable(GPT2_BASE, TIMER_B);
	}
	
	else if(channel == 4)
	{
		ti_lib_timer_disable(GPT3_BASE, TIMER_A);
	}
	
	else if(channel == 5)
	{
		ti_lib_timer_disable(GPT3_BASE, TIMER_B);
	}	
	
    lpm_unregister_module(&pwm_module);
}
/*---------------------------------------------------------------------------*/
/**/
void pwm_start(uint8_t channel)
{
	lpm_register_module(&pwm_module);
	
	if(channel == 0)
	{
		ti_lib_timer_enable(GPT1_BASE, TIMER_A);
	}
	
	else if(channel == 1)
	{
		ti_lib_timer_enable(GPT1_BASE, TIMER_B);
	}
	
	else if(channel == 2)
	{
		ti_lib_timer_enable(GPT2_BASE, TIMER_A);
	}
	
	else if(channel == 3)
	{
		ti_lib_timer_enable(GPT2_BASE, TIMER_B);
	}
	
	else if(channel == 4)
	{
		ti_lib_timer_enable(GPT3_BASE, TIMER_A);
	}
	
	else if(channel == 5)
	{
		ti_lib_timer_enable(GPT3_BASE, TIMER_B);
	}	
}
/*---------------------------------------------------------------------------*/
/** @} */
