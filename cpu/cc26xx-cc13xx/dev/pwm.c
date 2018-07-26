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
 *         Manchenko Oleg man4enkoos@gmail.com
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

#define IOC_OUTPUT (IOC_CURRENT_8MA    | IOC_STRENGTH_MAX | \
                    IOC_NO_IOPULL      | IOC_SLEW_DISABLE | \
                    IOC_HYST_DISABLE   | IOC_NO_EDGE      | \
                    IOC_INT_DISABLE    | IOC_IOMODE_INV   | \
                    IOC_NO_WAKE_UP     | IOC_INPUT_DISABLE )

/**/										 
static uint8_t settings_pwm = 0;
static uint8_t power_pwm = 0; 

/*---------------------------------------------------------------------------*/
/**/
LPM_MODULE(pwm_module, NULL, NULL, NULL, LPM_DOMAIN_PERIPH);

/*---------------------------------------------------------------------------*/
/*Конфигурироавание канала ШИМ*/
bool pwm_config(uint8_t channel, uint32_t frequency, uint8_t duty, uint32_t pin)
{
	/*Проверяем на количество существующих каналов ШИМ'а*/
	if(channel > 5)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] Invalid channel number\n");
		
		return false;
	}
	
	/*Проверяем на то что частота от 100 Hz до 100 kHz*/
	if((frequency < 100) || (frequency > 100000))
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] The frequency should be in the range from 100 Hz to 100 kHz\n");
		
		return false;
	}
	
	/*Коэффицент заполнения не может быть больше 100 процентов*/
	if(duty > 100)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] Duty cycle cannot be more than 100 percent\n");
		
		return false;		
	}
	
	/*Вычисляем количество тиков за период*/
	uint32_t frequency_tick = GET_MCU_CLOCK / frequency; 
	uint32_t duty_tick = frequency_tick / 100 * duty; 
	
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
		HWREG(GPT_Base + GPT_O_TAMR) &= ~(GPT_TAMR_TACM);
		HWREG(GPT_Base + GPT_O_TAMR) |= (GPT_TAMR_TAAMS_PWM | GPT_TAMR_TAMR_PERIODIC);
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TAPWML_INVERTED);	
		HWREG(GPT_Base + GPT_O_TAPR) = ((frequency_tick & 0x00FF0000) >> 16);
		HWREG(GPT_Base + GPT_O_TAMR) &= ~(GPT_TAMR_TAPWMIE_EN);
		HWREG(GPT_Base + GPT_O_TAILR) = (frequency_tick & 0x0000FFFF);
		HWREG(GPT_Base + GPT_O_TAPMR) = ((duty_tick & 0x00FF0000) >> 16);
		HWREG(GPT_Base + GPT_O_TAMATCHR) = (duty_tick & 0x0000FFFF);
		
	}
	
	/*Timer B*/
	if((channel == 1) || (channel == 3) || (channel == 5))
	{
		HWREG(GPT_Base + GPT_O_TBMR) &= ~(GPT_TBMR_TBCM);
		HWREG(GPT_Base + GPT_O_TBMR) |= (GPT_TBMR_TBAMS_PWM | GPT_TBMR_TBMR_PERIODIC);
		HWREG(GPT_Base + GPT_O_CTL) &= ~(GPT_CTL_TBPWML_INVERTED);
		HWREG(GPT_Base + GPT_O_TBPR) = ((frequency_tick & 0x00FF0000) >> 16);
		HWREG(GPT_Base + GPT_O_TBMR) &= ~(GPT_TBMR_TBPWMIE_EN);
		HWREG(GPT_Base + GPT_O_TBILR) = (frequency_tick & 0x0000FFFF);
		HWREG(GPT_Base + GPT_O_TBPMR) = ((duty_tick & 0x00FF0000) >> 16);
		HWREG(GPT_Base + GPT_O_TBMATCHR) = (duty_tick & 0x0000FFFF);
	}
	
	/*Подключаем ножку микроконтроллера к таймеру*/
	if(channel == 0)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT2, IOC_OUTPUT);
	else if(channel == 1)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT3, IOC_OUTPUT);
	else if(channel == 2)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT4, IOC_OUTPUT);
	else if(channel == 3)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT5, IOC_OUTPUT);
	else if(channel == 4)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT6, IOC_OUTPUT);
	else if(channel == 5)
		ti_lib_ioc_port_configure_set(pin, IOC_PORT_MCU_PORT_EVENT7, IOC_OUTPUT);
	
	/*Отмечаем то что этот канал ШИМ'а настроен*/
	settings_pwm |= (1 << channel);
	
	/*Вывод информационного сообщения в консоль*/
	printf("[PWM] Channel %i is configured: %lu Hz, duty %i percent, %lu pin\n", channel, frequency, duty, pin);
	
	return true;
}

/*---------------------------------------------------------------------------*/
/*Включение канала ШИМ'а*/
bool pwm_start(uint8_t channel)
{
	/*Проверяем на количество существующих каналов ШИМ'а*/
	if(channel > 5)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] Invalid channel number\n");
		
		return false;
	}
	
	/*Проверяем настроен ли канал*/
	if(((settings_pwm >> channel) & 0x01) != 0x01)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] The channel %i is not configured.\n", channel);
		
		return false;	
	}
	
	if(power_pwm == 0)
		lpm_register_module(&pwm_module);
	
	/*Отмечаем то что этот канал ШИМ'а запущен*/
	power_pwm |= (1 << channel);
	
	/*Запускаем канал ШИМ'а*/
	if(channel == 0)
		ti_lib_timer_enable(GPT1_BASE, TIMER_A);
	else if(channel == 1)
		ti_lib_timer_enable(GPT1_BASE, TIMER_B);
	else if(channel == 2)
		ti_lib_timer_enable(GPT2_BASE, TIMER_A);
	else if(channel == 3)
		ti_lib_timer_enable(GPT2_BASE, TIMER_B);
	else if(channel == 4)
		ti_lib_timer_enable(GPT3_BASE, TIMER_A);
	else if(channel == 5)
		ti_lib_timer_enable(GPT3_BASE, TIMER_B);
	
	/*Вывод информационного сообщения в консоль*/
	printf("[PWM] Channel %i is running\n", channel);
	
	return true;
}

/*---------------------------------------------------------------------------*/
/*Выключение канала ШИМ'а*/
bool pwm_stop(uint8_t channel)
{
	/*Проверяем на количество существующих каналов ШИМ'а*/
	if(channel > 5)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] Invalid channel number\n");
		
		return false;
	}
	
	/*Проверяем настроен ли канал*/
	if(((settings_pwm >> channel) & 0x01) != 0x01)
	{
		/*Вывод информационного сообщения в консоль*/
		printf("[PWM] The channel is not configured.\n");
		
		return false;	
	}
	
	/*Останавливаем канал ШИМ'а*/
	if(channel == 0)
		ti_lib_timer_disable(GPT1_BASE, TIMER_A);
	else if(channel == 1)
		ti_lib_timer_disable(GPT1_BASE, TIMER_B);
	else if(channel == 2)
		ti_lib_timer_disable(GPT2_BASE, TIMER_A);
	else if(channel == 3)
		ti_lib_timer_disable(GPT2_BASE, TIMER_B);
	else if(channel == 4)
		ti_lib_timer_disable(GPT3_BASE, TIMER_A);
	else if(channel == 5)
		ti_lib_timer_disable(GPT3_BASE, TIMER_B);
	
	/*Отмечаем то что этот канал ШИМ'а остановлен*/
	power_pwm &= ~(1 << channel);
	
	if(power_pwm == 0)
		lpm_unregister_module(&pwm_module);
	
	/*Вывод информационного сообщения в консоль*/
	printf("[PWM] Channel %i is stopped\n", channel);
	
	return true;
}

/*---------------------------------------------------------------------------*/
/** @} */
