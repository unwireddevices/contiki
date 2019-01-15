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
 *         Bootloader for Unwired Devices mesh system
 * \author
 *         Vladislav Zaytsev vvzvlad@gmail.com vz@unwds.com
 *         Manchenko Oleg man4enkoos@gmail.com
 *
 */
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include "ti-lib.h"
#include <stdbool.h>

#include "ota-bootloader.h"
#include "ecc.h"
#include "board.h"

#define UART_TX_IOID     BOARD_IOID_UART_SHELL_TX
#define UART_SPEED       115200
#define LED_IOID         BOARD_IOID_LED
#define MAC_15_4  		 0x500012F0

#define START_LICENSE_FLASH 	0x1D000
#define END_USER_FLASH			USER_FLASH_LENGTH + START_USER_FLASH

typedef  struct {   	
	EccPoint l_public;
	uint32_t r[8];
	uint32_t s[8];
}license_t;

void
hexraw_print(uint32_t flash_length, uint8_t *flash_read_data_buffer)
{
	for (uint32_t i = 0; i < flash_length; i++)
		ti_lib_uart_char_put(UART0_BASE, flash_read_data_buffer[i]);
}

void initialize_peripherals() 
{
	/* Disable global interrupts */
	bool int_disabled = ti_lib_int_master_disable();

	/* Turn on the PERIPH PD */
	ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);

	/* Wait for domains to power on */
	while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

	/* Enable GPIO peripheral */
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

	/* Apply settings and wait for them to take effect */
	ti_lib_prcm_load_set();
	while(!ti_lib_prcm_load_get());

	/* Re-enable interrupt if initially enabled. */
	if(!int_disabled) 
		ti_lib_int_master_enable();
}


void initialize_uart()
{
	ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
	while(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON);
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_UART0);
	ti_lib_prcm_load_set();
	while(!ti_lib_prcm_load_get());
	ti_lib_uart_disable(UART0_BASE);

	ti_lib_ioc_pin_type_gpio_output(UART_TX_IOID);
	ti_lib_gpio_set_dio(UART_TX_IOID);
	ti_lib_ioc_pin_type_uart(UART0_BASE, 
							 IOID_UNUSED, 
							 UART_TX_IOID, 
							 IOID_UNUSED, 
							 IOID_UNUSED);

	ti_lib_uart_config_set_exp_clk(UART0_BASE, 
								   ti_lib_sys_ctrl_clock_get(), 
								   UART_SPEED,
								   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ti_lib_uart_fifo_level_set(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX4_8);
	HWREG(UART0_BASE + UART_O_LCRH) |= UART_LCRH_FEN;
	HWREG(UART0_BASE + UART_O_CTL) = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}

void deinitialize_uart()
{
	while(ti_lib_uart_busy(UART0_BASE));
	ti_lib_ioc_pin_type_uart(UART0_BASE, IOID_UNUSED, IOID_UNUSED, IOID_UNUSED, IOID_UNUSED);
	ti_lib_ioc_pin_type_gpio_input(UART_TX_IOID);
	ti_lib_gpio_clear_event_dio(UART_TX_IOID);

	ti_lib_uart_disable(UART0_BASE);
	ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_UART0);
	ti_lib_prcm_load_set();

	ti_lib_prcm_power_domain_off(PRCM_DOMAIN_SERIAL);
	while(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_OFF);
}

void read_license(uint8_t *pui8DataBuffer, uint32_t ui32Count)
{
	uint32_t old_vims_state = ti_lib_vims_mode_get(VIMS_BASE);
	ti_lib_vims_mode_set(VIMS_BASE, VIMS_MODE_DISABLED);
	
	uint8_t *pui8ReadAddress = (uint8_t *)START_LICENSE_FLASH;
	while (ui32Count--)
		*pui8DataBuffer++ = *pui8ReadAddress++;

	ti_lib_vims_mode_set(VIMS_BASE, old_vims_state);
}

int main(void)
{
	initialize_peripherals();
	initialize_uart();
	print_uart("\n\n/-----------------------------------------------------------/\n");

/*---------------------------------------------------------------------------*/
#ifdef UNWDS_LICENSE 
	uint32_t l_hash[8];  //Считываем MAC 802.15.4 адрес 
	for (uint32_t i = 0; i < 32; i += 8)
	{
		((uint8_t*)(l_hash))[i]   = ((uint8_t*)(MAC_15_4))[0];
		((uint8_t*)(l_hash))[i+1] = ((uint8_t*)(MAC_15_4))[1];
		((uint8_t*)(l_hash))[i+2] = ((uint8_t*)(MAC_15_4))[2];
		((uint8_t*)(l_hash))[i+3] = ((uint8_t*)(MAC_15_4))[3];
		((uint8_t*)(l_hash))[i+4] = ((uint8_t*)(MAC_15_4))[4];
		((uint8_t*)(l_hash))[i+5] = ((uint8_t*)(MAC_15_4))[5];
		((uint8_t*)(l_hash))[i+6] = ((uint8_t*)(MAC_15_4))[6];
		((uint8_t*)(l_hash))[i+7] = ((uint8_t*)(MAC_15_4))[7];
	}
	
	license_t license;
	read_license((uint8_t*)(&license), 128); //Считываем лицензию из флеша (Открытый ключ и подпись)
	
	if(!ecdsa_verify(&license.l_public, l_hash, license.r, license.s))
	{
		print_uart("License: failed\n");
		while(1);
	}
	else
		print_uart("License: ok\n");
#endif
/*---------------------------------------------------------------------------*/
   
	ti_lib_ioc_pin_type_gpio_output(LED_IOID);
	ti_lib_gpio_set_dio(LED_IOID);

	bool spi_status = ext_flash_init();
	if (spi_status == false)
	{
		print_uart_bl("SPI flash not found, jump to main image\n");
		ti_lib_gpio_clear_dio(LED_IOID);
		deinitialize_uart();
		jump_to_image(CURRENT_FIRMWARE << 12);
	}

	uint8_t fw_flag = read_fw_flag();

	print_uart_bl("FW flag: ");
	print_uart_byte(fw_flag);
	print_uart("\n");

	// OTAMetadata_t current_firmware;
	// get_current_metadata( &current_firmware );

	// int8_t verify_result_int = verify_current_firmware(&current_firmware);
	// int8_t verify_result_ota_0 = verify_ota_slot(0);
	// int8_t verify_result_ota_1 = verify_ota_slot(1);
	// int8_t verify_result_ota_2 = verify_ota_slot(2);

	// print_uart_bl("Internal firmware: ");
	// if (verify_result_int == CORRECT_CRC)
	// 	print_uart("correct CRC\n");
	// else if (verify_result_int == NON_CORRECT_CRC)
	// 	print_uart("non-correct CRC\n");

	// print_uart_bl("OTA slot GI: ");
	// if (verify_result_ota_0 == NON_CORRECT_CRC)
	// 	print_uart("non-correct CRC\n");
	// else if (verify_result_ota_0 == NON_READ_FLASH)
	// 	print_uart("non-read flash\n");
	// else if (verify_result_ota_0 == CORRECT_CRC)
	// 	print_uart("correct CRC\n");

	// print_uart_bl("OTA slot 1 update: ");
	// if (verify_result_ota_1 == NON_CORRECT_CRC)
	// 	print_uart("non-correct CRC\n");
	// else if (verify_result_ota_1 == NON_READ_FLASH)
	// 	print_uart("non-read flash\n");
	// else if (verify_result_ota_1 == CORRECT_CRC)
	// 	print_uart("correct CRC\n");

	// print_uart_bl("OTA slot 2 update: ");
	// if (verify_result_ota_2 == NON_CORRECT_CRC)
	// 	print_uart("non-correct CRC\n");
	// else if (verify_result_ota_2 == NON_READ_FLASH)
	// 	print_uart("non-read flash\n");
	// else if (verify_result_ota_2 == CORRECT_CRC)
	// 	print_uart("correct CRC\n");

	ti_lib_gpio_clear_dio(LED_IOID);

	switch(fw_flag) 
	{
		/* Прыжок на основную программу, если процесса обновления нет */
		case FW_FLAG_NON_UPDATE: 
			print_uart_bl("Jump to main image(FW_FLAG_NON_UPDATE)\n\n");
			deinitialize_uart();
			jump_to_image(CURRENT_FIRMWARE << 12);
			break;

		/* Шьем обновление, если у нас флаг новой прошивки */
		case FW_FLAG_NEW_IMG_EXT:
			print_uart_bl("Flash OTA image(NEW_IMG_EXT)\n");
			update_firmware(2);
			print_uart_bl("Set flag to FW_FLAG_NEW_IMG_INT_RST\n");
			write_fw_flag(FW_FLAG_NEW_IMG_INT_RST);
			print_uart_bl("Reboot...\n");
			ti_lib_sys_ctrl_system_reset();
			break;

		/* Прыжок на основную программу после перезагрузки после обновления */
		case FW_FLAG_NEW_IMG_INT_RST:
			print_uart_bl("Set flag to FW_FLAG_NEW_IMG_INT\n");
			write_fw_flag(FW_FLAG_NEW_IMG_INT);
			print_uart_bl("Jump to main image(NEW_IMG_INT)\n\n");
			deinitialize_uart();
			jump_to_image(CURRENT_FIRMWARE << 12);
			break;

		/* Шьем Golden Image, если у нас нет флага подтверждения работы */
		case FW_FLAG_NEW_IMG_INT:
			print_uart_bl("Update error(FW_FLAG_NEW_IMG_INT), flash golden image\n");
			update_firmware(0);
			print_uart_bl("Set flag to ERROR_GI_LOADED\n");
			write_fw_flag(FW_FLAG_ERROR_GI_LOADED);
			print_uart_bl("Reboot...\n");
			ti_lib_sys_ctrl_system_reset();
			break;

		/* Сброс флага после процесса обновления и перезагрузка */
		case FW_FLAG_PING_OK: 
			print_uart_bl("OTA Update ok(PING_OK)\n");
			print_uart_bl("Set flag to FW_FLAG_NON_UPDATE\n");
			write_fw_flag(FW_FLAG_NON_UPDATE); 
			ti_lib_sys_ctrl_system_reset();
			break;

		/* Прыжок на основную программу после неудачного обновления */
		case FW_FLAG_ERROR_GI_LOADED: 
			print_uart_bl("Set flag to FW_FLAG_NON_UPDATE\n");
			write_fw_flag(FW_FLAG_NON_UPDATE); 
			print_uart_bl("Jump to main image(ERROR_GI_LOADED)\n\n");
			deinitialize_uart();
			jump_to_image(CURRENT_FIRMWARE << 12);
			break;

		default:
			write_fw_flag(FW_FLAG_NON_UPDATE);
			print_uart_bl("Jump to main image(FW_FLAG_NON_UPDATE)\n\n");
			deinitialize_uart();
			jump_to_image(CURRENT_FIRMWARE << 12);
			break;
	}

	//  main() *should* never return - we should have rebooted or branched
	//  to other code by now.
	return 0;
}
