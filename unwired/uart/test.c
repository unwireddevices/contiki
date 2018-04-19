0x0000 0000 FLASHMEM On-Chip Flash 128KB Flash
0x4003 0000 FLASH Flash Controller
0x4003 4000 VIMS Versatile Instruction Memory System Control

4кб - страница
1кб - сектор???

FLASH (RX) : ORIGIN = 0x00000000, LENGTH = 0x0001FFA8

0x00000000 - 0x0001FFA8 FLASH 
0x0001FFA8

0x0001C000 - flash
0x0001E000 - EEPROM


128/4 = 32 страницы
128 секторов


MEMORY
{
    /* Flash Size 128 KB minus the CCA area below (88 bytes) */
    FLASH (RX) : ORIGIN = 0x00000000, LENGTH = 0x0001FFA8

    /*
     * Customer Configuration Area and Bootloader Backdoor configuration
     * in flash, up to 88 bytes
     */
    FLASH_CCFG (RX) : ORIGIN = 0x0001FFA8, LENGTH = 88

    /* RAM Size 20KB */
    SRAM (RWX) : ORIGIN = 0x20000000, LENGTH = 0x00005000
    
    /* Application can use GPRAM region as RAM if cache is disabled in CCFG */
    GPRAM (RWX) : ORIGIN = 0x11000000, LENGTH = 0x00002000
	
}



uint8_t len










static void uart_to_air()
{
	uip_ipaddr_t addr = find_addr((uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
											 (udup_v5_rc_uart_rx_buffer[1] << 16) |
											 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
											  udup_v5_rc_uart_rx_buffer[3]));
											  
	uip_ip6addr_t addr_not_found;
	uip_ip6addr(&addr_not_found, 0, 0, 0, 0, 0, 0, 0, 0);
	
	if((((&addr)->u16[0])  == 0x00)  &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00) &&
		(((&addr)->u16[0])  == 0x00))	
	{	
		return; //Нету такого адреса
	}
	
	uint16_t nonce = get_nonce((uint32_t) ( (data[UDBP_V5_HEADER_LENGTH + 2] << 24) |
											(data[UDBP_V5_HEADER_LENGTH + 3] << 16) |
											(data[UDBP_V5_HEADER_LENGTH + 4] << 8)  |
											(data[UDBP_V5_HEADER_LENGTH + 5] )));
	
	nonce_key[0] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[1] = (uint8_t)(nonce & 0xFF);
	nonce_key[2] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[3] = (uint8_t)(nonce & 0xFF);
	nonce_key[4] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[5] = (uint8_t)(nonce & 0xFF);
	nonce_key[6] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[7] = (uint8_t)(nonce & 0xFF);
	nonce_key[8] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[9] = (uint8_t)(nonce & 0xFF);
	nonce_key[10] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[11] = (uint8_t)(nonce & 0xFF);
	nonce_key[12] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[13] = (uint8_t)(nonce & 0xFF);
	nonce_key[14] = (uint8_t)((nonce >> 8) & 0xFF);
	nonce_key[15] = (uint8_t)(nonce & 0xFF);

	uint8_t payload_length = iterator_to_byte(udup_v5_data_iterator) + 2;
	uint8_t udp_buffer[payload_length + UDBP_V5_HEADER_LENGTH];
	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = packet_counter_root.u8[0];
	udp_buffer[2] = packet_counter_root.u8[1];
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	udp_buffer[7] = UART_FROM_AIR_TO_TX;
	
	//for(uint8_t i = 0; i < udup_v5_data_iterator; i++) /*Копирование из буфера приема UART*/
	//	udp_buffer[i+8] = udup_v5_rc_uart_rx_buffer[i];
		
	if(udup_v5_data_iterator <= 16)
	{
		for(uint8_t i = 0; i < 16; i++)
		{
			if(i < udup_v5_data_iterator)
				aes_bufer_in[i] = udup_v5_rc_uart_rx_buffer[i];
			else
				aes_bufer_in[i] = 0;
		}
		aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_bufer_in, (uint32_t*)(&udp_buffer[8]), AES128_PACKAGE_LENGTH);
	}
	else
	{
		uint8_t uart_data_ptr = 0;
		while((uart_data_ptr + 16) <= udup_v5_data_iterator) //16<=17, 32<=32
		{
			aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&udup_v5_rc_uart_rx_buffer[uart_data_ptr], (uint32_t*)(&udp_buffer[uart_data_ptr+8]), AES128_PACKAGE_LENGTH);
			uart_data_ptr += 16;
		}
		
		if(uart_data_ptr != udup_v5_data_iterator) //16!=17, 32!=32
		{
			for(uint8_t i = 0; i < 16; i++)
			{
				if((uart_data_ptr + i) < udup_v5_data_iterator)
					aes_bufer_in[i] = udup_v5_rc_uart_rx_buffer[uart_data_ptr + i];
				else
					aes_bufer_in[i] = 0;
			}
			aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_bufer_in, (uint32_t*)(&udp_buffer[uart_data_ptr+8]), AES128_PACKAGE_LENGTH);
		}
		
	}
		
	simple_udp_sendto(&udp_connection, udp_buffer, payload_length + UDBP_V5_HEADER_LENGTH, &addr);
	packet_counter_root.u16++;
}


0x00 0x00 0x92 0xF6 0x00 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88 0x99 0xC7 0xEE

static uint8_t iterator_to_byte(uint8_t iterator)
{
	if(iterator <= 16)
		return 16;
	if((iterator > 16) && (iterator <= 32))
		return 32
	if((iterator > 32) && (iterator <= 48))
		return 48
	if((iterator > 48) && (iterator <= 64))
		return 64
	if((iterator > 64) && (iterator <= 80))
		return 80
	if((iterator > 80) && (iterator <= 96))
		return 96
	if((iterator > 96) && (iterator <= 112))
		return 112
	if((iterator > 112) && (iterator <= 128))
		return 128
	return 0;
}




udup_v5_data_iterator


d2 f5 b0 ae f8 6d fd e6 f9 0c 1b 77 32 73 9e 49 
16 a5 d6 ed 09 c9 24 d3 36 63 48 0f 6e 1d 9e f8

00 00 92 f6 00 11 22 33 44 55 66 77 88 99 00 af
ac e6 5c bd 14 7e 11 f5 15 1f f7 64 de 60 72 5a


ac e6 5c bd 14 7e 11 f5 15 1f f7 64 de 60 72 5a
EC 13 EC 13 EC 13 EC 13 EC 13 EC 13 EC 13 EC 13
_______________________________________________
40 F5 B0 AE F8 6D FD E6 F9 0C 1B 77 32 73 9E 49
d2 f5 b0 ae f8 6d fd e6 f9 0c 1b 77 32 73 9e 49







	udp_buffer[0] = UDBP_PROTOCOL_VERSION_V5;
	udp_buffer[1] = UNWDS_6LOWPAN_SYSTEM_MODULE_ID; 
	udp_buffer[2] = DATA_TYPE_JOIN_V5_STAGE_3;
	udp_buffer[3] = get_parent_rssi();
	udp_buffer[4] = get_temperature();
	udp_buffer[5] = get_voltage();

	udp_buffer[6] = packet_counter_node.u8[1];
	udp_buffer[7] = packet_counter_node.u8[0];




`	















	aes_buffer[0] = packet_counter_root.u8[0];//UNWDS_6LOWPAN_SYSTEM_MODULE_ID;
	aes_buffer[1] = packet_counter_root.u8[1];//UART_FROM_AIR_TO_TX;
	aes_buffer[2] = udup_v5_data_iterator;//Длина пакета
	
	for(uint8_t i = 3; i < payload_length; i++)
	{
		if(i < udup_v5_data_iterator)
			aes_buffer[i] = udup_v5_rc_uart_rx_buffer[i-3];
		else
			aes_buffer[i] = 0;
	}
	
	aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[UDBP_V5_HEADER_LENGTH]), payload_length);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	if((udup_v5_data_iterator + 3) <= 16)
	{
		for(uint8_t i = 0; i < 16; i++)
		{
			if(i < udup_v5_data_iterator)
				aes_buffer[i] = udup_v5_rc_uart_rx_buffer[i];
			else
				aes_buffer[i] = 0;
		}
		aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[8]), AES128_PACKAGE_LENGTH);
	}
	else
	{
		uint8_t uart_data_ptr = 0;
		while((uart_data_ptr + 16) <= udup_v5_data_iterator) //16<=17, 32<=32
		{
			aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)&udup_v5_rc_uart_rx_buffer[uart_data_ptr], (uint32_t*)(&udp_buffer[uart_data_ptr+8]), AES128_PACKAGE_LENGTH);
			uart_data_ptr += 16;
		}
		
		if(uart_data_ptr != udup_v5_data_iterator) //16!=17, 32!=32
		{
			for(uint8_t i = 0; i < 16; i++)
			{
				if((uart_data_ptr + i) < udup_v5_data_iterator)
					aes_buffer[i] = udup_v5_rc_uart_rx_buffer[uart_data_ptr + i];
				else
					aes_buffer[i] = 0;
			}
			aes_cbc_encrypt((uint32_t*)aes_key, (uint32_t*)nonce_key, (uint32_t*)aes_buffer, (uint32_t*)(&udp_buffer[uart_data_ptr+8]), AES128_PACKAGE_LENGTH);
		}
		
	}























