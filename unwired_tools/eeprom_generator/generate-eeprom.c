#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define INTERFACE_RS485 0
#define INTERFACE_CAN 	1

FILE *eeprom_bin; 

typedef  struct {
	uint16_t panid; 		
    uint8_t channel; 				
	uint8_t interface; 				
	uint8_t aes_key[16];		
	uint32_t serial;				
	uint8_t panid_configured;
	uint8_t channel_configured;	
	uint8_t interface_configured;	
	uint8_t aes_key_configured;		
	uint8_t serial_configured;	
}eeprom_t;

//generate-eeprom.exe [panid] [channel] [aes_key] [interface] [serial] 
//generate-eeprom.exe 0xAABB 26 11223344556677889900AABBCCDDEEFF CAN 37622

int main(int argc, char *argv[]) 
{
	if (argc != 6) 
	{
		puts("Wrong number of arguments");
		return -1;
	}
	
	eeprom_t eeprom;
	
	eeprom.panid = (uint16_t)(strtoul(argv[1], NULL, 16));
	eeprom.panid_configured = 0;
	
	eeprom.channel = (uint8_t)(strtoul(argv[2], NULL, 10));
	if(eeprom.channel >= 0 && eeprom.channel < 34)
		eeprom.channel_configured = 0;
	else
	{
		printf("Error channel\n");
		eeprom.channel_configured = 0xFF;
		return -1;
	}
	
	if(strlen(argv[3]) == 32)
	{
		uint8_t *crypto_key = (uint8_t*)(argv[3]);
		uint8_t crypto_byte[3];
		crypto_byte[2] = '\0';
		for(uint8_t i = 0; i < 16; i++)
		{
			crypto_byte[0] = crypto_key[i*2];
			crypto_byte[1] = crypto_key[(i*2)+1];
			eeprom.aes_key[i] = (uint8_t)(strtoul(crypto_byte, NULL, 16));
		}
		eeprom.aes_key_configured = 0;
	}
	else
	{
		printf("Error key\n");
		eeprom.interface_configured = 0xFF;
		return -1;
	}
	
	if(!strncmp(argv[4], "rs485", 5))
	{
		eeprom.interface = INTERFACE_RS485;
		eeprom.interface_configured = 0;
	}
	if(!strncmp(argv[4], "RS485", 5))
	{
		eeprom.interface = INTERFACE_RS485;
		eeprom.interface_configured = 0;
	}
	else if(!strncmp(argv[4], "can", 3))
	{
		eeprom.interface = INTERFACE_CAN;
		eeprom.interface_configured = 0;
	}
	else if(!strncmp(argv[4], "CAN", 3))
	{
		eeprom.interface = INTERFACE_CAN;
		eeprom.interface_configured = 0;
	}
	else
	{
		printf("Unknown interface\n");
		eeprom.interface_configured = 0xFF;
		return -1;
	}
	
	eeprom.serial = (uint32_t)(strtoul(argv[5], NULL, 10));
	if(eeprom.serial >= 0 && eeprom.serial < 1000000)
		eeprom.serial_configured = 0;
	else
	{
		printf("Error serial\n");
		eeprom.serial_configured = 0xFF;
		return -1;
	}

	eeprom_bin = fopen("eeprom.bin", "wb");
	if(eeprom_bin == NULL)
	{
		puts("File error\n");
		return -1;
	}

	fwrite(&eeprom, sizeof(eeprom), 1, eeprom_bin);
	fclose(eeprom_bin);

	printf("EEPROM generation Ok\n");
	return 0;
}
