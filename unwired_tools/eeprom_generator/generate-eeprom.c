#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define COLON 0x3A
#define NULL_S 0x00

FILE *eeprom_bin; 

typedef  struct {
	uint16_t panid; 				//+
    uint8_t channel; 				//+
	uint8_t interface; 				//+
	uint8_t aes_key[16];			//+
	uint32_t serial;				//+-
	uint8_t interface_configured;	//+
	uint8_t aes_key_configured;		//+
	uint8_t serial_configured;		//+-
}eeprom_t;

//generate-eeprom.exe [panid] [channel] [aes_key] [serial] [interface] 
//generate-eeprom.exe 0xAABB 26 11223344556677889900AABBCCDDEEFF 37622 CAN
//BBAA1A0111223344556677889900AABBCCDDEEFFF6920000000000FF

uint8_t char_to_number(uint8_t number)
{
	switch ( number ) 
	{
		case '0':
			return 0x00;
			break;
		case '1':
			return 0x01;
			break;
		case '2':
			return 0x02;
			break;
		case '3':
			return 0x03;
			break;
		case '4':
			return 0x04;
			break;
		case '5':
			return 0x05;
			break;
		case '6':
			return 0x06;
			break;
		case '7':
			return 0x07;
			break;
		case '8':
			return 0x08;
			break;
		case '9':
			return 0x09;
			break;
		case 'a':
			return 0x0a;
			break;
		case 'A':
			return 0x0a;
			break;
		case 'b':
			return 0x0b;
			break;
		case 'B':
			return 0x0b;
			break;
		case 'c':
			return 0x0c;
			break;
		case 'C':
			return 0x0c;
			break;
		case 'd':
			return 0x0d;
			break;
		case 'D':
			return 0x0d;
			break;
		case 'e':
			return 0x0e;
			break;
		case 'E':
			return 0x0e;
			break;
		case 'f':
			return 0x0f;
			break;
		case 'F':
			return 0x0f;
			break;
		default:
			return 0;
			break;
	}
}

int main(int argc, char *argv[]) 
{
	if (argc != 6) 
	{
		puts("Wrong number of arguments");
		return -1;
	}
	
	eeprom_t eeprom;
	
	// uint8_t *mac = (uint8_t*)(argv[1]);

	// uint8_t counter = 0;
	// for(int8_t i = 7; i >= 0; i--)
	// {
		// if(mac[counter] == COLON || mac[counter] == NULL_S)//0
		// {
			// ((uint8_t*)l_hash)[i] = 0;
			// ((uint8_t*)l_hash)[i+8] = 0;
			// ((uint8_t*)l_hash)[i+16] = 0;
			// ((uint8_t*)l_hash)[i+24] = 0;
			// counter++;
		// }
		// else if(mac[counter+1] == COLON || mac[counter+1] == NULL_S)//1
		// {
			// ((uint8_t*)l_hash)[i] = char_to_number(mac[counter]);
			// ((uint8_t*)l_hash)[i+8] = ((uint8_t*)l_hash)[i];
			// ((uint8_t*)l_hash)[i+16] = ((uint8_t*)l_hash)[i];
			// ((uint8_t*)l_hash)[i+24] = ((uint8_t*)l_hash)[i];
			// counter += 2;
		// }
		// else if(mac[counter+2] == COLON || mac[counter+2] == NULL_S)//2
		// {
			// ((uint8_t*)l_hash)[i] = ((char_to_number(mac[counter])<<4) | (char_to_number(mac[counter+1])));
			// ((uint8_t*)l_hash)[i+8] = ((uint8_t*)l_hash)[i];
			// ((uint8_t*)l_hash)[i+16] = ((uint8_t*)l_hash)[i];
			// ((uint8_t*)l_hash)[i+24] = ((uint8_t*)l_hash)[i];
			// counter += 3;
		// }
		// else
		// {
			// printf("Ret\n");
			// return -1;
		// }
	// }
	
	// for(uint8_t i = 0; i < 32; i++)
		// printf("%x ", ((uint8_t*)l_hash)[i]);
	// printf("\n");
	
	// uint8_t output_buffer[sizeof(eeprom)];
	// memcpy(output_buffer, (uint8_t *)&eeprom, sizeof(eeprom));

	eeprom_bin = fopen("eeprom.bin", "wb");
	if(eeprom_bin == NULL)
	{
		puts("File error\n");
		return -1;
	}

	fwrite(&eeprom, sizeof(eeprom), 1, eeprom_bin);
	fclose(eeprom_bin);

	printf("Ok\n");
	return 0;
}
