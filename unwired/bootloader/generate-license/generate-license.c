#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ecc.h"

FILE *license_bin; // metadata output .bin file

typedef  struct {   	//128
	EccPoint l_public; 	//64
	uint32_t r[8]; 		//32
	uint32_t s[8]; 		//32
}license_t;

int
main(int argc, char *argv[]) {
	// if ( !argv[1] ) 
	// {
		// printf("Please provide a .bin file to compute the CRC on as the first argument.\n");
		// return -1;
	// }

	// if ( !argv[2] ) 
	// {
		// printf("Please provide a 16-bit hex firmware version integer as the second argument.\n");
		// return -1;
	// }

	// if ( !argv[3] ) 
	// {
		// printf("Please provide a 32-bit hex UUID integer as the third argument.\n");
		// return -1;
	// }

	// if ( !argv[4] ) 
	// {
		// printf("Please provide 0 or 1 to indicate whether this image is pre-verified or not as the fourth argument.\n");
		// return -1;
	// }
	
	license_t license;
	
	EccPoint l_public;
    uint32_t l_private[NUM_ECC_DIGITS];
	uint32_t l_hash[NUM_ECC_DIGITS];
	uint32_t l_random[NUM_ECC_DIGITS];
	uint32_t r[NUM_ECC_DIGITS];
    uint32_t s[NUM_ECC_DIGITS];
	
	//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
	l_public.x[0] = 0x2FDEBE6C;
	l_public.x[1] = 0xA47719F1;
	l_public.x[2] = 0x99034867;
	l_public.x[3] = 0xF3698B3E;
	l_public.x[4] = 0x3E5E1334;
	l_public.x[5] = 0x6D54D442;
	l_public.x[6] = 0x18D0D2DC;
	l_public.x[7] = 0x6FF51D33;
	
	//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
	l_public.y[0] = 0x0C36BF5C;
	l_public.y[1] = 0x06FCC1EB;
	l_public.y[2] = 0x14129CC5;
	l_public.y[3] = 0xCC769E3E;
	l_public.y[4] = 0xA4D8C08B;
	l_public.y[5] = 0x9B96F6A5;
	l_public.y[6] = 0x47374183;
	l_public.y[7] = 0xAEF2B7CD;
	
	//d = 24 B8 50 94 0A 9B 79 02 14 D7 7E 95 D4 0B 11 49 5A 30 D5 80 CF 8C 39 0B BC 09 F1 BC FA 10 88 06
	l_private[0] = 0xFA108806;
	l_private[1] = 0xBC09F1BC;
	l_private[2] = 0xCF8C390B;
	l_private[3] = 0x5A30D580;
	l_private[4] = 0xD40B1149;
	l_private[5] = 0x14D77E95;
	l_private[6] = 0x0A9B7902;
	l_private[7] = 0x24B85094;

	//hash = 00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 	
	l_hash[0] = 0x0c468a86;
	l_hash[1] = 0x00124b00;
	l_hash[2] = 0x0c468a86;
	l_hash[3] = 0x00124b00;
	l_hash[4] = 0x0c468a86;
	l_hash[5] = 0x00124b00;
	l_hash[6] = 0x0c468a86;
	l_hash[7] = 0x00124b00;
	
	//random = 48 7A 0D 38 75 BD B8 12 9A 13 3F 00 5B 81 EC 32 F6 FB A3 06 5B C9 EE 44 06 0B 6E D5 73 53 83 2C
	l_random[0] = 0x7353832C;
	l_random[1] = 0x060B6ED5;
	l_random[2] = 0x5BC9EE44;
	l_random[3] = 0xF6FBA306;
	l_random[4] = 0x5B81EC32;
	l_random[5] = 0x9A133F00;
	l_random[6] = 0x75BDB812;
	l_random[7] = 0x487A0D38;
	
	//
	if(!ecdsa_sign(r, s, l_private, l_random, l_hash))
		printf("ecdsa_sign() failed\n");
	else
		printf("ecdsa_sign() ok\n");
	//
	
	//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
	license.l_public.x[0] = 0x2FDEBE6C;
	license.l_public.x[1] = 0xA47719F1;
	license.l_public.x[2] = 0x99034867;
	license.l_public.x[3] = 0xF3698B3E;
	license.l_public.x[4] = 0x3E5E1334;
	license.l_public.x[5] = 0x6D54D442;
	license.l_public.x[6] = 0x18D0D2DC;
	license.l_public.x[7] = 0x6FF51D33;
	
	//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
	license.l_public.y[0] = 0x0C36BF5C;
	license.l_public.y[1] = 0x06FCC1EB;
	license.l_public.y[2] = 0x14129CC5;
	license.l_public.y[3] = 0xCC769E3E;
	license.l_public.y[4] = 0xA4D8C08B;
	license.l_public.y[5] = 0x9B96F6A5;
	license.l_public.y[6] = 0x47374183;
	license.l_public.y[7] = 0xAEF2B7CD;	
	
	//r = 87 AE C4 33 E5 AC 57 32 69 E3 48 09 F3 6E D7 2D 2A DC 71 5B 1A 79 6B 4B 47 0A AC 17 54 4D 75 96	
	license.r[0] = r[0];
	license.r[1] = r[1];
	license.r[2] = r[2];
	license.r[3] = r[3];
	license.r[4] = r[4];
	license.r[5] = r[5];
	license.r[6] = r[6];
	license.r[7] = r[7];
	
	//s = D9 8F D4 43 D2 CC 7C B9 AB 85 88 8A 9A 14 05 E0 EE 4D DF ED E4 85 69 26 EE 04 E8 5B 06 3A 7D 56
	license.s[0] = s[0];
	license.s[1] = s[1];
	license.s[2] = s[2];
	license.s[3] = s[3];
	license.s[4] = s[4];
	license.s[5] = s[5];
	license.s[6] = s[6];
	license.s[7] = s[7];
	
	
	
	
	
	
	//r = 87 AE C4 33 E5 AC 57 32 69 E3 48 09 F3 6E D7 2D 2A DC 71 5B 1A 79 6B 4B 47 0A AC 17 54 4D 75 96	
	// license.r[0] = 0x544D7596;
	// license.r[1] = 0x470AAC17;
	// license.r[2] = 0x1A796B4B;
	// license.r[3] = 0x2ADC715B;
	// license.r[4] = 0xF36ED72D;
	// license.r[5] = 0x69E34809;
	// license.r[6] = 0xE5AC5732;
	// license.r[7] = 0x87AEC433;
	
	//s = D9 8F D4 43 D2 CC 7C B9 AB 85 88 8A 9A 14 05 E0 EE 4D DF ED E4 85 69 26 EE 04 E8 5B 06 3A 7D 56
	// license.s[0] = 0x063A7D56;
	// license.s[1] = 0xEE04E85B;
	// license.s[2] = 0xE4856926;
	// license.s[3] = 0xEE4DDFED;
	// license.s[4] = 0x9A1405E0;
	// license.s[5] = 0xAB85888A;
	// license.s[6] = 0xD2CC7CB9;
	// license.s[7] = 0xD98FD443;
	
	uint8_t output_buffer[ sizeof(license_t) ];
	memcpy( output_buffer, (uint8_t *)&license, sizeof(license_t) );

	license_bin = fopen( "licence.bin", "wb" );

	fwrite(output_buffer, sizeof(output_buffer), 1, license_bin);
	fclose( license_bin );

	return 0;
}
