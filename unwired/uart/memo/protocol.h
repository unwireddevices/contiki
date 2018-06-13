



header
payload











/*---------------------------------------------------------------------------*/

typedef union u8_u16_t
{
	uint16_t u16;
	uint8_t u8[2];
} u8_u16_t;

typedef union u8_u32_t
{
	uint32_t u32;
	uint8_t u8[4];
} u8_u32_t;

/*---------------------------------------------------------------------------*/

typedef struct {		
	ciphertext[16];
} crypto_1_block_t;

typedef struct {		
	ciphertext[32];
} crypto_2_block_t;

typedef struct {		
	ciphertext[48];
} crypto_3_block_t;

typedef struct {		
	ciphertext[64];
} crypto_4_block_t;

typedef struct {		
	ciphertext[80];
} crypto_5_block_t;

typedef struct {		
	ciphertext[96];
} crypto_6_block_t;

typedef struct {		
	ciphertext[112];
} crypto_7_block_t;

/*---------------------------------------------------------------------------*/

typedef struct {
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
} header_up_t;

typedef struct {		
    u8_u16_t counter;
	uint8_t length;
} header_down_t;

typedef struct {		
	uint8_t protocol_version;
    uint8_t device_id;
	uint8_t data_type;
	uint8_t rssi;
	uint8_t temperature;
	uint8_t voltage;
	u8_u16_t counter;
	uint8_t length;
} header_t;

typedef struct {		
	u8_u32_t serial;
} join_stage_1_t;

typedef struct {		
	u8_u16_t nonce;
} join_stage_2_t;

typedef struct {		
	u8_u16_t serial;
	u8_u16_t nonce;
} join_stage_3_t;

typedef struct {		
	uint8_t array_of_zeros[16];
} join_stage_4_t;






/*---------------------------------------------------------------------------*/












