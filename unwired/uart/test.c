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


Available commands:	

address: show net address	
	ADDRESS: not join to net, local address not available
	ADDRESS: node full ipv6 address: FD00::0212:4B00:0C46:8905
	ADDRESS: unwired net address: 02124B000C468905
	//Link layer addr: 00:12:4b:00:0c:46:7c:81
	//Node UD address: 02124B000C467C81
	FD00::0212:4B00:0C46:8A86

bootloader: bootloader start

channel <set/get> <num>: set/get radio channel
	Channel get: Current radio-channel: 26 (2480 MHz)

help: shows this help
	Available commands:
	address: show net address
	bootloader: bootloader start
	channel <set/get> <num>: set/get radio channel
	help: shows this help
	panid <set/get> <panid(ABCD)>: set/get panid
	randwait <maxtime> <command>: wait for a random time before running a command
	reboot: reboot the system
	repeat <num> <time> <command>: run a command every <time> seconds
	serial <set/get> <serial number>: set/get serial number
	status: show node status
	test: test func
	time: show the current node time in unix epoch
	timesync: sync time now
	uptime: show the current node uptime
	
	
panid <set/get> <panid(ABCD)>: set/get panid
	PAN ID: Current ID AABB
	
randwait <maxtime> <command>: wait for a random time before running a command

reboot: reboot the system

repeat <num> <time> <command>: run a command every <time> seconds

serial <set/get> <serial number>: set/get serial number
	Serial: 37622
	
status: show node status
	STATUS: rpl parent ip address: FE80::0212:4B00:0C46:8905
	STATUS: rpl dag root ip address: FD00::0212:4B00:0C46:8905
	STATUS: rpl parent last tx: 6 sec ago
	STATUS: rpl parent rssi: -53
	STATUS: rpl parent is reachable: 1
	STATUS: temp: 21C, voltage: 3320mv

test: test func

time: show the current node time in unix epoch
	RTC: 104 sec, 101 ms

timesync: sync time now

uptime: show the current node uptime
	Uptime: 1 sec, 178 ms


Available commands:

channel <set/get> <num>: set/get radio channel
panid <set/get> <panid(ABCD)>: set/get panid
serial <set/get> <serial number>: set/get serial number



Channel: 26 //uint8_t channel;
PAN ID: 0xAABB //uint16_t panid;
Serial: 37622 //uint32_t serial;
AES128: 11 22 33 44 55 66 77 88 99 00 AA BB CC DD EE FF //uint8_t aes_key[16];

struct eeprom 
{
    uint8_t channel;
    uint16_t panid;
	uint32_t serial;
	uint8_t aes_key[16];
};



00:12:4B:00:0C:46:8A:86 Зашитый в чипе
 Link layer addr: 00:12:4b:00:0c:46:8a:86 Он же
02:12:4B:00:0C:46:8A:86 EUID64????
DAG-root node: FD00::0212:4B00:0C46:8A86 IPv6




00:12:4B - TexasIns	Texas Instruments








 Link layer addr: 00:12:4b:00:0c:46:8d:03
 Node UD address: 02124B000C468D03




void add_route(uint32_t serial, uip_ip6addr_t addr, uint16_t nonce)
{
	if(route_table_ptr >= MAX_ROUTE_TABLE) //Проверка на макс размер таблицы
		return;
	
	for(uint8_t i = 0; i < route_table_ptr; i++) //Проверка есть ли такой серийник
	{
		if(route_table[i].serial == serial)
		{
			//Сокращение адреса
			//Сокращение адреса
			//Сокращение адреса
			//Сокращение адреса
			route_table[i].addr = addr;
			route_table[i].nonce = nonce;
			route_table[i].counter = 0xFFFF;
			//printf("Dont add serial: %lu\n", serial);
			//uip_debug_ipaddr_print(&addr);
			//printf("route_table_ptr: %i\n", route_table_ptr);
			return;
		}
	}
	
	//printf("Add serial: %lu\n", serial);
	//uip_debug_ipaddr_print(&addr);
	//printf("route_table_ptr: %i\n", route_table_ptr);
	route_table[route_table_ptr].serial = serial; //Добавляем в таблицу
	route_table[route_table_ptr].addr = addr;
	route_table[route_table_ptr].nonce = nonce;
	route_table[route_table_ptr].counter = 0xFFFF; //Добавляется в таблицу, но не будет работать, пока счетчик не обнулится
	route_table_ptr++;
}


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 



address: show net address	
	ADDRESS: not join to net, local address not available
	ADDRESS: node full ipv6 address: FD00::0212:4B00:0C46:8905
	ADDRESS: unwired net address: 02124B000C468905
	//Link layer addr: 00:12:4b:00:0c:46:7c:81
	//Node UD address: 02124B000C467C81
	FD00::0212:4B00:0C46:8A86
	
	
void rpl_initialize()
{
	//printf("rpl_initialize\n");
   /* Set MESH-mode for dc-power rpl-root(not leaf-mode) */
   rpl_set_mode(RPL_MODE_MESH);

   static uip_ipaddr_t ipaddr;

   /* Fill in the address with zeros and the local prefix */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

   /* Generate an address based on the chip ID */
   uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);

   /* Adding autoconfigured address as the device address */
   uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

   /* make local address as rpl-root */
   rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
   rpl_dag_t *dag = rpl_get_any_dag();

   uip_ipaddr_t prefix;
   uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
   rpl_set_prefix(dag, &prefix, 64);

   if(uart_status_r() == 0)
	 printf("UDM: Created a new RPL DAG, i'm root!\n");
 
   if(uart_status_r() == 0)
     printf("UDM: Time sync needed\n");
}


//printf("uart_to_air\n");
//if (dest_addr == NULL)
	//return;


//fd00:0000:0000:0000:0212:4b00:0c46:7a01
//uip_ipaddr_t addr = { 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4b, 0x00, 0x0c, 0x46, 0x7a, 0x01 };

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




//if(addr == addr_not_found)
	//return;
										  
//printf("\nSerial: %lu\n", (uint32_t)((udup_v5_rc_uart_rx_buffer[0] << 24) |
//								     (udup_v5_rc_uart_rx_buffer[1] << 16) |
//									 (udup_v5_rc_uart_rx_buffer[2] << 8)  |
//									  udup_v5_rc_uart_rx_buffer[3]));
//uip_debug_ipaddr_print(&addr);
//printf("\n");
//printf("route_table_ptr: %i\n", route_table_ptr);

static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
