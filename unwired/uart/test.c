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


























