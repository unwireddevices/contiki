import serial
import sys
import os

UDBP_PROTOCOL_VERSION = 0x01

#UNWDS_MODULE_IDS
UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 0x7F

#LENGTH#
#define HEADER_UP_LENGTH				sizeof(header_up_t)
#define HEADER_DOWN_LENGTH				sizeof(header_down_t)
#define HEADER_LENGTH 					HEADER_UP_LENGTH + HEADER_DOWN_LENGTH

#define CRYPTO_1_BLOCK_LENGTH 			sizeof(crypto_1_block_t)
#define CRYPTO_2_BLOCK_LENGTH 			sizeof(crypto_2_block_t)
#define CRYPTO_3_BLOCK_LENGTH 			sizeof(crypto_3_block_t)
#define CRYPTO_4_BLOCK_LENGTH 			sizeof(crypto_4_block_t)
#define CRYPTO_5_BLOCK_LENGTH 			sizeof(crypto_5_block_t)
#define CRYPTO_6_BLOCK_LENGTH 			sizeof(crypto_6_block_t)
#define CRYPTO_7_BLOCK_LENGTH 			sizeof(crypto_7_block_t)

#define JOIN_STAGE_1_PAYLOAD_LENGTH 	JOIN_STAGE_1_LENGTH
#define JOIN_STAGE_2_PAYLOAD_LENGTH 	CRYPTO_1_BLOCK_LENGTH
#define JOIN_STAGE_3_PAYLOAD_LENGTH 	CRYPTO_1_BLOCK_LENGTH

#define JOIN_STAGE_1_LENGTH 			sizeof(join_stage_1_t)
#define JOIN_STAGE_2_LENGTH 			sizeof(join_stage_2_t)
#define JOIN_STAGE_3_LENGTH 			sizeof(join_stage_3_t)
#define JOIN_STAGE_4_LENGTH 			sizeof(join_stage_4_t)
#define PING_LENGTH 					sizeof(ping_t)
#define PONG_LENGTH 					sizeof(pong_t)
#define ACK_LENGTH 						sizeof(ack_t)
#define NACK_LENGTH 					sizeof(nack_t)
START_OTA_LENGTH = 16
#define REQ_DATA_FOR_OTA_LENGTH 		sizeof(req_data_for_ota_t)
#define DATA_FOR_OTA_LENGTH 			sizeof(data_for_ota_t)
#define FINISH_OTA_LENGTH  				sizeof(finish_ota_t)
#define BUTTON_STATUS_LENGTH 			sizeof(button_status_t)
#define PWM_SETTINGS_LENGTH 			sizeof(pwm_settings_t)
#define PWM_POWER_LENGTH 				sizeof(pwm_power_t)
#define PWM_SET_LENGTH 					sizeof(pwm_set_t)
#define LIT_MEASURE_LENGTH				0
#define LIT_MEASURE_STATUS_LENGTH		sizeof(lit_measure_status_t)
#define GPIO_CMD_LENGTH					sizeof(gpio_command_t)

#COMMAND
#UNWDS-6LOWPAN_SYSTEM
JOIN_STAGE_1 =      0x00 
JOIN_STAGE_2 =      0x01 
JOIN_STAGE_3 =      0x02 
JOIN_STAGE_4 =      0x03 
PING =              0x04 
PONG =              0x05 
ACK =               0x06 
NACK =              0x07 
START_OTA = 	    0x08 
REQ_DATA_FOR_OTA =  0x09 
DATA_FOR_OTA =      0x0A 
FINISH_OTA = 	    0x0B 

def crc16_arc(data):
    crc = 0
    for j in range(len(data)):
        crc ^= data[j]
        for i in range(8):
            if(crc & 1):
                crc = (crc >> 1) ^ 0xA001 # 0xA001 is the reflection of 0x8005
            else:
                crc >>= 1
        # crc &= 0xFFFF
    return crc & 0xFFFF

def uint16_to_int(uint16_num):
    return ((uint16_num[1] << 8) | uint16_num[0])

def int_to_uint16(int_num):
    return [(int_num & 0x00FF), ((int_num >> 8) & 0x00FF)]

def get_ip_addr_to_str(ip_addr):
    ip_addr_to_str = chr(ip_addr[0]).encode('hex')
    ip_addr_to_str += chr(ip_addr[1]).encode('hex')
    ip_addr_to_str += ':'
    # ip_addr_to_str += chr(ip_addr[2]).encode('hex')
    # ip_addr_to_str += chr(ip_addr[3]).encode('hex')
    # ip_addr_to_str += ':'
    # ip_addr_to_str += chr(ip_addr[4]).encode('hex')
    # ip_addr_to_str += chr(ip_addr[5]).encode('hex')
    # ip_addr_to_str += ':'
    # ip_addr_to_str += chr(ip_addr[6]).encode('hex')
    # ip_addr_to_str += chr(ip_addr[7]).encode('hex')
    ip_addr_to_str += ':'
    ip_addr_to_str += chr(ip_addr[8]).encode('hex')
    ip_addr_to_str += chr(ip_addr[9]).encode('hex')
    ip_addr_to_str += ':'
    ip_addr_to_str += chr(ip_addr[10]).encode('hex')
    ip_addr_to_str += chr(ip_addr[11]).encode('hex')
    ip_addr_to_str += ':'
    ip_addr_to_str += chr(ip_addr[12]).encode('hex')
    ip_addr_to_str += chr(ip_addr[13]).encode('hex')
    ip_addr_to_str += ':'
    ip_addr_to_str += chr(ip_addr[14]).encode('hex')
    ip_addr_to_str += chr(ip_addr[15]).encode('hex')

    return ip_addr_to_str.upper()

def send_pack(dest_addr, device_id, data_type, payload_len, payload):
    pack = []
    pack.extend(dest_addr)
    pack.append(device_id)
    pack.append(data_type)
    pack.extend(int_to_uint16(payload_len))
    pack.extend(payload)
    crc16 = crc16_arc(pack)
    pack.append(crc16 & 0xFF)
    pack.append((crc16 & 0xFF00)>>8)
    
    ### print pack ###
    pack_str = ''
    for i in pack:
        pack_str += '0x{} '.format(chr(i).encode('hex'))
    print(pack_str)

    return ser.write(pack)

def start_ota(dest_addr, ota_metadata):
    return send_pack(dest_addr, UNWDS_6LOWPAN_SYSTEM_MODULE_ID, START_OTA, START_OTA_LENGTH, ota_metadata)

def send_data_for_ota(dest_addr, block):
    data = []
    f = open(sys.argv[1], 'rb')

    offset = block * 256
    f.seek(0, os.SEEK_END)
    size = f.tell()
    if(offset > size):
        return

    block_size = 0
    if((size - offset) >= 256):
        block_size = 256
    else:
        block_size = size - offset

    f.seek(offset)
    for i in range(0, 256):
        if(block_size > i):
            data.append(ord(f.read(1)))
        else:
            data.append(0xFF)

    f.close()

    data_for_ota = []
    data_for_ota.extend(int_to_uint16(block))
    data_for_ota.extend(data)
    
    return send_pack(dest_addr, UNWDS_6LOWPAN_SYSTEM_MODULE_ID, DATA_FOR_OTA, 258, data_for_ota)

if __name__ == "__main__":
    if(len(sys.argv) < 3):
        print("Malo argumentov")
        sys.exit()

    # print(sys.argv[1]) #bin
    # print(sys.argv[2]) #com

    ser = serial.Serial(
        port=sys.argv[2],\
        baudrate=115200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0.01)

    print("[OTA] Connected to: " + ser.portstr)
    print("[OTA] Start")

    ip_addr_ota = [0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4B, 0x00, 0x17, 0xB7, 0xCE, 0xD6]
    ota_metadata = []

    f = open(sys.argv[1], 'rb')
    for i in range(0, 15):
        ota_metadata.append(ord(f.read(1)))
    f.close()

    start_ota(ip_addr_ota, ota_metadata)

    while True:
        pack_str = ""

        pack = ser.read(300)    # read up to 300 bytes (timeout)

        if(len(pack) == 0):
            continue

        if(len(pack) < 28):
            continue

        protocol_version = ord(pack[16])
        device_id = ord(pack[17])
    	data_type = ord(pack[18])
    	rssi = ord(pack[19])
    	temperature = ord(pack[20])
    	voltage = ord(pack[21])
    	counter = ((ord(pack[22]) << 8) | ord(pack[23]))
    	crc = ((ord(pack[24]) << 8) | ord(pack[25]))
        length = ((ord(pack[26]) << 8) | ord(pack[27]))

        #print ip_addr
        ip_addr = []
        for i in range(0, 16):
            ip_addr.append(ord(pack[i]))
        print('[' + get_ip_addr_to_str(ip_addr) + '] '), 

        if(protocol_version == UDBP_PROTOCOL_VERSION):
            if(device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID):
                if(data_type == JOIN_STAGE_1):
                    print("Join stage 1")
                elif(data_type == JOIN_STAGE_3):
                    print("Join stage 3")
                elif(data_type == PING):
                    print("Ping")
                elif(data_type == ACK):
                    print("ACK")
                elif(data_type == NACK):
                    print("NACK")
                elif(data_type == REQ_DATA_FOR_OTA):
                    if(len(pack) <= 29):
                        continue

                    block = uint16_to_int([ord(pack[28]), ord(pack[29])])
                    print('Request ' + str(block) + ' block data for OTA')
                    send_data_for_ota(ip_addr, block)
                elif(data_type == FINISH_OTA):
                    print("FINISH_OTA")
                else:
                    print("Unknown command!")
            else:
                print("Unknown module!")
        else:
            print("Unknown protocol version!")

        # print pack
        # for i in pack:
        #     pack_str += '0x{} '.format(i.encode('hex'))
        # print(pack_str)

        # ser.write(serial.to_bytes([0x4C,0x12,0x01,0x00,0x03,0x40,0xFB,0x02,0x7a]))
        # ser.reset_input_buffer()

    ser.close()