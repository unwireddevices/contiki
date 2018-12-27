import serial
import sys

UDBP_PROTOCOL_VERSION = 0x01

#UNWDS_MODULE_IDS
UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 0x7F

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

    pack_ota = [0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4B, 0x00, 0x17, 0xB7, 0xCE, 0xD6, 0x7F, 0x08, 0x10, 0x00, 0x55, 0x1D, 0x55, 0x1D, 0xF4, 0x23, 0x01, 0x00, 0x01, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]
    ser.write(pack_ota)

    # print(type(pack_ota))
    # print(len(pack_ota))
    # print(pack_ota)

    while True:
        pack_str = ""

        pack = ser.read(300)    # read up to 300 bytes (timeout)

        if(len(pack) == 0):
            continue

        if(len(pack) < 27):
            continue

        ip_addr = pack[0:16]
        protocol_version = ord(pack[16])
        device_id = ord(pack[17])
    	data_type = ord(pack[18])
    	rssi = ord(pack[19])
    	temperature = ord(pack[20])
    	voltage = ord(pack[21])
    	counter = ((ord(pack[22]) << 8) | ord(pack[23]))
    	crc = ((ord(pack[24]) << 8) | ord(pack[25]))
        length = ((ord(pack[26]) << 8) | ord(pack[27]))

        # print header
        ip_addr_str = "ip_addr: "
        for i in ip_addr:
            ip_addr_str += '{} '.format(i.encode('hex'))
        print(ip_addr_str)
        # print()

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
                    print("REQ_DATA_FOR_OTA")
                elif(data_type == FINISH_OTA):
                    print("FINISH_OTA")
                else:
                    print("Unknown command!")
            else:
                print("Unknown module!")
        else:
            print("Unknown protocol version!")

        # print pack
        for i in pack:
            pack_str += '0x{} '.format(i.encode('hex'))
        print(pack_str)

        # ser.write(serial.to_bytes([0x4C,0x12,0x01,0x00,0x03,0x40,0xFB,0x02,0x7a]))
        # ser.reset_input_buffer()

    ser.close()