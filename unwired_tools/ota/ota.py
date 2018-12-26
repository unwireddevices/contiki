import serial

ser = serial.Serial(
    port='COM22',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=0.1)

print("connected to: " + ser.portstr)
count = 1

while True:
    result = ""
    num = ser.read(300)    # read up to 300 bytes (timeout)

    for i in num:
        result += '0x{} '.format(i.encode('hex'))

    print(result)
    # for line in ser.read(300):

    #     print(str(count) + str(': ')) #+ chr(line) )
    #     count = count + 1

ser.close()