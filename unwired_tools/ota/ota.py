import serial

ser = serial.Serial(
    port='COM22',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=1)

print("connected to: " + ser.portstr)
count = 1

while True:
    for line in ser.read():

        print(str(count) + str(': ')) #+ chr(line) )
        count = count + 1

ser.close()