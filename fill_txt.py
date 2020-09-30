import serial
import struct

connected = False

locations=["/dev/ttyACM0",'/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']

for test in locations:
    try:
        print("Trying...",test)
        ser = serial.Serial(test, 115200, timeout=1)
        print("ok...", test)
        break
    except:
        print("Failed to connect on",test)

# loop until the arduino tells us it is ready

start = True
text_file = open("operation.txt", 'w')
# ser.write(1)


# while start:
#    ser.write(1)
#    response = ser.read()
#    print("response ", response)
#    if response:
#        print("hola")
#        start = False

while 1:
    buffer = (ser.read(1))
    testResult = struct.unpack('<B', buffer)
    print(testResult[0])

#while ser.read():
    #x=ser.read()
    #print(x)
    #if x=="\0":
   #   text_file.seek(0)
  #    text_file.truncate()
    #text_file.write(x)
 #   text_file.flush()
#text_file.close()
#ser.close()