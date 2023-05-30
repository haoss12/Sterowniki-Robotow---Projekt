import serial
import time

ser = serial.Serial(
    port='COM8',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)


print("connected to: " + ser.portstr)
f = open("STMACC2.txt", "wb")

while True:
    # input("Dawaj pomiar wariacie... ")
    bytesToRead = ser.inWaiting()
    ser.read(bytesToRead)
    time.sleep(1)
    if ser.inWaiting():
        x = ser.readline()
        print(x)
        
        f.write(x)
        
f.close()
ser.close()