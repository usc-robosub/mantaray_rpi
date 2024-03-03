import smbus
import time
import struct

bus = smbus.SMBus(0) # 1 indicates /dev/i2c-1
address = 0x2d # The I2C address of the Teensy device

def writeBigNumber(value, register):
    bus.write_word_data(address, register, int(value))
    return -1

def writeNumber(value, register):
    bus.write_byte_data(address, register, int(value))
    return -1

try:
    while True:
        # Sending a number to Teensy
        num = int(input("Enter a number: "))
        # writeNumber(num)
        if (num%2 == 0):
            print(writeBigNumber(num/10, 0))
        elif (num%10 == 0 or num%10 == 2 or num%10 == 3) :
            print(writeNumber(num/10, num%10))
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped")
