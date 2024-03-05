import smbus
import time
import struct

bus = smbus.SMBus(0) # 0 indicates /dev/i2c-0
address = 0x2d # The I2C address of the Teensy device

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def readString(register):
    data = bus.read_i2c_block_data(address, register, 16) # Read a block of up to 16 bytes
    result = ''.join(chr(i) for i in data)
    return result

def readData(_data):
    data = bus.read_byte_data(address, _data) # Read a block of up to 16 bytes
    print(data)
    # result = ''.join(chr(i) for i in data)
    # print(result)
    return data

def receive_float(command, size=4):
    # Assuming a function to read 'size' bytes from I2C into 'data'
    print("turning back into float")
    data = bus.read_i2c_block_data(address, command, size)
    data = struct.unpack('f', bytearray(data))[0]
    print(data)
    return data

try:
    while True:
        # Sending a number to Teensy
        num = int(input("Enter a number: "))
        # writeNumber(num)
        if (num == 5 or num == 9 or num == 13 or num == 17) :
            print(receive_float(num))
        else:
            readData(num)
        print(f"Sent {num}")
        time.sleep(0.1)

        # Receiving string from Teensy
        # print("Receiving from Teensy:")
        # print(readString())
        # time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped")
