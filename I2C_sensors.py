import smbus
import time
import ctypes
from struct import pack, unpack

BMA180 = 0x40  #address of the accelerometer
RESET = 0x10
PWR = 0x0D
BW = 0x20
RANGE = 0X35
DATA = 0x02

def signConversion(number):
        binary_value = bin(number)
        
        sign = 1
        if(len(binary_value)>=16):
            if(binary_value[2]=='1'):
                    sign = -1
        else:
            return number
        
        xor_operand = 2**(len(binary_value)-2)-1

        xor_result  = number^xor_operand
        xor_result  += 1
        
        return(xor_result*sign)

def AccelerometerInit():
    bus.write_byte_data(BMA180,RESET,0xB6);
    #wake up mode
    
    bus.write_byte_data(BMA180,PWR,0x10);
    #low pass filter,
    temp = bus.read_byte_data(BMA180,BW);
    temp1=temp&0x0F;

    bus.write_byte_data(BMA180, BW, temp1);
    #range +/- 2g
    
    temp=0
    while (temp==0):
        temp = bus.read_byte_data(BMA180,RANGE)
    
    temp1=(temp&0xF1) | 0x04;
    bus.write_byte_data(BMA180,RANGE,temp1);


def AccelerometerRead():

     #read in the 3 axis data, each one is 14 bits
     #print the data to terminal
     i = 0
     #result = []
     result = bus.read_i2c_block_data(BMA180,DATA)

     x= signConversion((result[0] | result[1]<<8)>>2);
     x1=x/4096.0;

     y= signConversion(( result[2] | result[3]<<8)>>2);
     y1=y/4096.0;

     z= signConversion(( result[4] | result[5]<<8)>>2);
     z1=z/4096.0;
     
     print "x=" + str(x1) + "g  " + "y=" + str(y1) + "g  " + "z=" + str(z1) + "g"
     time.sleep(0.3)


# 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
bus = smbus.SMBus(1)    

print "Demo started, initializing sensors"
AccelerometerInit()
print "Sensors have been initialized"

while(1):
    AccelerometerRead()
