import smbus
import time
import ctypes
from struct import pack, unpack
import math
import Adafruit_BMP.BMP085 as BMP085

BMA180 = 0x40  #address of the accelerometer
RESET = 0x10
PWR = 0x0D
BW = 0x20
RANGE = 0X35
DATA = 0x02
GYRO =  0x68
G_SMPLRT_DIV = 0x15
G_DLPF_FS = 0x16
G_INT_CFG = 0x17
G_PWR_MGM =  0x3E
G_TO_READ =  8 # 2 bytes for each axis x, y, z

MAGNETOMETER_ADDRESS = 0x1e

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

def read_word_2c(adr):
    high = bus.read_byte_data(MAGNETOMETER_ADDRESS, adr)
    low = bus.read_byte_data(MAGNETOMETER_ADDRESS, adr+1)
    val = (high << 8) + low
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def spock(num):
    if(num > 2**15):
        bits  = num.bit_length()
        num = 2**bits - num
        num = num - 1
        num = ~num
        return num
    else:
        return num


def InitializeAccelerometer():
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

def InitializeGyroscope():
    bus.write_byte_data(GYRO, G_PWR_MGM, 0x00);
    bus.write_byte_data(GYRO, G_SMPLRT_DIV, 0x07); # EB, 50, 80, 7F, DE, 23, 20, FF
    bus.write_byte_data(GYRO, G_DLPF_FS, 0x1E); # +/- 2000 dgrs/sec, 1KHz, 1E, 19
    bus.write_byte_data(GYRO, G_INT_CFG, 0x00);



def getBarometerData():
    sensor = BMP085.BMP085()

    print 'Temp = {0:0.2f} *C'.format(sensor.read_temperature())
    print 'Pressure = {0:0.2f} Pa'.format(sensor.read_pressure())
    print 'Altitude = {0:0.2f} m'.format(sensor.read_altitude())
    print 'Sealevel Pressure = {0:0.2f} Pa'.format(sensor.read_sealevel_pressure())
    time.sleep(0.2)

def getMagnetometerData():
    bus.write_byte_data(MAGNETOMETER_ADDRESS,0, 0b01110000)
    bus.write_byte_data(MAGNETOMETER_ADDRESS, 1, 0b00100000)
    bus.write_byte_data(MAGNETOMETER_ADDRESS, 2, 0b00000000)

    scale = 0.92

    x_out = read_word_2c(3) * scale
    y_out = read_word_2c(7) * scale
    z_out = read_word_2c(5) * scale

    bearing  = math.atan2(y_out, x_out) 
    if (bearing < 0):
        bearing += 2 * math.pi

    print "Bearing: ", math.degrees(bearing)
    time.sleep(0.2)

def getAccelerometerData():

     #read in the 3 axis data, each one is 14 bits
     #print the data to terminal
     i = 0
     #result = []
     result = bus.read_i2c_block_data(BMA180,DATA)

     x= (result[0] |spock(result[1]<<8))>>2;
     x1=x/4096.0;

     y= ( result[2] | spock(result[3]<<8))>>2;
     y1=y/4096.0;

     z= ( result[4] |spock(result[5]<<8))>>2;
     z1=z/4096.0;
     
     print "x=" + str(x1) + "g  " + "y=" + str(y1) + "g  " + "z=" + str(z1) + "g"
     time.sleep(0.2)

def getGyroscopeData():
    regAddress = 0x1B;
    g_offx = 0
    g_offy = 60
    g_offz = 30
    buff = bus.read_i2c_block_data(GYRO, regAddress)#read the gyro data from the ITG3200
    #print buff
    x = (spock((buff[2] << 8)) | buff[3] )+ g_offx;
    y = (spock((buff[4] << 8)) | buff[5] )+ g_offy;
    z = (spock((buff[6] << 8)) | buff[7] )+ g_offz;

    
    hx = x / 14.375;
    hy = y / 14.375;
    hz = z / 14.375;
    
    print "x=" + str(hx) + "y=" + str(hy) + "z="  + str(hz)
    time.sleep(0.2)

# 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
bus = smbus.SMBus(1)    

print "Demo started, initializing sensors"
InitializeAccelerometer()
InitializeGyroscope()

print "Sensors have been initialized"

while(1):
    try:
     getAccelerometerData()
     getGyroscopeData()
     getMagnetometerData()
     getBarometerData()
    except:
        continue
