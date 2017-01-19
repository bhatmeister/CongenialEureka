#include <Wire.h> 
#include <HMC5883L.h>

#define BMA180 0x40  //address of the accelerometer
#define BMP085_ADDRESS 0x77 //address of barometer
#define RESET 0x10   
#define PWR 0x0D
#define BW 0X20
#define RANGE 0X35
#define DATA 0x02
#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z
// offsets are chip specific. 
HMC5883L compass;
const unsigned char OSS = 0;  // Oversampling Setting
// Read 1 byte from the BMP085 at 'address'

int g_offx = 30;
int g_offy = 60;
int g_offz = 30;

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

int hx, hy, hz, turetemp;

void setup() 
{ 
 Serial.begin(9600); 
 Wire.begin(); 
 Serial.println("Demo started, initializing sensors"); 
 InitializeAccelerometer(); 
 InitializeGyroScope();
 InitializeMagnetometer();
 InitializeBarometer();
 Serial.println("Sensors have been initialized"); 
} 
//
void InitializeGyroScope()
{

writeTo(GYRO, G_PWR_MGM, 0x00);
writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
writeTo(GYRO, G_INT_CFG, 0x00);
}
void InitializeAccelerometer() 
{ 
 byte temp[1];
 byte temp1;
  //
  writeTo(BMA180,RESET,0xB6);
  //wake up mode
  writeTo(BMA180,PWR,0x10);
  // low pass filter,
  readFrom(BMA180, BW,1,temp);
  temp1=temp[0]&0x0F;
  writeTo(BMA180, BW, temp1);   
  // range +/- 2g 
  readFrom(BMA180, RANGE, 1 ,temp);  
  temp1=(temp[0]&0xF1) | 0x04;
  writeTo(BMA180,RANGE,temp1);
}

void InitializeBarometer()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  
}

void InitializeMagnetometer()
{
   while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0);
}

int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}
void getAccelerometerData() 
{ 
 // read in the 3 axis data, each one is 14 bits 
 // print the data to terminal 
 int n=6;
 byte result[5];
 readFrom(BMA180, DATA, n , result);
 
 int x= (( result[0] | result[1]<<8)>>2) ;
 float x1=x/4096.0;
 Serial.print("ax=");
 Serial.print(x1);
 Serial.print("g"); 
 //
 int y= (( result[2] | result[3]<<8 )>>2);
 float y1=y/4096.0;
 Serial.print(",ay=");
 Serial.print(y1);
 Serial.print("g"); 
 //
 int z= (( result[4] | result[5]<<8 )>>2);
 float z1=z/4096.0;
 Serial.print(",az=");
 Serial.print(z1);
 Serial.println("g"); 
}

void getGyroscopeData()
{
int regAddress = 0x1B;
int gyro[3];
int temp, x, y, z;
byte buff[G_TO_READ];
readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
gyro[0] = ((buff[2] << 8) | buff[3]) + g_offx;
gyro[1] = ((buff[4] << 8) | buff[5]) + g_offy;
gyro[2] = ((buff[6] << 8) | buff[7]) + g_offz;
gyro[3] = (buff[0] << 8) | buff[1]; // temperature
//byte addr;

hx = gyro[0] / 14.375;
hy = gyro[1] / 14.375;
hz = gyro[2] / 14.375;
Serial.print(" GX=");
Serial.print(hx);
Serial.print(" GY=");
Serial.print(hy);
Serial.print(" GZ="); 
Serial.print(hz);
}


void getMagnetometerData()
{
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);

  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  float headingDegrees = heading * 180/M_PI; 

  Serial.print("Heading = ");
  Serial.print(heading);
  Serial.print("\tDegress = ");
  Serial.print(headingDegrees);
  Serial.println();
}

void getBarometerData()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  unsigned int ut;
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  writeTo(BMP085_ADDRESS,0xF4,0x34 + (OSS<<6)) ;
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  Serial.print("Pressure: ");
  Serial.print(up, DEC);
  Serial.print('\t');

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  writeTo(BMP085_ADDRESS, 0xF4, 0x2E) ;
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  
  Serial.print("Temperature: ");
  Serial.print(ut, DEC);
  Serial.print('\t');
  
}

void loop() 
{ 
 getAccelerometerData(); 
 getGyroscopeData();
 getBarometerData();
 getMagnetometerData();
 
 Serial.print("\t\t");
 delay(500); // slow down output   
}





//
//---------------- Functions--------------------
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) 
{
  Wire.beginTransmission(DEVICE);   //start transmission to ACC
  Wire.write(address);               //send register address
  Wire.write(val);                   //send value to write
  Wire.endTransmission();           //end trnsmisson
}
//reads num bytes starting from address register in to buff array
 void readFrom(int DEVICE, byte address , int num ,byte buff[])
 {
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.write(address);            //send reguster address
 Wire.endTransmission();        //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE,num);  //request 6 bits from ACC
 
 int i=0;
 while(Wire.available())        //ACC may abnormal
 {
 buff[i] =Wire.read();        //receive a byte
 i++;
 }
 Wire.endTransmission();         //end transmission
 }
