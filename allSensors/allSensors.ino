#include <Wire.h> 
#define BMA180 0x40  //address of the accelerometer
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
int g_offx = 30;
int g_offy = 60;
int g_offz = 30;

int hx, hy, hz, turetemp;

void setup() 
{ 
 Serial.begin(9600); 
 Wire.begin(); 
 Serial.println("Demo started, initializing sensors"); 
 InitializeAccelerometer(); 
 InitializeGyroScope();
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
//
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

void loop() 
{ 
 getAccelerometerData(); 
 getGyroscopeData();
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
