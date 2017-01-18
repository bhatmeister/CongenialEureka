#include <Wire.h>

void setup()
{
  Serial.begin(115200);
  Wire.begin(); 

  Serial.println("Demo started, initializing sensors"); 

  AccelerometerInit();

  Serial.println("Sensors have been initialized");
} 

void AccelerometerInit()
{
  Wire.beginTransmission(0x40); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x10);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10); 

  Wire.beginTransmission(0x40); // address of the accelerometer
  // low pass filter, range settings
  Wire.write(0x0D);
  Wire.write(0x10);
  Wire.endTransmission(); 

  Wire.beginTransmission(0x40); // address of the accelerometer
  Wire.write(0x20); // read from here
  Wire.endTransmission();
  Wire.requestFrom(0x40, 1);
  byte data = Wire.read();
  Wire.beginTransmission(0x40); // address of the accelerometer
  Wire.write(0x20);
  Wire.write(data & 0x0F); // low pass filter to 10 Hz
  Wire.endTransmission(); 

  Wire.beginTransmission(0x40); // address of the accelerometer
  Wire.write(0x35); // read from here
  Wire.endTransmission();
  Wire.requestFrom(0x40, 1);
  data = Wire.read();
  Wire.beginTransmission(0x40); // address of the accelerometer
  Wire.write(0x35);
  Wire.write((data & 0xF1) | 0x04); // range +/- 2g
  Wire.endTransmission();
} 

void AccelerometerRead()
{
  Wire.beginTransmission(0x40); // start transmission to device, address of the accelerometer 
  Wire.write(0x02); // send register address, set read pointer to data
  Wire.endTransmission();
  Wire.requestFrom(0x40, 6); 

  // read in the 3 axis data, each one is 16 bits
  // print the data to terminal
  Serial.print("Accelerometer: X = ");
  short data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.print(" , Y = ");
  data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.print(" , Z = ");
  data = Wire.read();
  data += Wire.read() << 8;
  Serial.print(data);
  Serial.println();
} 

void loop()
{
  AccelerometerRead();

  delay(500); // slow down output
}
