/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 

void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    accel[0] = (((int16_t) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
    accel[1] = (((int16_t) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
    accel[2] = (((int16_t) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (((int16_t) buff[0]) << 8) | buff[1];
    magnetom[1] = (((int16_t) buff[4]) << 8) | buff[5];
    magnetom[2] = (((int16_t) buff[2]) << 8) | buff[3];
  }
  else
  {
    num_magn_errors++;
  }
}

#include <L3G4200D.h>

L3G4200D gyroscope;

void Gyro_Init()
{
  gyroscope.enableDefault();
  gyroscope.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyroscope.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  gyroscope.read();
  gyro[0] = gyroscope.g.x;
  gyro[1] = gyroscope.g.y;
  gyro[2] = gyroscope.g.z;
}

#include <BMP085.h>

Adafruit_BMP085 press;

void Pressure_Init() {
  press.begin(BMP085_ULTRALOWPOWER);
}

void Read_Pressure() {
  //press.update();
  temperature = press.readTemperature();
  pressure = 0.01f * press.readPressure();
  altitude = press.readAltitude(ALT_SEA_LEVEL_PRESSURE);
}

