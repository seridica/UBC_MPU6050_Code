#include "MPU6050_Interface.h"
#include "MPU6050_defines.h"
#include "User_Types.h"
#include <Wire.h>
#include "SD_Interface.h"

void ReadSensor(imu_data_t* sample) {
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  int error;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);  // request a total of 14 registers
  sample->x_accel=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  sample->y_accel=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  sample->z_accel=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int16_t Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  sample->x_gyro=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  sample->y_gyro=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  sample->z_gyro=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Conversion for the accelerometer from bits to m/s^2
double ConvertAccel( int16_t inbits ) {
  double outValue = 0.0;
  double g = 9.81;
  switch (MPU_ACCEL_SET) {
      case MPU6050_AFS_SEL_0:
        outValue = (double)( inbits ) / 32767.0 * 2.0 * g;
        break;
      case MPU6050_AFS_SEL_1:
        outValue = (double)( inbits ) / 32767.0 * 4.0 * g;
        break;
      case MPU6050_AFS_SEL_2:
        outValue = (double)( inbits ) / 32767.0 * 8.0 * g;
        break;
      case MPU6050_AFS_SEL_3:
        outValue = (double)( inbits ) / 32767.0 * 16.0 * g;
        break;
  }
  return outValue;
}

// Conversion for the gyroscope from bits to rad/s
double ConvertGyro( int16_t inbits ) {
  double outValue = 0.0;
  double deg2rad = 3.1415 / 180.0;
  switch (MPU_ACCEL_SET) {
      case MPU6050_AFS_SEL_0:
        outValue = (double)( inbits ) / 32767.0 * 250.0 * deg2rad;
        break;
      case MPU6050_AFS_SEL_1:
        outValue = (double)( inbits ) / 32767.0 * 500.0 * deg2rad;
        break;
      case MPU6050_AFS_SEL_2:
        outValue = (double)( inbits ) / 32767.0 * 1000.0 * deg2rad;
        break;
      case MPU6050_AFS_SEL_3:
        outValue = (double)( inbits ) / 32767.0 * 2000.0 * deg2rad;
        break;
  }
  return outValue;
}

// Record Samples for calibration
/*
void CalibrationLoop(SdFat *SD) {
      int incomingByte = 0;
      Serial.println( "Place Subject z-up" );
      
      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      record_file( SD, "Calibration_Z.csv", 1000, 100000 );
      
      Serial.println( "Place Subject x-up" );

      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      record_file( SD, "Calibration_X.csv", 1000, 100000 );
      
      Serial.println( "Place Subject y-up" );
      
      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      record_file( SD, "Calibration_Y.csv", 1000, 100000 );
      
      Serial.println( "Calibration Complete" );
}
*/


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
