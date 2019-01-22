#include "MPU6050_Interface.h"
#include "MPU6050_defines.h"
#include "User_Types.h"
#include <Wire.h>
#include "SD_Interface.h"
#include <math.h>

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

// Load Calibration files
void LoadCalibration(SdFat *sd, calibration_t *OutputCalib) {
  
  SdBaseFile calibFile;
  if (!calibFile.open("Calibration_Z", O_RDWR)) {
    Serial.println();
    Serial.println(F("No Calibration File Found"));
    return;
  }

  calibFile.rewind();
  int32_t x_val = 0;
  int32_t y_val = 0;
  int32_t z_val = 0;
  uint32_t counter = 0;
  imu_sample_t data_sample;
  while (calibFile.read(&data_sample, sizeof(imu_sample_t)) == sizeof(imu_sample_t)) {
      x_val += data_sample.sample.x_accel;
      y_val += data_sample.sample.y_accel;
      z_val += data_sample.sample.z_accel;
      counter++;
  }

  double z_vec[3] = { (double)x_val / (double)counter, (double)y_val / (double)counter, (double)z_val / (double) counter };
  double vec_mag = sqrt( z_vec[0]*z_vec[0] + z_vec[1]*z_vec[1] + z_vec[2]*z_vec[2] );
  z_vec[0] /= vec_mag;
  z_vec[1] /= vec_mag;
  z_vec[2] /= vec_mag;
  calibFile.close();

  if (!calibFile.open("Calibration_X", O_RDWR)) {
    Serial.println();
    Serial.println(F("No Calibration File Found"));
    return;
  }

  calibFile.rewind();
  x_val = 0;
  y_val = 0;
  z_val = 0;
  counter = 0;
  while (calibFile.read(&data_sample, sizeof(imu_sample_t)) == sizeof(imu_sample_t)) {
      x_val += data_sample.sample.x_accel;
      y_val += data_sample.sample.y_accel;
      z_val += data_sample.sample.z_accel;
      counter++;
  }

  double x_vec[3] = { (double)x_val / (double)counter, (double)y_val / (double)counter, (double)z_val / (double) counter };
  vec_mag = sqrt( x_vec[0]*x_vec[0] + x_vec[1]*x_vec[1] + x_vec[2]*x_vec[2] );
  x_vec[0] /= vec_mag;
  x_vec[1] /= vec_mag;
  x_vec[2] /= vec_mag;
  calibFile.close();
  
  if (!calibFile.open("Calibration_Y", O_RDWR)) {
    Serial.println();
    Serial.println(F("No Calibration File Found"));
    return;
  }

  calibFile.rewind();
  x_val = 0;
  y_val = 0;
  z_val = 0;
  counter = 0;
  while (calibFile.read(&data_sample, sizeof(imu_sample_t)) == sizeof(imu_sample_t)) {
      x_val += data_sample.sample.x_accel;
      y_val += data_sample.sample.y_accel;
      z_val += data_sample.sample.z_accel;
      counter++;
  }

  double y_vec[3] = { (double)x_val / (double)counter, (double)y_val / (double)counter, (double)z_val / (double) counter };
  vec_mag = sqrt( y_vec[0]*y_vec[0] + y_vec[1]*y_vec[1] + y_vec[2]*y_vec[2] );
  y_vec[0] /= vec_mag;
  y_vec[1] /= vec_mag;
  y_vec[2] /= vec_mag;
  calibFile.close();

  /*
   * RIGHT NOW I JUST USE THE CROSS PRODUCT METHOD FOR FINDING FRAMES. WOULD LIKE TO GET SVD WORKING HERE IN THE FUTURE
   */
  OutputCalib->z_row[0] = z_vec[0];
  OutputCalib->z_row[1] = z_vec[1];
  OutputCalib->z_row[2] = z_vec[2];
  OutputCalib->y_row[0] = z_vec[1]*x_vec[2] - z_vec[2]*x_vec[1];
  OutputCalib->y_row[1] = z_vec[2]*x_vec[0] - z_vec[0]*x_vec[2];
  OutputCalib->y_row[2] = z_vec[0]*x_vec[1] - z_vec[1]*x_vec[0];
  OutputCalib->x_row[0] = OutputCalib->y_row[1]*z_vec[2] - OutputCalib->y_row[2]*z_vec[1];
  OutputCalib->x_row[1] = OutputCalib->y_row[2]*z_vec[0] - OutputCalib->y_row[0]*z_vec[2];
  OutputCalib->x_row[2] = OutputCalib->y_row[0]*z_vec[1] - OutputCalib->y_row[1]*z_vec[0];

  return;
}

void ComputeHeadPitch( calibration_t *CalibStruct, imu_data_t* record_sample, headrp_t *head_rp )
{
  double invec[3] = {(double)record_sample->x_accel, (double)record_sample->y_accel, (double)record_sample->z_accel};
  double magvec = sqrt( invec[0]*invec[0] + invec[1]*invec[1] + invec[2]*invec[2] );
  invec[0] /= magvec;
  invec[1] /= magvec;
  invec[2] /= magvec;

  double newvec[3];
  newvec[0] = invec[0]*CalibStruct->x_row[0] + invec[1]*CalibStruct->x_row[1] + invec[2]*CalibStruct->x_row[2];
  newvec[1] = invec[0]*CalibStruct->y_row[0] + invec[1]*CalibStruct->y_row[1] + invec[2]*CalibStruct->y_row[2];
  newvec[2] = invec[0]*CalibStruct->z_row[0] + invec[1]*CalibStruct->z_row[1] + invec[2]*CalibStruct->z_row[2];

  //head_rp->headpitch = ( atan2(sqrt( newvec[0]*newvec[0] + newvec[1]*newvec[1] ), -newvec[2]) * 180. / 3.1415 );
  //head_rp->headroll = ( atan2( newvec[1], newvec[0] ) * 180. / 3.1415 );
  head_rp->headpitch = atan2( -newvec[0], newvec[2] ) * 180. / 3.1415;
  head_rp->headroll = atan2( newvec[1], sqrt( newvec[0]*newvec[0] + newvec[2]*newvec[2] ) ) * 180. / 3.1415;
  return;
}

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
