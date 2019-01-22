/*
* SD_Interface.h
*
* This file contains useful functions for saving and reading from the SD card
*/

#ifndef MPU6050INTERFACE_H
#define MPU6050INTERFACE_H

#include "User_Types.h"
#include "MPU6050_defines.h"
#include "Arduino.h"
#include <SdFat.h>

/**********************************
 * SETTINGS FOR THIS SYSTEM. PLEASE CHANGE THEM HERE
 *********************************/
#define MPU_ACCEL_SET MPU6050_AFS_SEL_2
#define MPU_GYRO_SET MPU6050_FS_SEL_2
#define MPU_RATE MPU6050_DLPF_CFG_0
#define MPU6050_I2C_ADDRESS 0x68

double ConvertAccel( int16_t bit_data );
double ConvertGyro( int16_t bit_data );
void ReadSensor(imu_data_t* sample);
void LoadCalibration(SdFat *sd, calibration_t *OutputCalib);
int MPU6050_write(int start, const uint8_t *pData, int size);
int MPU6050_write_reg(int reg, uint8_t data);
void ComputeHeadPitch( calibration_t *CalibStruct, imu_data_t* record_sample, headrp_t* head_rp );

#endif // MPU6050INTERFACE_H
