// Definition of user specified types
#ifndef USERTYPES_H
#define USERTYPES_H

#include "Arduino.h"

#define SAMPLES_PER_BLOCK 32

// Struct for accel and gyro data
typedef struct imu_data_t
{
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
};

// Struct for sample time stamp and data
typedef struct imu_sample_t
{
    uint32_t time_stamp;
    imu_data_t sample;
};

// Struct representing a 512 byte block for saving
typedef struct imu_data_block_t
{
	imu_sample_t block_storage[SAMPLES_PER_BLOCK];
};

// Struct for calibration
typedef struct calibration_t
{
  double x_row[3];
  double y_row[3];
  double z_row[3];
};

typedef struct headrp_t
{
  double headroll;
  double headpitch;
};

#endif
