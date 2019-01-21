#include <Wire.h>
#include <SdFat.h>
#include "MPU6050_defines.h"
#include "User_Types.h"
#include "SD_Interface.h"
#include "MPU6050_Interface.h"

SdFat SD;
SdBaseFile binFile;
const int chipSelect = 4;
const int nCalibrationSamples = 100;

void setup()
{
  while (!Serial) {
  }
  int error;
  uint8_t c;
  Serial.begin(115200);

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
  Wire.setClock(400000);
  if (!SD.begin(chipSelect, SD_SCK_MHZ(50))) {
    SD.initErrorPrint(&Serial);
  }
  
  // Configure the MPU6050
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU_ACCEL_SET); // Set Accelerometer Range
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU_GYRO_SET); // Set Gyroscope Range
  MPU6050_write_reg(MPU6050_CONFIG, MPU_RATE); // Set gyro and accel bandwidth and sample rate
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 1); // Divide Gyroscope Sample Rate to match Accelerometer

  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  /******************* 
   * CALIBRATE
   ******************/
  int incomingByte = 0;
  char filename[18];
  strcpy(filename, "Calibration_Z.csv");
  bool doCalibrate = false;
  bool sdExists = SD.exists(filename);
  
  if (sdExists) {
    Serial.println("Overwrite Calibration?");
  }
  else {
    Serial.println("Calibrate?");
  }
  
  while (Serial.available() <= 0) {
  }

  // Use first byte as 1 = yes, everything else = no
  incomingByte = Serial.read();
  if ( incomingByte == 49 ) {

      // Clear input
      while( incomingByte != 10 ) {
          incomingByte = Serial.read();
      }
      CalibrationLoop();
  }
  else {

      if (sdExists) {
        File dataFile = SD.open(filename, FILE_READ);
        dataFile.close();
        Serial.println( "Loaded Calibration" );
      }
      else {
        Serial.println( "Using Default Calibration" );
      }
  }

}

void CalibrationLoop() {
      int incomingByte = 0;
      Serial.println( "Place Subject z-up" );
      
      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      //record_file( &SD, "Calibration_Z.csv", 1000, 100000 );
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 100000 );
      //record_bin_file( &SD, &binFile, 2000, 100000 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_Z.csv" );
      
      Serial.println( "Place Subject x-up" );

      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      //record_file( &SD, "Calibration_X.csv", 1000, 100000 );
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 100000 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_X.csv" );
      
      Serial.println( "Place Subject y-up" );
      
      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      //record_file( &SD, "Calibration_Y.csv", 1000, 100000 );
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 100000 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_Y.csv" );
      
      Serial.println( "Calibration Complete" );
}

void loop()
{
  Serial.println("Select Type of Record Method: ");
  Serial.println("1) Direct to Serial Out: ");
  Serial.println("2) Direct to Flash with CSV conversion: ");
  Serial.println("3) Direct to Flash with no conversion: ");
  
  while (Serial.available() <= 0) {
  }

  // Only read the first value
  int incomingByte = Serial.read();
  int userSelect = incomingByte;
  // Clear incoming buffer
  while( incomingByte != 10 ) {
    incomingByte = Serial.read();
  }
  
  imu_data_t record_sample;
  uint32_t sample_time;
  uint32_t logTime = micros();
  String record_line;
  
  switch (userSelect) {
    case 49:

      // Stay in loop until Serial command sent
      while (true) {
        if ( Serial.available() > 0 ) {
          break;
        }
        
        sample_time = micros();
        ReadSensor( &record_sample );
        record_line = String( String(sample_time) + "," + String(record_sample.x_accel) + "," + String(record_sample.y_accel) + "," + String(record_sample.z_accel) + "," + String(record_sample.x_gyro) + "," + String(record_sample.y_gyro) + "," + String(record_sample.z_gyro) );
        Serial.println(record_line);
      }
      break;
      
    case 50:
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 60000000 );
      convert_bin_to_csv( &SD, &binFile, "TrialData.csv" );
      break;
    case 51:
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 60000000 );
      //convert_bin_to_csv( &SD, &binFile, "TrialData.csv" );
      break;
  }

      
  Serial.println("COMPLETE");
  delay(100000000);
}
