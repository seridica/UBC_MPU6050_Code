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
const int8_t LED_PIN = 13;
calibration_t CalibStruct;

void setup()
{
  while (!Serial) {
  }
  int error;
  uint8_t c;
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

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
      LoadCalibration(&SD, &CalibStruct);
  }
  else {

      if (sdExists) {
        Serial.println( "Loaded Calibration" );
        LoadCalibration(&SD, &CalibStruct);
      }
      else {
        Serial.println( "Using Default Calibration" );
        CalibStruct.x_row[0] = 1.0;
        CalibStruct.x_row[1] = 0.0;
        CalibStruct.x_row[2] = 0.0;
        CalibStruct.y_row[0] = 0.0;
        CalibStruct.y_row[1] = 1.0;
        CalibStruct.y_row[2] = 0.0;
        CalibStruct.z_row[0] = 0.0;
        CalibStruct.z_row[1] = 0.0;
        CalibStruct.z_row[2] = 1.0;
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
      record_bin_file_block( &SD, &binFile, 2000, 500 );
      //record_bin_file( &SD, &binFile, 2000, 100000 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_Z.csv" );
      binFile.rename(SD.vwd(),"Calibration_Z");
      
      Serial.println( "Place Subject x-up" );

      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      //record_file( &SD, "Calibration_X.csv", 1000, 100000 );
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 500 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_X.csv" );
      binFile.rename(SD.vwd(),"Calibration_X");
      
      Serial.println( "Place Subject y-up" );
      
      while(Serial.available() <= 0) {
      }
      incomingByte = Serial.read();
      while( incomingByte != 10 ) {
        incomingByte = Serial.read();
      }
      
      //record_file( &SD, "Calibration_Y.csv", 1000, 100000 );
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 500 );
      convert_bin_to_csv( &SD, &binFile, "Calibration_Y.csv" );
      binFile.rename(SD.vwd(),"Calibration_Y");
      
      Serial.println( "Calibration Complete" );
}

void loop()
{
  Serial.println("Select Type of Record Method: ");
  Serial.println("1) Direct to Serial Out: ");
  Serial.println("2) Direct to Flash with CSV conversion: ");
  Serial.println("3) Direct to Flash with no conversion: ");
  Serial.println("4) Head Angle Computation: ");
  
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
  int32_t delta;
  uint32_t microDelay = 2000;
  uint32_t headMicroDelay = 100000;
  headrp_t head_angles;
  
  digitalWrite(LED_PIN, HIGH);
  switch (userSelect) {
    case 49:

      // Stay in loop until Serial command sent
      while (true) {

        do {
          delta = micros() - logTime;
        } while (delta < 0);
        
        if ( Serial.available() > 0 ) {
          break;
        }
        
        sample_time = micros();
        ReadSensor( &record_sample );
        record_line = String( String(sample_time) + "," + String(record_sample.x_accel) + "," + String(record_sample.y_accel) + "," + String(record_sample.z_accel) + "," + String(record_sample.x_gyro) + "," + String(record_sample.y_gyro) + "," + String(record_sample.z_gyro) );
        Serial.println(record_line);
        
        // Update expected time for next 
        logTime += microDelay;
      }
      break;
      
    case 50:
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 2000, 10000 );
      convert_bin_to_csv( &SD, &binFile, "TrialData.csv" );
      //binFile.rename(SD.vwd(),"TrialData");
      break;
    case 51:
      create_bin_file( &SD, &binFile );
      record_bin_file_block( &SD, &binFile, 1000, 7200000 );
      //convert_bin_to_csv( &SD, &binFile, "TrialData.csv" );
      //binFile.rename(SD.vwd(),"TrialData");
      break;
    case 52:
      
      // Stay in loop until Serial command sent
      while (true) {

        do {
          delta = micros() - logTime;
        } while (delta < 0);
        
        if ( Serial.available() > 0 ) {
          break;
        }
        
        sample_time = micros();
        ReadSensor( &record_sample );
        ComputeHeadPitch( &CalibStruct, &record_sample, &head_angles );
        
        record_line = String(String(sample_time) + "," + String(head_angles.headpitch) + "," + String(head_angles.headroll) + "," + String(ConvertGyro(record_sample.x_gyro))  + "," + String(ConvertGyro(record_sample.y_gyro))  + "," + String(ConvertGyro(record_sample.z_gyro)) );
        Serial.println(record_line);
        
        // Update expected time for next 
        logTime += headMicroDelay;
      }
      break;
  }
  digitalWrite(LED_PIN, LOW);

      
  Serial.println("COMPLETE");
  delay(100000000);
}
