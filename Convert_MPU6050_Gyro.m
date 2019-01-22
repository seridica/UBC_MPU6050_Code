%%%
% File: Convert_MPU6050_Gyro.m
% Author: Calvin Kuo
% Date: 01-21-2019
%
% Conversion of linear accelerations and angular velocities depends on your
% settings for the MPU6050. Refer here:
%
% Gyroscope conversion (gyro stored as 16bit integer):
% 1) MPU6050_FS_SEL_250 - 16bit to 250deg/s
% 2) MPU6050_FS_SEL_500 - 16bit to 500deg/s
% 3) MPU6050_FS_SEL_1000 - 16bit to 1000deg/s
% 4) MPU6050_FS_SEL_2000 - 16bit to 2000deg/s

function convert_out = Convert_MPU6050_Gyro( raw_in )
    deg2rad = pi/180;
    
    gyro_range = 1000.0;
    
    convert_out = double( raw_in ) / 32767.0 * gyro_range * deg2rad;
end