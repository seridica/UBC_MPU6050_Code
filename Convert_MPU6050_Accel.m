%%%
% File: Convert_MPU6050_Accel.m
% Author: Calvin Kuo
% Date: 01-21-2019
%
% Conversion of linear accelerations and angular velocities depends on your
% settings for the MPU6050. Refer here:
%
% Acceleration conversion (accel stored as 16bit integer):
% 1) MPU6050_AFS_SEL_2G - 16bit to 2G
% 2) MPU6050_AFS_SEL_4G - 16bit to 4G
% 3) MPU6050_AFS_SEL_8G - 16bit to 8G
% 4) MPU6050_AFS_SEL_16G - 16bit to 16G

function convert_out = Convert_MPU6050_Accel( raw_in )
    gravity = 9.80665;  % [m/s^2] Standard gravity
    
    accel_range = 8.0;
    
    convert_out = double( raw_in ) / 32767.0 * accel_range * gravity;
end