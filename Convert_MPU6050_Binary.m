%%%
% File: Convert_MPU6050_Binary.m
% Author: Calvin Kuo
% Date: 01-21-2019
%
% Notes: This code converts the MPU6050 binary file into matlab nx7 matrix 
% with n data samples and columns:
% 1) Time stamp (microseconds)
% 2) X-accel (bits)
% 3) Y-accel (bits)
% 4) Z-accel (bits)
% 5) X-gyro (bits)
% 6) Y-gyro (bits)
% 7) Z-gyro (bits)


function [converted_matrix, binary_matrix] = Convert_MPU6050_Binary()
    defaultDirectory = 'C:\Data\';
    [infiles, inpath] = uigetfile({'*', 'Binary Data File'});

    % FILE IMPORT
    % =========================================================================    
    infiles = cellstr(infiles); % Convert infiles to cell array (to catch case of a single file)
    
    fileID = fopen([inpath, infiles{1}]);
    allData = fread(fileID);
    fclose(fileID);
    
    % All data is an array of single bytes.
    % Time code is 4 bytes
    % x-, y- ,z-accel and x-, y-, z- gyro are all 2 bytes
    nBytes = length( allData );
    assert( mod( nBytes, 16 ) == 0 );
    
    nData = nBytes / 16;
    binary_matrix = zeros( nData, 7 );
    converted_matrix = zeros( nData, 7 );
    
    cbyte = 1;
    for i=1:nData
        binary_matrix(i,1) = bitshift(allData(cbyte+3),24,'uint32') + bitshift(allData(cbyte+2),16,'uint32') + bitshift(allData(cbyte+1),8,'uint32') + uint32( allData(cbyte) );
        cbyte = cbyte + 4;
        
        binary_matrix(i,2) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
        
        binary_matrix(i,3) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
        
        binary_matrix(i,4) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
        
        binary_matrix(i,5) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
        
        binary_matrix(i,6) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
        
        binary_matrix(i,7) = bitshift(allData(cbyte+1),8,'int16') + int16(allData(cbyte));
        cbyte = cbyte + 2;
    end
    
    converted_matrix(:,1) = ( binary_matrix(:,1) - binary_matrix(1,1) ) / 1000000;
    converted_matrix(:,2:4) = Convert_MPU6050_Accel( binary_matrix(:,2:4) );
    converted_matrix(:,5:7) = Convert_MPU6050_Gyro( binary_matrix(:,5:7) );
end