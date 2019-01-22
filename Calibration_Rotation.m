%%%
% File: Calibration_Rotation.m
% Author: Calvin Kuo
% Date: 01-21-2019
%
% Notes: Solves the calibration IMU rotation given either 2 axes or 3 axes.
% The algorithm for 2-axis calibration vs. 3-axis calibration is different.
% For 2-axis calibration, we assume you have a z-up and an x-up
% measurement. For 3-axis calibration, we assume you have z-up, x-up, and
% y-up measurements.

function [Rcal, score] = Calibration_Rotation( z_up_data, x_up_data, y_up_data )
    z_up_vec = mean( z_up_data(:,2:4) );
    x_up_vec = mean( x_up_data(:,2:4) );
    if ( nargin == 2 )
        [Rcal, score] = CalibrateTwo( z_up_vec, x_up_vec );
    elseif ( nargin == 3 )
        y_up_vec = mean( y_up_data(:,2:4) );
        [Rcal, score] = CalibrateThree( z_up_vec, x_up_vec, y_up_vec );
    end
end

function [Rcal, score] = CalibrateTwo( z_up_vec, x_up_vec )
    Rcal = eye(3);
    z_norm = z_up_vec / norm( z_up_vec );
    x_norm = x_up_vec / norm( x_up_vec );
    Rcal(3,1:3) = z_norm;
    y_norm = cross( z_norm, x_norm );
    Rcal(2,1:3) = y_norm;
    Rcal(1,1:3) = cross( y_norm, z_norm );
    
    score = acos( Rcal(1,1:3) * x_norm' ) * 180/pi;
end

function [Rcal, score] = CalibrateThree( z_up_vec, x_up_vec, y_up_vec )
    z_norm = z_up_vec / norm( z_up_vec );
    y_norm = y_up_vec / norm( y_up_vec );
    x_norm = x_up_vec / norm( x_up_vec );
    
    A = eye(3);
    B = [x_norm; y_norm; z_norm];
    
    [U,S,V] = svd( A*B );
    Rcal = U*V';
    
    score = acos( Rcal(1,1:3) * x_norm' ) * 180/pi
    score = score + acos( Rcal(2,1:3) * y_norm' ) * 180/pi
    score = score + acos( Rcal(3,1:3) * z_norm' ) * 180/pi
end