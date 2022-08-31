function [x,y,z] = ECEFtoENU(X,Y,Z,lat,lon,initialPoint)

% ****************************************************************************
% NOTICE
%
% This is the copyright work of The MITRE Corporation, and was produced
% for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
% is subject to Federal Aviation Administration Acquisition Management
% System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
% (Oct. 1996).  No other use other than that granted to the U. S.
% Government, or to those acting on behalf of the U. S. Government,
% under that Clause is authorized without the express written
% permission of The MITRE Corporation. For further information, please
% contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
% McLean, VA  22102-7539, (703) 983-6000. 
%
% Copyright 2020 The MITRE Corporation. All Rights Reserved.
% ****************************************************************************

% Convert ECEF coordinates to ENU
% Inputs: X,Y,Z - ECEF coordinate vector, m
%         lat, lon - vectors with corresponding latitude and longitude
%         initialPoint - starting point for the trajectory in ENU (coule be
%         (0,0,0) or a different value depending on offset for
%         non-coincident routes).

x(1) = initialPoint(1); y(1) = initialPoint(2); z(1) = initialPoint(3);

lat = deg2rad(lat);
lon = deg2rad(lon);

for ind0 = 2:length(X)
    
    r = [-sin(lon(ind0))                cos(lon(ind0))                  0;
        -sin(lat(ind0))*cos(lon(ind0))  -sin(lat(ind0))*sin(lon(ind0))  cos(lat(ind0));
        cos(lat(ind0))*cos(lon(ind0))   cos(lat(ind0))*sin(lon(ind0))   sin(lat(ind0))];
    
    d = r*[-X(ind0-1)+X(ind0); -Y(ind0-1)+Y(ind0); -Z(ind0-1)+Z(ind0)];
    
    x(ind0) = x(ind0-1) + d(1);
    y(ind0) = y(ind0-1) + d(2);
    z(ind0) = z(ind0-1) + d(3);
    
end