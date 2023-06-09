function [X,Y,Z] = GeoToECEF(lat,lon,alt)

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

% Convert geodetic to ECEF coordinates
% Assume inputs are in (deg,deg,m)
% Outputs are ECEF coordinates in m

% Convert inputs to radians
lat = deg2rad(lat);
lon = deg2rad(lon);

% WGS-84 parameters
a = 6378137; %m
f = 1/298.257223563; % unitless
b = a*(1-f); % m

% Conversion
e2 = 1 - b^2/a^2;
N = a/sqrt(1-e2*(sin(lat)).^2);

% Check N is same dimensions as lat/lon
[rN,~] = size(N);
[rL,~] = size(lat);
if rN ~= rL
    N=N';
end

% Calculate ECEF coordinates
X = (N+alt) .* cos(lat) .* cos(lon);
Y = (N+alt) .* cos(lat) .* sin(lon);
Z = (N+alt) .* (1-e2) .* sin(lat);



