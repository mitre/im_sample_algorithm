function distance = CalculateDistance(lat1,lon1,lat2,lon2)

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

% Finds the great circle distance between two latitude/longitude points.
% Inputs are latitudes and longitudes for each point in degrees. Output is
% range in NM.

R = 60; % 1 NM = 1 minute of Earth circumference

lat2 = deg2rad(lat2);
lon2 = deg2rad(lon2);
lat1 = deg2rad(lat1);
lon1 = deg2rad(lon1);

deltaLat = lat2 - lat1;    % Difference in latitudes
deltaLon = lon2 - lon1;    % Difference in longitudes

% Haversine formula for great circle distance
a = sin(deltaLat/2).^2 + ...
        cos(lat1).*cos(lat2).*sin(deltaLon/2).^2;

c = 2 * atan2(sqrt(a), sqrt(1-a));

distance = R * rad2deg(c); % Convert to NM