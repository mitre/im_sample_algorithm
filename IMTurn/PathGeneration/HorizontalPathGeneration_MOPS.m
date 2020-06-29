function ac = HorizontalPathGeneration_MOPS(scenarioName)

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

% Generate the horizontal path to be followed during the IM Turn
% Inputs: scenarioName - name of scenario (string)
% Outputs: ac - struct with route and horizontal path (hpt) sub-structs

load(scenarioName,'scenario','data');

% Build routes
ac = struct('ac_type',{'A319'},'bankAngle',23);
if strcmp(scenario.airport,'KDEN')
    ac.route.routeName = 'KIPPR4_CTFSH';
elseif strcmp(scenario.airport,'KLAS')
    ac.route.routeName = 'SUNST3_TACUS';
elseif strcmp(scenario.airport,'KPHX')
    ac.route.routeName = 'KOOLY4_SSO';
end
ac = getRouteDetails(ac,scenario,data);


for ind0 = 1:length(ac)
    
    % Reverse route waypoints so that trajectory is built backwards
    fieldnames = fields(ac(ind0).route);
    for ind2 = 1:length(fieldnames)
        ac(ind0).route.(fieldnames{ind2}) = flipud(ac(ind0).route.(fieldnames{ind2}));
    end
    
    % Convert latitudes and longitudes to ECEF coordinates
    X = zeros(size(ac(ind0).route.lat)); Y = zeros(size(ac(ind0).route.lat)); Z = zeros(size(ac(ind0).route.lat));
    for ind1 = 1:length(ac(ind0).route.lat)
        [X(ind1),Y(ind1),Z(ind1)] = GeoToECEF(ac(ind0).route.lat(ind1),ac(ind0).route.lon(ind1),0);
    end    
    
    % Store ECEF points in route structure
    ac(ind0).route.X = X;
    ac(ind0).route.Y = Y;
    ac(ind0).route.Z = Z;
    
    % Determine starting point for ENU coordinates, first aircraft's last
    % waypoint is chosen as the origin
    if ind0 == 1
        initialPoint = [0,0,0];
    else
        smallCat = @(field) [ac(1).route.(field)(1), ac(ind0).route.(field)(1)];
        tempLat = smallCat('lat');
        tempLon = smallCat('lon');
        tempX = smallCat('X');
        tempY = smallCat('Y');
        tempZ = smallCat('Z');
        [xi,yi,zi] = ECEFtoENU(tempX,tempY,tempZ,tempLat,tempLon,[0,0,0]);
        initialPoint = [xi(2),yi(2),zi(2)];
    end
    
    % Convert from ECEF to ENU
    [x,y,z] = ECEFtoENU(X,Y,Z,ac(ind0).route.lat,ac(ind0).route.lon,initialPoint);
    ac(ind0).route.x = x;
    ac(ind0).route.y = y;
    ac(ind0).route.z = z;
    
    % Determine distance and course angle for each waypoint
    for ind1 = 2:length(x)
        lat1 = ac(ind0).route.lat(ind1-1);
        lon1 = ac(ind0).route.lon(ind1-1);
        lat2 = ac(ind0).route.lat(ind1);
        lon2 = ac(ind0).route.lon(ind1);
        ac(ind0).route.L(ind1-1) = NM2m(CalculateDistance(lat1, lon1, lat2, lon2));
        ac(ind0).route.pathLength(ind1-1) = sum(ac(ind0).route.L);
        
        course = atan2(-y(ind1-1)+y(ind1),-x(ind1-1)+x(ind1));
        ac(ind0).route.course(ind1-1) = wrapTo2Pi(course);
    end
    
        
end

% Calculate the horizontal path transition points
ac = CalculateHPTs(ac);








