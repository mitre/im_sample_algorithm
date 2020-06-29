function ac = getRouteDetails(ac,scenario,data)

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

% Unpack inputs
operation = scenario.operation;
routeFile = data.routeFile;

%% Get route waypoints
wpt = getWaypoints(data.wptDatabase);
ac = readRoute(routeFile,ac,operation);

for ind0 = 1:length(ac)
    %% Get Lat/Long and Constraints
    
    for ind1 = 1:length(ac(ind0).route.waypoint)
        
        index = find(strcmp(wpt.name,ac(ind0).route.waypoint(ind1)));
        ac(ind0).route.lat(ind1,1) = wpt.lat(index);
        ac(ind0).route.lon(ind1,1) = wpt.lon(index);
        ac(ind0).route.spdHi(ind1,1) = wpt.spdHi(index);
        ac(ind0).route.spdLo(ind1,1) = wpt.spdLo(index);
        
        
    end
end

end

