function ac = readRoute(routeFile,ac,operation)

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

% Read waypoints from text file. 

% Open file and read data
fid = fopen(routeFile);
data = textscan(fid,'%s%f%f%s%f%s%[^\n]','delimiter',sprintf('\t'),'headerlines',1,'MultipleDelimsAsOne',1);
fclose(fid);

for ind1 = 1:length(ac)
    
    % Get route for this aircraft
    route = ac(ind1).route.routeName;
    
    % Find the line with the desired route
    routeNums = find(contains(data{6},route)==1);
    operationNums = find(contains(data{4},operation)==1);
    routeNum = intersect(routeNums,operationNums);
    routeNum = routeNum(1);
    
    % Read that line to get the waypoints
    raw_waypoints = data{end}(routeNum);
    waypoints = strtrim(strsplit(raw_waypoints{:},sprintf('\t'),'CollapseDelimiters',true));
    waypoints = waypoints(~cellfun(@isempty,waypoints));
       
    % Store route information in ac struct
    ac(ind1).route.waypoint = waypoints';
    
end