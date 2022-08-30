function [totalIterations, validity, initError_s, waypointSequence, turnPoint] = main_closedLoopTurn_pathShortening(airports,initSpacing,plotting,routeType)

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

% Main function to calculate the location of an IM Turn
% Inputs: airports - cell array of airports of interest, options are KLAS,
%                    KDEN, KPHX
%         initSpacing - initial along-path spacing between IM and Target 
%                       aircraft
%         plotting - can be 0 (no plots) or 1 (make plots)
%         routeType - (string) can be short or long
% Outputs: totalIterations - number of iterations to find the IM Turn
%          validity - result of validity check requirements
%          initError_s - initial true spacing error, seconds
%          waypointSequence - struct containing x,y (NM) values for the 3
%                             route waypoints with and without the IM Turn 
%                             Point
%          turnPoint - struct containing the x,y (NM) values for the IM
%                      Turn Point

% Path handling
addpath(genpath('PathGeneration'))
addpath(genpath('Data'))
addpath(genpath('Utilities'))

% Wind values
Vwx0 = kts2mps(0); 
Vwy0 = kts2mps(0);

% Assigned Spacing Goal (seconds)
ASG = 120;

% Preallocate loop variables
initError_s = zeros(length(airports),length(initSpacing));
totalIterations = zeros(length(airports),length(initSpacing));
validity = cell(length(airports),length(initSpacing));
waypointSequence0.new = struct('x',0,'y',0);
waypointSequence0.old = struct('x',0,'y',0);
waypointSequence = repmat(waypointSequence0,length(airports),length(initSpacing));
turnPoint = repmat(struct('x',0,'y',0),length(airports),length(initSpacing));

% Loop through airports
for ind0 = 1:length(airports)
    
    if plotting == 1
        % Figure window
        figure('Position',[0 0 1300 1100])
        hold on
    end
    
    for ind1 = 1:length(initSpacing)
        
        % Plotting variable
        subplotnum = 310 + ind1;
        
        % Location of Intercept Point and Route Generation
        scenarioName = [airports{ind0},'_IM_Turn_',upper(routeType(1)),lower(routeType(2:end))];
        ac = HorizontalPathGeneration_MOPS(scenarioName);
        s = fliplr(ac.route.L); % distance from PTP, m
        x = fliplr(ac.route.x); % waypoint x-coordinate, m
        y = fliplr(ac.route.y); % waypoint y-coordinate, m
        alpha = wrapTo2Pi(atan2(diff(y),diff(x))); % course between waypoints, radians
        
        %% Initial Conditions - IM Aircraft
        
        dtgIM = sum(s);        % Total route length, m
        tasIM = kts2mps(280);  % Assumed constant TAS, m/s
        
        hdg = alpha(1);        % vector from original route
        
        xdot = tasIM*sin(hdg) + Vwx0;   % x-component of velocity, m/s
        ydot = tasIM*cos(hdg) + Vwy0;   % y-component of velocity, m/s
        
        gsIM = sqrt(xdot^2 + ydot^2);   % groundspeed, m/s
                
        %% Initial Conditions - Target Aircraft
        
        dtgTarget = dtgIM - NM2m(initSpacing(ind1)); % Total route length, m
        
        tasTarget = kts2mps(290);   % Assumed constant TAS, m/s
        
        gsTarget = sqrt(tasTarget^2 - Vwx0^2) + Vwy0; % Groundspeed, m/s
        
        % Initial time-based spacing error without the IM Turn
        tTarget = dtgTarget/gsTarget;
        tIM = dtgIM/(sqrt(tasIM^2 - Vwx0^2) + Vwy0);
        
        initSpacing_s = tIM - tTarget;
        initError_s(ind0,ind1) = initSpacing_s - ASG;
        
        % Calculate IM Turn Point
        [iterations,turnPointValidity,waypointSequenceTemp,turnPointTemp] = calcPathShorteningTurnPointTTG(dtgTarget,gsTarget,dtgIM,tasIM,gsIM,ASG,Vwx0,Vwy0,subplotnum,s,x,y,alpha,plotting);
        totalIterations(ind0,ind1) = iterations;
        validity{ind0,ind1} = turnPointValidity;
        waypointSequence(ind0,ind1) = waypointSequenceTemp;
        turnPoint(ind0,ind1) = turnPointTemp;
        
        % Plot title
        if plotting == 1
            title({['Initial Spacing Error = ',num2str(round(initError_s(ind0,ind1),0)),' sec']})
        end
        
    end
    
    if plotting == 1
        % More plotting stuff
        h = get(gcf);
        linkaxes([h.Children],'xy')
        figName = ['Figures/',airports{ind0},'_IM_Turn'];
        savefig(gcf,figName)
        saveas(gcf,figName,'png')
    end
    
end



