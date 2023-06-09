function [iterationCounter,turnPointValidity,waypointSequence,turnPoint] = calcPathShorteningTurnPointTTG(dtgTarget,gsTarget,dIM0,tasIM,gsIM,ASG,Vwx0,Vwy0,subplotnum,s,x,y,alpha,plotting)

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

% IM Turn Point calculation
% Inputs: dtgTarget - Target's total route length (m)
%         gsTarget - Target's ground speed (m/s)
%         dIM0 - IM Aircraft's total route length (m)
%         tasIM - IM Aircraft's TAS (m/s)
%         gsIM - IM Aircraft's ground speed (m/s)
%         ASG - Assigned Spacing Goal (m)
%         Vwx0 - x-component of wind (m/s)
%         Vwy0 - y-component of wind (m/s)
%         subplotnum - index for correct subplot if plotting is on
%         s - vector of path lengths from start of each leg on IM
%             Aircraft's route to PTP (m)
%         x - x-coordinate of waypoints on IM Aircraft's route (m)
%         y - y-coordinate of waypoints on IM Aircraft's route (m)
%         alpha - vector of course between waypoints on IM Aircraft's
%                 route (m)
%         plotting - flag to turn plotting on/off (1 = on, 0 = off)
% Outputs: iterationCounter - number of iterations required to find a turn
%                             point
%          turnPointValidity - flag indicating whether the turn point meets
%                              the validity checks, can be VALID, INVALID -
%                              TURN POINT TURN TOO LARGE, INVALID -
%                              INTERCEPT POINT TURN TOO LARGE, INVALID - NO
%                              SOLUTION
%          waypointSequence - struct containing (x,y) location in (NM) of
%                             both the original route and the turn-point
%                             route

% Fontsize for plotting
fontsize = 11;

% Intercept point coordinates
xIP = x(end);
yIP = y(end);

%% Times to Go

TTGtarget = dtgTarget/gsTarget;
TTGim = TTGtarget + ASG;

%% Determine Error in Path Length for max/min values of dIMA

% Determine max and min values for dIMA
shortestPossible = sqrt((xIP-x(1))^2 + (yIP-y(1))^2);
dIM = [shortestPossible,.999*dIM0];

% Check if operation is possible, if not, plot "X"
check = dIM/gsIM;
if TTGim < min(check) || TTGim > max(check)
    waypointSequence.old.x = m2NM(x);
    waypointSequence.old.y = m2NM(y);
    waypointSequence.new.x = m2NM(x);
    waypointSequence.new.y = m2NM(y);
    
    turnPoint.x = nan;
    turnPoint.y = nan;
    
    iterationCounter = nan;
    turnPointValidity = 'INVALID - NO SOLUTION';
    if plotting == 1
        h = gcf;
        h = h.Children(end);
        subplot(subplotnum)
        plot(h.XLim,h.YLim,'k')
        hold on
        plot(fliplr(h.XLim),h.YLim,'k')
        grid on
        box on
        xlabel('x (NM)')
        ylabel('y (NM)')
        set(gca,'fontsize',fontsize)
        axis([0 25 h.YLim])
    end
    return
end

% Find correct leg and leg length for IM turn point
dIMA = zeros(size(dIM));
counterOut = zeros(size(dIM));
legLengths = zeros(size(dIM));
for ind0 = 1:length(dIM)
    
    % Initial values for loop variables
    L = 0;
    dIMA(ind0) = dIM0;    
    counter = 1;
    
    % Initial dIMA calculation
    d = sqrt((xIP-x(counter))^2 + (yIP-y(counter))^2);
    phi = wrapTo2Pi(alpha(counter)-wrapTo2Pi(atan2(yIP-y(counter),xIP-x(counter))));
    dIMA(ind0) = ((dIM(ind0) - L)^2 - d^2)/(2*(dIM(ind0)-L-d*cos(phi)));
    
    % Loop until correct leg and dIMA value are found
    while dIMA(ind0) > s(counter)
        
        % Increment total leg length and loop counter
        L = L + s(counter);
        counter = counter + 1;
        
        % Find dIMA
        d = sqrt((xIP-x(counter))^2 + (yIP-y(counter))^2);
        phi = wrapTo2Pi(alpha(counter)-wrapTo2Pi(atan2(yIP-y(counter),xIP-x(counter))));
        dIMA(ind0) = ((dIM(ind0) - L)^2 - d^2)/(2*(dIM(ind0)-L-d*cos(phi)));
        
    end
    
    % Store counter and leg lengths for each loop
    counterOut(ind0) = counter;
    legLengths(ind0) = L;
    
end

% Time to turn point
tTurn = (dIMA+legLengths)/gsIM;
tRem = TTGim - tTurn;

% Location of turn point
xTurn = x(counterOut) + dIMA.*cos(alpha(counterOut));
yTurn = y(counterOut) + dIMA.*sin(alpha(counterOut));


% Return Track
trkReturn = zeros(2,1);
for ind1 = 1:2
    trkReturn(ind1) = atan2(yIP-yTurn(ind1),xIP-xTurn(ind1));
end

% Wind speeds
Vw_para = Vwx0*cos(trkReturn) + Vwy0*sin(trkReturn);
Vw_perp = -Vwy0*sin(trkReturn) + Vwy0*cos(trkReturn);

% Ground speed on the return leg of the IM Turn
gs_ret = sqrt(tasIM^2 - Vw_perp.^2) + Vw_para;

% Length of return leg in TTGim
dIMB = gs_ret.*tRem;

% dIMb with no error (ending at intercept point)
dIMB_ = sqrt((xIP - xTurn).^2 + (yIP - yTurn).^2);

derror = dIMB - dIMB_;

%% Interate on dIM
if plotting == 1
    subplot(subplotnum(:))
    hold on
    plot(m2NM([x(1:counterOut(1)), xTurn(1), x(end)]),m2NM([y(1:counterOut(1)), yTurn(1), y(end)]),'--','color',PlotColors(2))
    plot(m2NM([x(1:counterOut(2)), xTurn(2), x(end)]),m2NM([y(1:counterOut(2)), yTurn(2), y(end)]),'--','color',PlotColors(2))
end

iterationCounter = 0;
de = 100; % Threshold for distance-error in true vs ideal time to fly
while(abs(de) > 0.025) && iterationCounter < 50
    
    % Initial values for loop variables
    dIM_new = mean(dIM);
    counter = 1;
    L = 0;
    
    % Initial dIMA_new calculation
    d = sqrt((xIP-x(counter))^2 + (yIP-y(counter))^2);
    phi = wrapTo2Pi(alpha(counter)-wrapTo2Pi(atan2(yIP-y(counter),xIP-x(counter))));
    dIMA_new = ((dIM_new - L)^2 - d^2)/(2*(dIM_new-L-d*cos(phi)));
    
    % Compare dIMA_new to leg lengths to see which leg it falls on
    while dIMA_new > s(counter) && counter < length(s)
        
        % Increment total leg length and loop counter
        L = L + s(counter);
        counter = counter + 1;
        
        % Find dIMA
        d = sqrt((xIP-x(counter))^2 + (yIP-y(counter))^2);
        phi = wrapTo2Pi(alpha(counter)-wrapTo2Pi(atan2(yIP-y(counter),xIP-x(counter))));
        dIMA_new = ((dIM_new - L)^2 - d^2)/(2*(dIM_new-L-d*cos(phi)));
        
    end
    
    % Store loop variables
    counterOut(ind0) = counter;
    legLengths(ind0) = L;
    
    % Determine time to turn point
    tTurn_new = (L + dIMA_new)/gsIM;
    tRem_new = TTGim - tTurn_new;
    
    % New turn point
    xTurn_new = x(counter) + dIMA_new.*cos(alpha(counter));
    yTurn_new = y(counter) + dIMA_new.*sin(alpha(counter));
    
    % Return track from new turn point
    trkReturn_new = wrapTo2Pi(atan2(yIP-yTurn_new,xIP-xTurn_new));
    
    % Winds
    Vw_para = Vwx0 * cos(trkReturn_new) + Vwy0 * sin(trkReturn_new);
    Vw_perp = -Vwx0 * sin(trkReturn_new) + Vwy0 * cos(trkReturn_new);
    
    % Ground speed on return leg
    gs_new = sqrt(tasIM^2 - Vw_perp^2) + Vw_para;
    
    % dIMB
    dIMB_new = gs_new.*tRem_new;
    
    % Distance error in total path length
    de = dIMB_new - sqrt((xIP - xTurn_new).^2 + (yIP - yTurn_new).^2);
    
    % Update max or min dIMA
    if(de >= 0)
        dIM(1) = dIM_new;
        derror(1) = de;
    elseif(de < 0)
        dIM(2) = dIM_new;
        derror(2) = de;
    end
    
    % Update plot and iteration counter
    if plotting == 1
        plot(m2NM([x(1:counter), xTurn_new, x(end)]),m2NM([y(1:counter), yTurn_new, y(end)]),'--','color',PlotColors(6),'linewidth',1)
    end    
    iterationCounter = iterationCounter + 1;
    
end

% Total path length
dIM = dIMA_new + dIMB_new;

% Check track change at turn point
trkChange = abs(wrapToPi(alpha(counter)-wrapTo2Pi(atan2(yIP-yTurn_new,xIP-xTurn_new))));
altitude = min(tand(3)*dIM,ft2m(35e3));
if (trkChange >= deg2rad(120) && altitude < ft2m(19500)) || (trkChange >= deg2rad(70) && altitude > ft2m(19500))
    turnPointValidity = 'INVALID - TURN POINT TURN TOO LARGE';
else
    turnPointValidity = 'VALID';
end

% Check track change at intercept point
trkChange = abs(wrapToPi(wrapTo2Pi(atan2(yIP-yTurn_new,xIP-xTurn_new))-alpha(end)));
if strcmp(turnPointValidity,'VALID')
    if (trkChange >= deg2rad(120) && altitude < ft2m(19500))
        turnPointValidity = 'INVALID - INTERCEPT POINT TURN TOO LARGE';
    else
        turnPointValidity = 'VALID';
    end
end

% Final waypoint sequence
waypointSequence.new.x = m2NM([x(1:counter), xTurn_new, x(end)]);
waypointSequence.new.y = m2NM([y(1:counter), yTurn_new, y(end)]);
waypointSequence.old.x = m2NM(x);
waypointSequence.old.y = m2NM(y);

% IM Turn Point
turnPoint.x = m2NM(xTurn_new);
turnPoint.y = m2NM(yTurn_new);

if plotting == 1
    % Plot final IM Turn Point
    plot(m2NM([x(1:counter), xTurn_new, x(end)]),m2NM([y(1:counter), yTurn_new, y(end)]),'color',PlotColors(1))
    plot(m2NM(x),m2NM(y),'.-','color',[.4 .4 .4])
    xlabel('x (NM)')
    ylabel('y (NM)')
    set(gca,'fontsize',fontsize)
    grid on
    box on
end

