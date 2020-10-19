function ac = CalculateHPTs(ac)

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

% Calculate the horizontal path transition points

for ind0 = 1:length(ac)
    
    % Values at first point
    counter = 2;
    ac(ind0).hpath.x = ac(ind0).route.x(1);
    ac(ind0).hpath.y = ac(ind0).route.y(1);
    ac(ind0).hpath.apd = 0;
    ac(ind0).hpath.segmentType = {'straight'};
    ac(ind0).hpath.course = ac(ind0).route.course(1);
    ac(ind0).hpath.xTurnCenter = 0;
    ac(ind0).hpath.yTurnCenter = 0;
    ac(ind0).hpath.startAngle = 0;
    ac(ind0).hpath.endAngle = 0;
    ac(ind0).hpath.R = 0;
    
    for ind1 = 2:length(ac(ind0).route.x)-1
        
        % Calculate delta theta
        delta_theta = ac(ind0).route.course(ind1)-ac(ind0).route.course(ind1-1);
        ac(ind0).route.delta_theta(ind1) = wrapToPi(delta_theta);
        
        % Determine ground speed through turn, assume aircraft flies
        % procedural speed and no winds
        Vcas = kts2mps(ac(ind0).route.spdHi(ind1));
        if Vcas == 1000
            Vcas = kt2mps(ac(ind0).route.spdHi(ind1-1));
        end
        alt = (ac(ind0).route.pathLength(ind1-1))*tand(3)+914;
        Vgs = CAS2TAS(Vcas,alt);
        ac(ind0).route.Vgs(ind1) = mps2kts(Vgs);
        
        % Bank angle
        ac(ind0).route.phi(ind1) = deg2rad(ac(ind0).bankAngle);
        
        % Calculate turn radius
        R = (Vgs^2/(physics.g*tan(ac(ind0).route.phi(ind1)))); % m
        
        % Turn anticipation
        turnAnticipation = R*tan(abs(ac(ind0).route.delta_theta(ind1))/2);
        if m2NM(turnAnticipation) > 0.2
            
            % Turn segment
            
            [xTurnEnd,yTurnEnd,xTurnCenter,yTurnCenter,xTurnStart,...
                yTurnStart,startAngle,endAngle] = CalcTurnPoints(ac,ind0,ind1,R);
            
            % Check leg lengths
            startLength = sqrt((xTurnStart-ac(ind0).route.x(ind1))^2 + (yTurnStart-ac(ind0).route.y(ind1))^2);
            endLength = sqrt((xTurnEnd-ac(ind0).route.x(ind1))^2 + (yTurnEnd-ac(ind0).route.y(ind1))^2);
            
            if endLength>ac(ind0).route.L(ind1-1)/2 || startLength>ac(ind0).route.L(ind1)/2
                shortLength = min(ac(ind0).route.L(ind1),ac(ind0).route.L(ind1-1));
                R = (shortLength/2)/tan((abs(delta_theta))/2);
                [xTurnEnd,yTurnEnd,xTurnCenter,yTurnCenter,xTurnStart,...
                    yTurnStart,startAngle,endAngle] = CalcTurnPoints(ac,ind0,ind1,R);
            end
            
            % Calculate along-path distance
            turnDist1 = sqrt((xTurnEnd-ac(ind0).route.x(ind1-1))^2 + (yTurnEnd-ac(ind0).route.y(ind1-1))^2);
            turnDist2 = sqrt((xTurnEnd-xTurnStart)^2 + (yTurnEnd-yTurnStart)^2);
            
            % Store everything
            ac(ind0).hpath.x(counter) = xTurnEnd;
            ac(ind0).hpath.y(counter) = yTurnEnd;
            ac(ind0).hpath.apd(counter) = ac(ind0).hpath.apd(end)+turnDist1;
            ac(ind0).hpath.segmentType{counter} = 'turn';
            ac(ind0).hpath.course(counter) = 1e7;
            ac(ind0).hpath.xTurnCenter(counter) = xTurnCenter;
            ac(ind0).hpath.yTurnCenter(counter) = yTurnCenter;
            ac(ind0).hpath.startAngle(counter) = startAngle;
            ac(ind0).hpath.endAngle(counter) = endAngle;
            ac(ind0).hpath.R(counter) = R;
            
            counter = counter + 1;
            
            % Store the next hpt which will be a straight segment
            ac(ind0).hpath.x(counter) = xTurnStart;
            ac(ind0).hpath.y(counter) = yTurnStart;
            ac(ind0).hpath.apd(counter) = ac(ind0).hpath.apd(end)+turnDist2;
            ac(ind0).hpath.segmentType{counter} = 'straight';
            ac(ind0).hpath.course(counter) = ac(ind0).route.course(ind1);
            ac(ind0).hpath.xTurnCenter(counter) = 0;
            ac(ind0).hpath.yTurnCenter(counter) = 0;
            ac(ind0).hpath.startAngle(counter) = 0;
            ac(ind0).hpath.endAngle(counter) = 0;
            ac(ind0).hpath.R(counter) = 0;
            
            counter = counter + 1;
        else
            
            % Straight segment
            
            ac(ind0).hpath.x(counter) = ac(ind0).route.x(ind1);
            ac(ind0).hpath.y(counter) = ac(ind0).route.y(ind1);
            ac(ind0).hpath.apd(counter) = ac(ind0).hpath.apd(end)+sqrt((ac(ind0).route.x(ind1)-ac(ind0).route.x(ind1-1))^2 + (ac(ind0).route.y(ind1)-ac(ind0).route.y(ind1-1))^2);
            ac(ind0).hpath.segmentType{counter} = 'straight';
            ac(ind0).hpath.course(counter) = ac(ind0).route.course(ind1);
            ac(ind0).hpath.xTurnCenter(counter) = 0;
            ac(ind0).hpath.yTurnCenter(counter) = 0;
            ac(ind0).hpath.startAngle(counter) = 0;
            ac(ind0).hpath.endAngle(counter) = 0;
            ac(ind0).hpath.R(counter) = 0;
            
            counter = counter + 1;
        end
        
    end
    
    % Store the last point
    ac(ind0).hpath.x(counter) = ac(ind0).route.x(end);
    ac(ind0).hpath.y(counter) = ac(ind0).route.y(end);
    ac(ind0).hpath.apd(counter) = ac(ind0).hpath.apd(end)+sqrt((ac(ind0).route.x(end)-ac(ind0).route.x(end-1))^2 + (ac(ind0).route.y(end)-ac(ind0).route.y(end-1))^2);
    ac(ind0).hpath.segmentType{counter} = 'straight';
    ac(ind0).hpath.course(counter) = ac(ind0).route.course(end);
    ac(ind0).hpath.xTurnCenter(counter) = 0;
    ac(ind0).hpath.yTurnCenter(counter) = 0;
    ac(ind0).hpath.startAngle(counter) = 0;
    ac(ind0).hpath.endAngle(counter) = 0;
    ac(ind0).hpath.R(counter) = 0;
    
end
end

function [xTurnEnd,yTurnEnd,xTurnCenter,yTurnCenter,xTurnStart,yTurnStart,startAngle,endAngle] = CalcTurnPoints(ac,ind0,ind1,R)


% Turn endpoints
tmp = ac(ind0).route.L(ind1-1)-R*tan(abs(ac(ind0).route.delta_theta(ind1))/2);
xTurnEnd = ac(ind0).route.x(ind1-1) + tmp*cos(ac(ind0).route.course(ind1-1));
yTurnEnd = ac(ind0).route.y(ind1-1) + tmp*sin(ac(ind0).route.course(ind1-1));

% Turn direction
if ac(ind0).route.delta_theta(ind1) < 0
    % Turn is counter-clockwise
    startAngle = ac(ind0).route.course(ind1-1)+pi/2;
    endAngle = ac(ind0).route.course(ind1)+pi/2;
    
    % Turn center
    xTurnCenter = xTurnEnd + R*cos(ac(ind0).route.course(ind1-1)-pi/2);
    yTurnCenter = yTurnEnd + R*sin(ac(ind0).route.course(ind1-1)-pi/2);
    
    
    % Turn start
    xTurnStart = xTurnCenter + R*cos(ac(ind0).route.course(ind1)+pi/2);
    yTurnStart = yTurnCenter + R*sin(ac(ind0).route.course(ind1)+pi/2);
    
else
    % Turn is clockwise
    startAngle = ac(ind0).route.course(ind1-1)-pi/2;
    endAngle = ac(ind0).route.course(ind1)-pi/2;
    
    % Turn center
    xTurnCenter = xTurnEnd + R*cos(ac(ind0).route.course(ind1-1)+pi/2);
    yTurnCenter = yTurnEnd + R*sin(ac(ind0).route.course(ind1-1)+pi/2);
    
    
    % Turn start
    xTurnStart = xTurnCenter + R*cos(ac(ind0).route.course(ind1)-pi/2);
    yTurnStart = yTurnCenter + R*sin(ac(ind0).route.course(ind1)-pi/2);
    
end
end
