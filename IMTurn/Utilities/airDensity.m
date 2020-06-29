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

function [rho,P] = airDensity(h)

% Inputs: 
%   altitude, h (meters)
% Outputs:
%   air density, rho (kg/m^3)
%   air pressure, P (kg/m^2)

% Load constants
h_trop = physics.Hp_trop;
g = physics.g;
T0 = physics.T0;
rho0 = physics.rho0;
R = physics.R;
k_T = physics.k;
rho_trop = physics.rho_trop;
P_trop = physics.p_trop;
P0 = physics.p0;

% Find air temperature (Kelvin), density (kg/m^3), and pressure (kg/m^2)
T = getTemp(h);

if h < h_trop
    rho = rho0*(T./T0).^(-g/(k_T*R)-1);
    P = P0*(T./T0).^(-g/(k_T*R));
else
    rho = rho_trop*exp(-g/(R*T)*(h-h_trop));
    P = P_trop*exp(-g/(R*T)*(h-h_trop));
end