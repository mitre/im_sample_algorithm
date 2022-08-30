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

classdef physics
    properties (Constant)
        g = 9.81    % gravitational constant m/s^2

        % adiabatic index of air, unitless
        k = 1.4;

        % standard atmospheric temp at MSL, K
        T0 = 288.15;

        % standard atmospheric pressure at MSL, Pa
        p0 = 101325;
        
        % tropopause pressure, p_trop = p0*(Ttrop/T0)^(g/(beta_tLT*R))
        p_trop = 101325*((288.15 + -0.0065*11000)/288.15)^(9.81/(-0.0065*287.05287));
        
        % standard atmostpheric density at MSL, kg/m^3
        rho0 = 1.225;

        % speed of sound, m/s
        a0 = 240.294;
    
        % real gas constant for air, m2/(K*s^2)
        R = 287.05287;
        
        % ISA temp gradient with alt below tropopause, K/m
        beta_tLT = -0.0065;

        % Tropopause altitude, m
        Hp_trop = 11000; 
        
        % Tropopause temperature, T_trop = T0 + beta*Hp_trop
        T_trop = 288.15 + -0.0065*11000;

        mu = 1/3.5; % assumes k = 1.4
        
        % WGS 84 flattening GRS80
        f = 0.003352810681183637418;
        
        % WGS 84 GRS80 semi-major axis
        a = 6378137; % meters
        
        % Density of the tropopause at 11,000 meters
        rho_trop = 0.3639; % kg/m^3

    end
end