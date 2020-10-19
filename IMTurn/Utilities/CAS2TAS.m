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

function [Vtas,rho,P] = CAS2TAS(Vcas,h)

% Converts CAS to TAS
% Inputs:
%   calibrate air speed, Vcas (meters/sec)
%   altitude, h (meters)
% Outputs:
%   true air speed, Vtas (m/s)
    
rho0 = physics.rho0;
P0 = physics.p0;
mu = physics.mu;

% Get the air density
[rho,P] = airDensity(h);

Vtas = zeros(size(Vcas));
for i = 1:length(h)
    % Terms in the conversion
    temp1 = 1 + mu/2*(rho0/P0)*Vcas.^2;
    
    temp2 = temp1.^(1/mu) - 1;
    
    temp3 = (1 + P0./P(i)*temp2).^mu;
    
    temp4 = 2./mu.*P(i)./rho(i).*(temp3-1);
    
    Vtas(i) = sqrt(temp4);
end
