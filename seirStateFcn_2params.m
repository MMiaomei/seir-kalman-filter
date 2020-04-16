function x = seirStateFcn_2params(x,u) 
% seirStateFcn Discrete-time approximation to SEIR for fixed parameters
% Sample time is 0.5 day.
%
% Example state transition function for discrete-time nonlinear state
% estimators.
%
% xk1 = seirStateFcn(xk)
%
% Inputs:
%    xk - States x[k]
%
% Outputs:
%    xk1 - Propagated states x[k+1]
%
% See also extendedKalmanFilter, unscentedKalmanFilter

%   Copyright 2016 The MathWorks, Inc.

%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

% Euler integration of continuous-time dynamics x'=f(x) with sample time dt
dt = 0.5;                               % [days] Sample time is 1/2 day
x = x + seirStateFcnContinuous(x,u)*dt;   % ... step forward 2x 1/2 day
x = x + seirStateFcnContinuous(x,u)*dt;   % ... is 1 day.
end

function dxdt = seirStateFcnContinuous(x,u)
%seirStateFcnContinuous Evaluate the SEIR for fixed parameters.

N = u.N; %6939373;            % MA population
sigma = 1/5.2;          % Incubation rate [1/days]
%gamma = 1/2.3;          % Recovery rate [1/days]

S = x(1);
E = x(2);
I = x(3);
R = x(4);
beta  = x(5);           % Rate of spread
gamma = x(6);

dxdt = [(-beta*S*I/N); ...
        beta*S*I/N - sigma*E; ...
        sigma*E - gamma*I; ...
        gamma*I; ...
        0; ...
        0];

end