function x = seirStateFcn(x) 
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
dt = 0.5;                               % [days] Sample time
x = x + seirStateFcnContinuous(x)*dt;
x = x + seirStateFcnContinuous(x)*dt;
end

function dxdt = seirStateFcnContinuous(x)
%seirStateFcnContinuous Evaluate the SEIR for fixed parameters.
sigma = 1/5.2;
gamma = 1/2.3;
N = 6939373;

S = x(1);
E = x(2);
I = x(3);
R = x(4);
beta  = x(5);

dxdt = [(-beta*S*I/N); ...
        beta*S*I/N - sigma*E; ...
        sigma*E - gamma*I; ...
        gamma*I; ...
        0];

end