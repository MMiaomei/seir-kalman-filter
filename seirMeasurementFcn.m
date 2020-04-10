function yk = seirMeasurementFcn(xk)
% seirMeasurementFcn Example measurement function for discrete-time
% nonlinear state estimators. 
%
% The measurement is the first state (S) and third state (I)
%
% yk = seirMeasurementFcn(xk)
%
% Inputs:
%    xk - x[k], states at time k
%
% Outputs:
%    yk - y[k], measurements at time k
%
% See also extendedKalmanFilter, unscentedKalmanFilter

%   Copyright 2016 The MathWorks, Inc.

%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

% We measure only the number of infected.
S = xk(1);
E = xk(2);
I = xk(3);
R = xk(4);

yk = [I];
end