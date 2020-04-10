function dydt = seir1(t,y,u)
%seir1  Evaluate the seir for fixed parameters
%
%   See also ODE113, ODE23, ODE45.

%   Jacek Kierzenka and Lawrence F. Shampine
%   Copyright 1984-2014 The MathWorks, Inc.

N = u.N; %6939373;            % MA population
sigma = 1/5.2;          % Incubation rate [1/days]
gamma = 1/2.3;          % Recovery rate [1/days]

S = y(1);
E = y(2);
I = y(3);
R = y(4);
beta  = y(5);           % Rate of spread 

dydt = [(-beta*S*I/N); ...
        beta*S*I/N - sigma*E; ...
        sigma*E - gamma*I; ...
        gamma*I; ...
        0];