function dydt = seir1(t,y)
%seir1  Evaluate the seir for fixed parameters
%
%   See also ODE113, ODE23, ODE45.

%   Jacek Kierzenka and Lawrence F. Shampine
%   Copyright 1984-2014 The MathWorks, Inc.

N = 100000;
beta  = y(5); %2.2;
sigma = y(6); %1/5.2;
gamma = y(7); %1/2.3;

S = y(1);
E = y(2);
I = y(3);
R = y(4);

dydt = [(-beta*S*I/N); ...
        beta*S*I/N - sigma*E; ...
        sigma*E - gamma*I; ...
        gamma*I; ...
        0; ...
        0; ...
        0];