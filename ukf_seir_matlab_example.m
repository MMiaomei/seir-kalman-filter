clear; clc

N = 100000;                         % Population size.

% Initial state guess
                    % SEIR    beta,sigma,gamma
initialStateGuess = [N;5;0;0; 1;0.1;0.1];

% Construct the filter
ukf = unscentedKalmanFilter(...
    @seirStateFcn,...               % State transition function
    @seirMeasurementFcn,...         % Measurement function 
    initialStateGuess, ...
    'HasAdditiveProcessNoise', true);

%%%%
I_var = 10; % Variance of the measurement noise in I
ukf.MeasurementNoise = diag([I_var]);

% Process noise.
ukf.ProcessNoise = diag([0.1 0.1 0.1 0.0 0.0 0.0 0.0]);

T = 1;                              % 1 day step.
timeVector = 0:T:500;               % 500 days.
[~,xTrue]=ode45(@seir1,timeVector,[N;5;0;0;  0.5;1/5.2;1/2.3]);
%plot(xTrue)

% Simulate the observed data.

yTrue = xTrue(:,[3]);                                       % True I from SEIR simulation.
yMeas = yTrue + randn(size(yTrue))*sqrt(diag([I_var]));     % Add noise.

%%%% Run the Kalman filter.

Nsteps = size(yMeas,1);                 % Number of time steps
xCorrectedUKF = zeros(Nsteps,7);        % Corrected state estimates
PCorrected = zeros(Nsteps,7,7);         % Corrected state estimation error covariances
e = zeros(Nsteps,1);                    % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    e(k,:) = yMeas(k,:) - seirMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrected(k,:,:)] = correct(ukf,yMeas(k,:));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf);
end

%figure();
labels = {'S'; 'E'; 'I'; 'R'; 'beta'; 'sigma'; 'gamma'};

for k=1:length(ukf.State)
    subplot(length(ukf.State),1,k);
    plot(timeVector,xTrue(:,k),timeVector,xCorrectedUKF(:,k));
    ylabel(labels{k});
end
subplot(length(ukf.State),1,3);
hold on
plot(timeVector,yMeas(:,1));
hold off
legend('True','UKF estimate', 'Observation')