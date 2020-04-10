clear; clc

N = 1000;

% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
initialStateGuess = [N;5;0;0;1]; % xhat[k|k-1]
% Construct the filter
ukf = unscentedKalmanFilter(...
    @seirStateFcn,...                % State transition function
    @seirMeasurementFcn,...         % Measurement function 
    initialStateGuess, ...
    'HasAdditiveProcessNoise', true);

%%%%
R1 = 10000;
R3 = 100; % Variance of the measurement noise v[k]
ukf.MeasurementNoise = diag([R1 R3]);

ukf.ProcessNoise = diag([1 1 1 1 0.01]);

T = 0.5; % [s] Filter sample time
timeVector = 0:T:100;
[~,xTrue]=ode45(@seir1,timeVector,[N;5;0;0;2.2]);
plot(xTrue)

%%%%

%rng(1); % Fix the random number generator for reproducible results
yTrue = xTrue(:,[1,3]);
yMeas = yTrue + randn(size(yTrue))*sqrt(diag([R1 R3])); % sqrt(R): Standard deviation of noise

%%%%

Nsteps = size(yMeas,1); % Number of time steps
xCorrectedUKF = zeros(Nsteps,5); % Corrected state estimates
PCorrected = zeros(Nsteps,5,5); % Corrected state estimation error covariances
e = zeros(Nsteps,2); % Residuals (or innovations)

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
subplot(5,1,1);
plot(timeVector,xTrue(:,1),timeVector,xCorrectedUKF(:,1),timeVector,yMeas(:,1));
legend('True','UKF estimate','Measured')
%ylim([-2.6 2.6]);
ylabel('x_1, S');
subplot(5,1,2);
plot(timeVector,xTrue(:,2),timeVector,xCorrectedUKF(:,2));
%ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_2, E');

subplot(5,1,3);
plot(timeVector,xTrue(:,3),timeVector,xCorrectedUKF(:,3),timeVector,yMeas(:,2));
%ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_3, I');

subplot(5,1,4);
plot(timeVector,xTrue(:,4),timeVector,xCorrectedUKF(:,4));
%ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_4, R');

subplot(5,1,5);
plot(timeVector,xTrue(:,5),timeVector,xCorrectedUKF(:,5));
%ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_5, R0');
