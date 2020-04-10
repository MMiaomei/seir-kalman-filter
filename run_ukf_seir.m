clear; clc

state = 'MA';

switch state
    
    case 'MA', u.N = 6939373;        % population of massachusetts
    case 'NY', u.N = 19453561;       % population of new york
    case 'CA', u.N = 39512223;       % population of california
    case 'NM', u.N = 2096829;        % population of new mexico
        
end

% Initial state guess.
                    %S; E; I; R; beta; 
initialStateGuess = [u.N; 0; 2; 0; 2.2];

% Construct the filter
ukf = unscentedKalmanFilter(...
    @seirStateFcn,...                % State transition function
    @seirMeasurementFcn,...          % Measurement function 
    initialStateGuess, ...
    'HasAdditiveProcessNoise', true);

% Define noise terms.
RI = 10;                           % Variance of the measurement noise in I
ukf.MeasurementNoise = diag([RI]);
                                    % Process noise.
                       % S E    I   R    beta
ukf.ProcessNoise = diag([0 1000 100 10   0.1]);

% Load the measurements
load(['data/' state '.mat']);
yMeas      = I;
timeVector = (1:length(I));

% Perform the filtering

Nsteps        = length(yMeas);      % Number of time steps
xCorrectedUKF = zeros(Nsteps,5);    % Corrected state estimates
PCorrected    = zeros(Nsteps,5,5);  % Corrected state estimation error covariances
xCorrectedstd = zeros(Nsteps,5,5);
e = zeros(Nsteps,1);                % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    % Residuals (or innovations): Measured output - Predicted output
    e(k) = yMeas(k) - seirMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrected(k,:,:)] = correct(ukf,yMeas(k));
    xCorrectedstd(k,:,:) = chol(ukf.StateCovariance);
    
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf, u);
    day_label{k} = string(datetime(d(k,[3,1,2])));
end

% Predict the future.
future_days = 20;                       % Steps in the future
xPredictedUKF = zeros(future_days,5);   % Predicted state estimates
xPredictedstd = zeros(future_days,5,5);
for k=1:future_days
    predict(ukf,u);
    xPredictedUKF(k,:) = ukf.State;
    xPredictedstd(k,:,:) = chol(ukf.StateCovariance);
end
day_label_future = string(datetime(d(end,[3,1,2]),'Format','dd-MMM-yyyy')+days(1:future_days));

%figure();
labels = {'x_1,S'; 'x_2,E'; 'x_3,I'; 'x_4,R'; 'x_5,beta'};
for k=1:5
    subplot(5,1,k);
    plot(timeVector,xCorrectedUKF(:,k), 'b', 'LineWidth', 2);
    hold on
    plot(timeVector,xCorrectedUKF(:,k)+2*squeeze(xCorrectedstd(:,k,k)), '--b','HandleVisibility','off')
    plot(timeVector,xCorrectedUKF(:,k)-2*squeeze(xCorrectedstd(:,k,k)), '--b','HandleVisibility','off')
    plot(timeVector(end)+(1:future_days), xPredictedUKF(:,k), 'k', 'LineWidth', 2)
    plot(timeVector(end)+(1:future_days), xPredictedUKF(:,k)+2*squeeze(xPredictedstd(:,k,k)), '--k','HandleVisibility','off')
    plot(timeVector(end)+(1:future_days), xPredictedUKF(:,k)-2*squeeze(xPredictedstd(:,k,k)), '--k','HandleVisibility','off')
    hold off
    ylabel(labels{k});
    axis tight
    grid on
end
subplot(5,1,1); title(['State = ' state])
subplot(5,1,3)
hold on
plot(timeVector,yMeas, 'o', 'Color', [0.6,0.6,0.6]);
hold off
[mx, imx] = max([xCorrectedUKF(:,3); xPredictedUKF(:,3)]);
hold on
plot([imx, imx], [yMeas(1), mx], 'r', 'LineWidth', 2)
hold off
ds = [day_label,day_label_future];
legend({'Corrected'; 'Predicted'; 'Observed'; ['Max ' num2str(mx,3) ' ' ds{imx}]}, 'Location', 'NorthWest')
subplot(5,1,5)
set(gca, 'XTick', (1:timeVector(end)+future_days))
set(gca, 'XTickLabel', [day_label,day_label_future])
xtickangle(90)
subplot(5,1,5)
title(['Last estimate of beta = ' num2str(xCorrectedUKF(end,5),3) ', std = ' num2str(squeeze(xCorrectedstd(end,5,5)),3)])

