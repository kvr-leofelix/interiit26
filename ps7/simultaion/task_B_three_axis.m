%% PS-07 TASK B - FINAL THREE-AXIS RATE CONTROLLER
% Roll, pitch and yaw rate step responses

clear;
clc;
close all;

%% Project setup

scriptPath = mfilename('fullpath');
simulationFolder = fileparts(scriptPath);
projectRoot = fileparts(simulationFolder);

addpath(fullfile(projectRoot,'src'));

resultsFolder = fullfile(projectRoot,'results');
plotFolder = fullfile(resultsFolder,'plots');

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

if ~exist(plotFolder,'dir')
    mkdir(plotFolder);
end

%% Load aircraft parameters

parameters;

%% Controller gains
%
% PRELIMINARY TUNED VALUES:
%
% Roll Kp = 0.11 from roll tuning experiment.
% Pitch uses same Kp because Ixx = Iyy.
% Yaw is initially scaled using Izz/Ixx.
%
% Ki and Kd are zero because they did not improve the
% ideal rate model in our previous experiment.

Kp = [0.11;
      0.11;
      0.21];

Ki = [0;
      0;
      0];

Kd = [0;
      0;
      0];

%% Simulation settings

dt = 1/rate_hz;           % 500 Hz from PS starting point
simulation_time = 3;      % test choice [s]

time = 0:dt:simulation_time;
N = length(time);

stepTime = 0.5;           % test choice [s]
target = 1.0;             % test choice [rad/s]

%% Storage

responses = zeros(3,N);

axisNames = {'Roll','Pitch','Yaw'};

%% Run each axis separately

for axis = 1:3

    rate = zeros(3,N);

    integralError = zeros(3,1);
    previousError = zeros(3,1);

    for k = 1:N-1

        %% Desired rate

        desired = zeros(3,1);

        if time(k) >= stepTime
            desired(axis) = target;
        end

        %% Error

        error = desired - rate(:,k);

        integralError = ...
            integralError + error*dt;

        derivativeError = ...
            (error - previousError)/dt;

        %% PID controller

        torque = ...
              Kp .* error ...
            + Ki .* integralError ...
            + Kd .* derivativeError;

        %% Rotational dynamics

        rateDot = rotational_dynamics( ...
            rate(:,k), ...
            torque, ...
            I);

        %% Integrate angular acceleration

        rate(:,k+1) = ...
            rate(:,k) + rateDot*dt;

        previousError = error;

    end

    responses(axis,:) = rate(axis,:);

end

%% Desired signal for plots

desiredPlot = zeros(1,N);
desiredPlot(time >= stepTime) = target;

%% Calculate performance metrics

riseTime = NaN(3,1);
overshoot = NaN(3,1);
settlingTime = NaN(3,1);
steadyError = NaN(3,1);

for axis = 1:3

    response = responses(axis,:);

    %% Rise time: 10% -> 90%

    idx10 = find( ...
        time >= stepTime & ...
        response >= 0.1*target, ...
        1,'first');

    idx90 = find( ...
        time >= stepTime & ...
        response >= 0.9*target, ...
        1,'first');

    if ~isempty(idx10) && ~isempty(idx90)

        riseTime(axis) = ...
            time(idx90)-time(idx10);

    end

    %% Overshoot

    peak = max(response(time >= stepTime));

    overshoot(axis) = ...
        max(0,(peak-target)/target*100);

    %% Settling time (+/-2%)

    lower = 0.98*target;
    upper = 1.02*target;

    indices = find(time >= stepTime);

    for k = indices

        remaining = response(k:end);

        if all(remaining >= lower & ...
               remaining <= upper)

            settlingTime(axis) = ...
                time(k)-stepTime;

            break;

        end

    end

    %% Steady state error

    finalMask = ...
        time >= simulation_time-0.2;

    steadyError(axis) = ...
        target - mean(response(finalMask));

end

%% Results table

results = table( ...
    string(axisNames'), ...
    Kp, ...
    Ki, ...
    Kd, ...
    riseTime, ...
    overshoot, ...
    settlingTime, ...
    steadyError, ...
    'VariableNames',{ ...
    'Axis', ...
    'Kp', ...
    'Ki', ...
    'Kd', ...
    'RiseTime_s', ...
    'Overshoot_percent', ...
    'SettlingTime_s', ...
    'SteadyStateError_rad_s'});

disp(' ');
disp('===== TASK B - THREE AXIS RESULTS =====');
disp(results);

%% Plot all three axes

figure('Color','w');

for axis = 1:3

    subplot(3,1,axis);

    plot( ...
        time, ...
        desiredPlot, ...
        'k--', ...
        'LineWidth',2);

    hold on;

    plot( ...
        time, ...
        responses(axis,:), ...
        'b', ...
        'LineWidth',2);

    yline(1.02,':','102%');
    yline(0.98,':','98%');

    grid on;

    ylabel('Rate [rad/s]');

    title( ...
        sprintf('%s Rate Response', ...
        axisNames{axis}));

    legend('Desired','Actual', ...
        'Location','best');

end

xlabel('Time [s]');

sgtitle('Task B - Roll, Pitch and Yaw Rate Control');

%% Save graph

plotFile = fullfile( ...
    plotFolder, ...
    'task_B_three_axis_rate.png');

exportgraphics( ...
    gcf, ...
    plotFile, ...
    'Resolution',300);

%% Save table

csvFile = fullfile( ...
    resultsFolder, ...
    'task_B_three_axis_metrics.csv');

writetable(results,csvFile);

fprintf('\nGraph saved: %s\n',plotFile);
fprintf('Metrics saved: %s\n',csvFile);