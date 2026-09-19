%% PS-07 Task B - Learning PID Gains
% Compare different rate-controller gains on the SAME roll-rate command.

clear;
clc;
close all;

%% ---------------------------------------------------------
% Project folders
% ----------------------------------------------------------

scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    error('Save this script before running it.');
end

simulationFolder = fileparts(scriptPath);
projectRoot = fileparts(simulationFolder);

addpath(fullfile(projectRoot, 'src'));

plotFolder = fullfile(projectRoot, 'results', 'plots');

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

%% ---------------------------------------------------------
% Load aircraft parameters
% ----------------------------------------------------------

parameters;

%% ---------------------------------------------------------
% Simulation settings
% ---------------------------------------------------------

dt = 1 / rate_hz;           % 500 Hz - suggested by PS

simulation_time = 3;        % [s] TEST CHOICE
time = 0:dt:simulation_time;
N = length(time);

%% ---------------------------------------------------------
% Desired roll-rate command
% ---------------------------------------------------------

desired_roll = zeros(1,N);

% TEST CHOICE:
% Ask for 1 rad/s starting at t = 0.5 s

desired_roll(time >= 0.5) = 1.0;

%% ---------------------------------------------------------
% Experimental PID gain sets
% ---------------------------------------------------------
%
% IMPORTANT:
% These gains are NOT given by the problem statement.
% They are experimental values chosen to understand tuning.
%
% Each ROW is one experiment.
%
%           Kp       Ki       Kd

gainSets = [
             0.025    0        0;
             0.070    0        0;
             0.070    0.040    0;
             0.070    0.150    0;
             0.070    0.040    0.0025
           ];

testNames = {
    'Small P'
    'Larger P'
    'P + small I'
    'P + large I'
    'PID'
};

numTests = size(gainSets,1);

%% ---------------------------------------------------------
% Storage for results
% ---------------------------------------------------------

allRollRates = zeros(numTests,N);

%% ---------------------------------------------------------
% Run every controller
% ---------------------------------------------------------

for test = 1:numTests

    Kp_roll = gainSets(test,1);
    Ki_roll = gainSets(test,2);
    Kd_roll = gainSets(test,3);

    % Current angular rates:
    % [roll rate; pitch rate; yaw rate]

    rate = zeros(3,N);

    integralError = 0;
    previousError = 0;

    for k = 1:N-1

        %% Desired and actual roll rate

        desired = desired_roll(k);
        actual = rate(1,k);

        %% Error

        error = desired - actual;

        %% Integral

        integralError = integralError + error * dt;

        %% Derivative

        derivativeError = ...
            (error - previousError) / dt;

        %% PID output = requested roll torque

        tau_roll = ...
            Kp_roll * error ...
          + Ki_roll * integralError ...
          + Kd_roll * derivativeError;

        %% No pitch or yaw command in this experiment

        torque = [
            tau_roll;
            0;
            0
        ];

        %% Quadcopter rotational dynamics

        rate_dot = rotational_dynamics( ...
            rate(:,k), torque, I);

        %% Numerical integration

        rate(:,k+1) = ...
            rate(:,k) + rate_dot * dt;

        previousError = error;

    end

    allRollRates(test,:) = rate(1,:);

end

%% ---------------------------------------------------------
% Plot comparison
% ---------------------------------------------------------

figure('Color','w');

plot(time, desired_roll, ...
    'k--', ...
    'LineWidth', 2.5);

hold on;

colors = lines(numTests);

for test = 1:numTests

    plot(time, ...
        allRollRates(test,:), ...
        'LineWidth', 1.7, ...
        'Color', colors(test,:));

end

grid on;

xlabel('Time [s]');
ylabel('Roll rate [rad/s]');

title('Task B - Effect of PID Gains on Roll Rate');

legend( ...
    ['Desired'; testNames], ...
    'Location','best');

%% ---------------------------------------------------------
% Save plot
% ---------------------------------------------------------

saveFile = fullfile( ...
    plotFolder, ...
    'task_B_PID_learning.png');

exportgraphics( ...
    gcf, ...
    saveFile, ...
    'Resolution',300);

fprintf('\nPlot saved to:\n%s\n\n', saveFile);

%% ---------------------------------------------------------
% Print gains
% ---------------------------------------------------------

fprintf('PID TEST VALUES\n');
fprintf('-----------------------------------------\n');

for test = 1:numTests

    fprintf( ...
        '%s: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', ...
        testNames{test}, ...
        gainSets(test,1), ...
        gainSets(test,2), ...
        gainSets(test,3));

end