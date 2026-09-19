%% PS-07 TASK C1 - ATTITUDE + RATE CASCADE

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

if ~exist(plotFolder,'dir')
    mkdir(plotFolder);
end

%% Aircraft

parameters;

%% Inner RATE controller
% From Task B

Kp_rate = [0.11;
           0.11;
           0.21];

%% Outer ATTITUDE controller

% IMPORTANT:
% Experimental initial gains, not provided by PS.

Kp_att = [3.0;
          3.0;
          2.0];

%% Simulation

dt = 1/rate_hz;

simulation_time = 4;

time = 0:dt:simulation_time;
N = length(time);

%% States

angle = zeros(3,N);
rate  = zeros(3,N);

%% Command

desiredAngle = zeros(3,N);

% Roll command = 10 degrees at 0.5 s
%
% Test command, NOT aircraft parameter.

desiredAngle(1,time >= 0.5) = deg2rad(10);

%% Simulation loop

for k = 1:N-1

    %% ATTITUDE LOOP

    desiredRate = attitude_controller( ...
        desiredAngle(:,k), ...
        angle(:,k), ...
        Kp_att);

    %% RATE LOOP

    rateError = ...
        desiredRate - rate(:,k);

    torque = ...
        Kp_rate .* rateError;

    %% AIRCRAFT rotational dynamics

    rateDot = rotational_dynamics( ...
        rate(:,k), ...
        torque, ...
        I);

    rate(:,k+1) = ...
        rate(:,k) + rateDot*dt;

    %% Angular rate -> angle

    angle(:,k+1) = ...
        angle(:,k) + rate(:,k)*dt;

end

%% Plot

figure('Color','w');

plot( ...
    time, ...
    rad2deg(desiredAngle(1,:)), ...
    'k--', ...
    'LineWidth',2);

hold on;

plot( ...
    time, ...
    rad2deg(angle(1,:)), ...
    'b', ...
    'LineWidth',2);

grid on;

xlabel('Time [s]');
ylabel('Roll angle [deg]');

title('Task C - Cascaded Roll Attitude Control');

legend('Desired','Actual');

%% Save

plotFile = fullfile( ...
    plotFolder, ...
    'task_C_attitude_roll.png');

exportgraphics(gcf, ...
    plotFile, ...
    'Resolution',300);

fprintf('\nAttitude simulation complete.\n');