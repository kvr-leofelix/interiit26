%% PS-07 TASK D - BREAK BANDWIDTH SEPARATION

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

%% Aircraft parameters

parameters;

%% Inner rate-loop gain from Task B

Kp_rate = 0.11;

%% ---------------------------------------------------------
% Attitude gains to compare
% ---------------------------------------------------------
%
% IMPORTANT:
% These are experimental test gains, NOT values from PS.
%
% We deliberately increase the attitude-loop gain to demonstrate
% what happens when the outer loop becomes too aggressive.

Kp_att_tests = [
    1.5
    3
    8
    20
    50
];

testNames = {
    'Katt = 1.5'
    'Katt = 3'
    'Katt = 8'
    'Katt = 20'
    'Katt = 50'
};

numTests = length(Kp_att_tests);

%% Simulation settings

dt = 1/rate_hz;

simulationTime = 3;

time = 0:dt:simulationTime;
N = length(time);

%% Desired roll angle

desiredAngle = zeros(1,N);

desiredAngle(time >= 0.5) = deg2rad(10);

%% Store results

allAngles = zeros(numTests,N);
allRates = zeros(numTests,N);

%% ---------------------------------------------------------
% Run tests
% ---------------------------------------------------------

for test = 1:numTests

    Kp_att = Kp_att_tests(test);

    angle = zeros(1,N);
    rate = zeros(1,N);

    for k = 1:N-1

        %% Outer ATTITUDE controller

        angleError = ...
            desiredAngle(k) - angle(k);

        desiredRate = ...
            Kp_att * angleError;

        %% Inner RATE controller

        rateError = ...
            desiredRate - rate(k);

        torque = ...
            Kp_rate * rateError;

        %% Roll dynamics

        angularAcceleration = ...
            torque / Ixx;

        rate(k+1) = ...
            rate(k) ...
            + angularAcceleration*dt;

        angle(k+1) = ...
            angle(k) ...
            + rate(k)*dt;

    end

    allAngles(test,:) = angle;
    allRates(test,:) = rate;

end

%% ---------------------------------------------------------
% Plot attitude
% ---------------------------------------------------------

figure('Color','w');

subplot(2,1,1);

plot( ...
    time, ...
    rad2deg(desiredAngle), ...
    'k--', ...
    'LineWidth',2.5);

hold on;

colors = lines(numTests);

for test = 1:numTests

    plot( ...
        time, ...
        rad2deg(allAngles(test,:)), ...
        'Color',colors(test,:), ...
        'LineWidth',1.6);

end

grid on;

ylabel('Roll angle [deg]');

title('Task D - Breaking Loop Separation');

legend( ...
    ['Desired'; testNames], ...
    'Location','best');

%% Plot angular rates

subplot(2,1,2);

for test = 1:numTests

    plot( ...
        time, ...
        allRates(test,:), ...
        'Color',colors(test,:), ...
        'LineWidth',1.6);

    hold on;

end

grid on;

xlabel('Time [s]');
ylabel('Roll rate [rad/s]');

title('Inner-Loop Rate Demand Behaviour');

%% ---------------------------------------------------------
% Save
% ---------------------------------------------------------

plotFile = fullfile( ...
    plotFolder, ...
    'task_D_break_separation.png');

exportgraphics( ...
    gcf, ...
    plotFile, ...
    'Resolution',300);

fprintf('\nTask D simulation complete.\n');
fprintf('Saved: %s\n',plotFile);