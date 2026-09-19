%% PS-07 TASK C - POSITION -> VELOCITY -> ATTITUDE -> RATE

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

g = 9.81;  % gravitational acceleration [m/s^2]

%% ---------------------------------------------------------
% Controller gains
% ---------------------------------------------------------

% Inner rate controller from Task B
Kp_rate = 0.11;

% Attitude controller
Kp_att = 3.0;

% INITIAL TEST VALUES:
% These position and velocity gains are NOT given by the PS.
% They are starting tuning values.

Kp_vel = 1.2;
Kp_pos = 0.8;

%% ---------------------------------------------------------
% Simulation
% ---------------------------------------------------------

dt = 1/rate_hz;

simulationTime = 10;

time = 0:dt:simulationTime;
N = length(time);

%% State variables

x = zeros(1,N);              % position [m]
vx = zeros(1,N);             % velocity [m/s]

pitch = zeros(1,N);          % pitch angle [rad]
pitchRate = zeros(1,N);      % pitch rate [rad/s]

%% ---------------------------------------------------------
% Position command
% ---------------------------------------------------------

desiredX = zeros(1,N);

% TEST COMMAND:
% At t = 1 s ask drone to move to x = 5 m.

desiredX(time >= 1) = 5;

%% ---------------------------------------------------------
% Simulation loop
% ---------------------------------------------------------

for k = 1:N-1

    %% 1. POSITION CONTROLLER
    %
    % Position error -> desired velocity

    desiredVx = position_controller( ...
        desiredX(k), ...
        x(k), ...
        Kp_pos);

    %% Limit demanded velocity

    % ASSUMED SOFTWARE LIMIT for this simulation.
    % Prevent outer loop requesting unrealistic speed.

    maxVelocity = 3;       % [m/s]

    desiredVx = min( ...
        max(desiredVx,-maxVelocity), ...
        maxVelocity);

    %% 2. VELOCITY CONTROLLER
    %
    % Velocity error -> desired pitch

    desiredPitch = velocity_controller( ...
        desiredVx, ...
        vx(k), ...
        Kp_vel, ...
        g);

    %% Limit tilt angle

    % ASSUMED TEST LIMIT:
    % Keep small-angle model reasonable.

    maxPitch = deg2rad(20);

    desiredPitch = min( ...
        max(desiredPitch,-maxPitch), ...
        maxPitch);

    %% 3. ATTITUDE CONTROLLER
    %
    % Pitch error -> desired pitch rate

    pitchError = ...
        desiredPitch - pitch(k);

    desiredPitchRate = ...
        Kp_att * pitchError;

    %% 4. RATE CONTROLLER
    %
    % Pitch-rate error -> pitch torque

    rateError = ...
        desiredPitchRate - pitchRate(k);

    pitchTorque = ...
        Kp_rate * rateError;

    %% 5. ROTATIONAL DYNAMICS

    pitchAcceleration = ...
        pitchTorque / Iyy;

    pitchRate(k+1) = ...
        pitchRate(k) ...
        + pitchAcceleration*dt;

    pitch(k+1) = ...
        pitch(k) ...
        + pitchRate(k)*dt;

    %% 6. TRANSLATIONAL DYNAMICS

    % Small-angle approximation:
    %
    % forward acceleration ~= g * pitch

    ax = g * pitch(k);

    vx(k+1) = ...
        vx(k) + ax*dt;

    x(k+1) = ...
        x(k) + vx(k)*dt;

end

%% ---------------------------------------------------------
% Plots
% ---------------------------------------------------------

figure('Color','w');

%% Position

subplot(3,1,1);

plot(time,desiredX, ...
    'k--','LineWidth',2);

hold on;

plot(time,x, ...
    'b','LineWidth',2);

grid on;

ylabel('Position [m]');
title('Position');

legend('Desired','Actual');

%% Velocity

subplot(3,1,2);

plot(time,vx, ...
    'LineWidth',2);

grid on;

ylabel('Velocity [m/s]');
title('Forward Velocity');

%% Pitch

subplot(3,1,3);

plot(time,rad2deg(pitch), ...
    'LineWidth',2);

grid on;

xlabel('Time [s]');
ylabel('Pitch [deg]');
title('Pitch Angle');

sgtitle( ...
    'Task C - Position → Velocity → Attitude → Rate');

%% ---------------------------------------------------------
% Save
% ---------------------------------------------------------

plotFile = fullfile( ...
    plotFolder, ...
    'task_C_position_control.png');

exportgraphics( ...
    gcf, ...
    plotFile, ...
    'Resolution',300);

fprintf('\nTask C position simulation complete.\n');
fprintf('Final position = %.3f m\n',x(end));
fprintf('Plot saved: %s\n',plotFile);