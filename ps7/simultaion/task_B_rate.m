%% PS-07 Task B - Rate Controller

clear;
clc;
close all;

scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    error('Please save taskA_mixer.m before running it.');
end
%%% Allow MATLAB to find files in src/
%scriptFolder = 'C:\Users\rasto\OneDrive\Desktop\interiit\ps7\src';
%projectRoot = 'C:\Users\rasto\OneDrive\Desktop\interiit\ps7';

%% Find project folders automatically
scriptFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptFolder);
addpath(fullfile(projectRoot, 'src'));

%% Load aircraft parameters

parameters;

%% Simulation settings

dt = 1 / rate_hz;       % 500 Hz controller
simulation_time = 5;    % seconds

time = 0:dt:simulation_time;
N = length(time);

%% Desired rate

rate_desired = zeros(3, N);

% Step command starts at 0.5 seconds

rate_desired(1, time >= 0.5) = 1.0;   % roll rate [rad/s]

%% Controller gains

% IMPORTANT:
% These are INITIAL TUNING VALUES, not values given by the PS.
% They are starting points and must be judged from the response.


%format of setting pid
%              Roll     Pitch     Yaw
% Kp =        [ Kp_r     Kp_p     Kp_y ]
% Ki =        [ Ki_r     Ki_p     Ki_y ]
% Kd =        [ Kd_r     Kd_p     Kd_y ]


%Kp = [0.08; 0.08; 0.12];
%Ki = [0.02; 0.02; 0.02];
%Kd = [0.001; 0.001; 0.002];

%test 1
%Kp=[0.03;0.03;0.03]; Ki=[0;0;0]; Kd=[0;0;0];
%test 2
%Kp=[0.015;0.15;0.15]; Ki=[0;0;0]; Kd=[0;0;0];
%test 3
%Kp=[0.5;0.5;0.5]; Ki=[0;0;0]; Kd=[0;0;0];
%test 3
%Kp=[0.9;0.9;0.9]; Ki=[0;0;0]; Kd=[0;0;0];
%test 4
%Kp=[0.09;0.09;0.18]; Ki=[0.05;0.05;0.05]; Kd=[0;0;0];
%test 5
%Kp=[0.08;0.08;0.08]; Ki=[0.20;0.20;0.20]; Kd=[0;0;0];
%test 6
Kp=[0.08;0.08;0.08]; Ki=[0.05;0.05;0.05]; Kd=[0.003;0.003;0.003];

%% Storage

rate = zeros(3, N);
torque_history = zeros(3, N);

integral_error = zeros(3,1);
prev_error = zeros(3,1);

%% Simulation

for k = 1:N-1

    desired = rate_desired(:,k);
    actual = rate(:,k);

    error = desired - actual;

    integral_error = integral_error + error * dt;

    torque = rate_controller( ...
        desired, ...
        actual, ...
        Kp, Ki, Kd, ...
        integral_error, ...
        prev_error, ...
        dt);

    torque_history(:,k) = torque;

    %% Drone rotational dynamics

    rate_dot = rotational_dynamics( ...
        actual, torque, I);

    % Numerical integration
    rate(:,k+1) = rate(:,k) + rate_dot * dt;

    prev_error = error;

end

%% Plot Roll Rate

figure;

plot(time, rate_desired(1,:), ...
    '--', 'LineWidth', 2);

hold on;

plot(time, rate(1,:), ...
    'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('Roll Rate [rad/s]');
title('Task B - Roll Rate Step Response');

legend('Desired', 'Actual');

%% Save

plotFolder = fullfile(projectRoot, 'results', 'plots');

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

saveas(gcf, fullfile(plotFolder, ...
    'task_B_roll_rate.png'));

fprintf('Task B roll-rate simulation complete.\n');