%% PS-07 Task A - Quadcopter Mixer Test

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



%% Load parameters
parameters;

%% Yaw coefficient

% IMPORTANT:
% The problem statement does NOT provide a yaw torque coefficient.
%
% For these Task-A demonstrations only, we use a NORMALIZED TEST VALUE.


kYaw = 0.02;    % [m] NORMALIZED/ASSUMED TEST VALUE

fprintf('\nIMPORTANT:\n');
fprintf('kYaw = %.3f m is an ASSUMED test value.\n', kYaw);
fprintf('It is NOT supplied by the problem statement.\n\n');

%% ---------------------------------------------------------
% TEST 1: Pure total thrust
% ----------------------------------------------------------

T = 16;                 % [N]
tau_roll  = 0;
tau_pitch = 0;
tau_yaw   = 0;

[F, Fraw] = mixer(T, tau_roll, tau_pitch, tau_yaw, ...
                  L, kYaw, Tmax);

disp('TEST 1 - Pure thrust');
disp('Rotor thrusts [N]:');
disp(F');

%% ---------------------------------------------------------
% TEST 2: Roll command
% ----------------------------------------------------------

T = 16;
tau_roll  = 0.5;       % [N*m]
tau_pitch = 0;
tau_yaw   = 0;

[Froll, ~] = mixer(T, tau_roll, tau_pitch, tau_yaw, ...
                   L, kYaw, Tmax);

disp('TEST 2 - Roll');
disp('Rotor thrusts [N]:');
disp(Froll');

%% ---------------------------------------------------------
% TEST 3: Pitch command
% ----------------------------------------------------------

T = 16;
tau_roll  = 0;
tau_pitch = 0.5;       % [N*m]
tau_yaw   = 0;

[Fpitch, ~] = mixer(T, tau_roll, tau_pitch, tau_yaw, ...
                    L, kYaw, Tmax);

disp('TEST 3 - Pitch');
disp('Rotor thrusts [N]:');
disp(Fpitch');

%% ---------------------------------------------------------
% TEST 4: Yaw command
% ----------------------------------------------------------

T = 16;
tau_roll  = 0;
tau_pitch = 0;
tau_yaw   = 0.05;      % [N*m]

[Fyaw, ~] = mixer(T, tau_roll, tau_pitch, tau_yaw, ...
                  L, kYaw, Tmax);

disp('TEST 4 - Yaw');
disp('Rotor thrusts [N]:');
disp(Fyaw');

%% ---------------------------------------------------------
% TEST 5: Saturation
% Ask for more thrust than four motors can provide.
% ----------------------------------------------------------

T = 40;                 % Requested total thrust [N]
tau_roll  = 0;
tau_pitch = 0;
tau_yaw   = 0;

[Fsat, FrawSat] = mixer(T, tau_roll, tau_pitch, tau_yaw, ...
                        L, kYaw, Tmax);

disp('TEST 5 - Saturation');
disp('Requested rotor thrust before saturation [N]:');
disp(FrawSat');

disp('Actual rotor thrust after saturation [N]:');
disp(Fsat');

%% ---------------------------------------------------------
% Plot all tests
% ----------------------------------------------------------

results = [F, Froll, Fpitch, Fyaw, Fsat];

figure;

bar(results');

grid on;
xlabel('Test');
ylabel('Rotor thrust [N]');
title('Task A - Mixer Tests');

legend('M1', 'M2', 'M3', 'M4', ...
       'Location', 'best');

xticklabels({'Pure Thrust', ...
             'Roll', ...
             'Pitch', ...
             'Yaw', ...
             'Saturation'});

yline(Tmax, '--r', '8 N Rotor Limit');

%% Save plot

plotFolder = fullfile(projectRoot, 'results', 'plots');

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

saveas(gcf, fullfile(plotFolder, 'task_A_mixer.png'));

fprintf('\nPlot saved to results/plots/task_A_mixer.png\n');