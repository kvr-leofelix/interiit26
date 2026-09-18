%% PS-07 Task B - Fine Tune Roll P Gain

clear;
clc;
close all;

%% Project setup

scriptPath = mfilename('fullpath');
simulationFolder = fileparts(scriptPath);
projectRoot = fileparts(simulationFolder);

addpath(fullfile(projectRoot, 'src'));

plotFolder = fullfile(projectRoot, 'results', 'plots');

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

%% Aircraft parameters

parameters;

%% Simulation

dt = 1/rate_hz;

% Longer simulation than previous test
simulation_time = 5;       % [s] analysis choice

time = 0:dt:simulation_time;
N = length(time);

stepTime = 0.5;
target = 1.0;

desired_roll = zeros(1,N);
desired_roll(time >= stepTime) = target;

%% Experimental Kp values

KpTests = [0.05 0.07 0.09 0.11 0.14];

numTests = length(KpTests);

allResponses = zeros(numTests,N);

%% Run simulations

for test = 1:numTests

    Kp = KpTests(test);

    rate = zeros(3,N);

    for k = 1:N-1

        error = desired_roll(k) - rate(1,k);

        % P-only controller
        tau_roll = Kp * error;

        torque = [tau_roll; 0; 0];

        rate_dot = rotational_dynamics( ...
            rate(:,k), ...
            torque, ...
            I);

        rate(:,k+1) = ...
            rate(:,k) + rate_dot*dt;

    end

    allResponses(test,:) = rate(1,:);

end

%% Calculate metrics

riseTime = NaN(numTests,1);
overshoot = NaN(numTests,1);
settlingTime = NaN(numTests,1);
steadyError = NaN(numTests,1);

for test = 1:numTests

    response = allResponses(test,:);

    %% Rise time

    idx10 = find( ...
        time >= stepTime & response >= 0.1*target, ...
        1, 'first');

    idx90 = find( ...
        time >= stepTime & response >= 0.9*target, ...
        1, 'first');

    if ~isempty(idx10) && ~isempty(idx90)

        riseTime(test) = ...
            time(idx90) - time(idx10);

    end

    %% Overshoot

    peak = max(response(time >= stepTime));

    overshoot(test) = ...
        max(0, ...
        (peak-target)/target*100);

    %% Settling time (+/-2%)

    lower = 0.98*target;
    upper = 1.02*target;

    afterStep = find(time >= stepTime);

    for k = afterStep

        remaining = response(k:end);

        if all(remaining >= lower & ...
               remaining <= upper)

            settlingTime(test) = ...
                time(k)-stepTime;

            break;

        end

    end

    %% Steady state error

    finalSamples = ...
        time >= simulation_time-0.2;

    steadyError(test) = ...
        target - mean(response(finalSamples));

end

%% Results table

results = table( ...
    KpTests', ...
    riseTime, ...
    overshoot, ...
    settlingTime, ...
    steadyError, ...
    'VariableNames', { ...
    'Kp', ...
    'RiseTime_s', ...
    'Overshoot_percent', ...
    'SettlingTime_s', ...
    'SteadyStateError_rad_s'});

disp(' ');
disp('===== ROLL P-GAIN FINE TUNING =====');
disp(results);

%% Plot

figure('Color','w');

plot( ...
    time, ...
    desired_roll, ...
    'k--', ...
    'LineWidth',2.5);

hold on;

colors = lines(numTests);

for test = 1:numTests

    plot( ...
        time, ...
        allResponses(test,:), ...
        'LineWidth',1.7, ...
        'Color',colors(test,:));

end

yline(1.02, ':', '102%');
yline(0.98, ':', '98%');

grid on;

xlabel('Time [s]');
ylabel('Roll rate [rad/s]');
title('Task B - Roll Kp Fine Tuning');

legend( ...
    'Desired', ...
    'Kp=0.05', ...
    'Kp=0.07', ...
    'Kp=0.09', ...
    'Kp=0.11', ...
    'Kp=0.14', ...
    'Location','best');

%% Save results safely

scriptPath = mfilename('fullpath');
simulationFolder = fileparts(scriptPath);
projectRoot = fileparts(simulationFolder);

resultsFolder = fullfile(projectRoot, 'results');
plotFolder = fullfile(resultsFolder, 'plots');

if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

%% Save plot

plotFile = fullfile( ...
    plotFolder, ...
    'task_B_roll_Kp_finetune.png');

exportgraphics( ...
    gcf, ...
    plotFile, ...
    'Resolution',300);

%% Save table

csvFile = fullfile( ...
    resultsFolder, ...
    'task_B_roll_Kp_finetune.csv');

writetable(results, csvFile);

fprintf('\nPlot saved to:\n%s\n', plotFile);
fprintf('\nTable saved to:\n%s\n', csvFile);
%% Save numbers

resultsFolder = fullfile(projectRoot,'results');

writetable( ...
    results, ...
    fullfile( ...
    resultsFolder, ...
    'task_B_roll_Kp_finetune.csv'));

fprintf('\nFine-tuning complete.\n');