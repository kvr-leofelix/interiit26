%% ---------------------------------------------------------
% Calculate performance metrics
% ---------------------------------------------------------

% We analyse only after the step happens.
stepTime = 0.5;          % [s]
target = 1.0;            % [rad/s]

% Conventional definitions used here:
% Rise time     = 10% -> 90%
% Settling band = +/- 2%
%
% NOTE:
% These definitions are our analysis choices,
% not requirements given directly by the PS.

riseTimes = NaN(numTests,1);
overshoots = NaN(numTests,1);
settlingTimes = NaN(numTests,1);
steadyStateErrors = NaN(numTests,1);

for test = 1:numTests

    response = allRollRates(test,:);

    %% Rise time: 10% to 90%

    idx10 = find( ...
        time >= stepTime & response >= 0.10*target, ...
        1, 'first');

    idx90 = find( ...
        time >= stepTime & response >= 0.90*target, ...
        1, 'first');

    if ~isempty(idx10) && ~isempty(idx90)
        riseTimes(test) = time(idx90) - time(idx10);
    end

    %% Overshoot

    peakValue = max(response(time >= stepTime));

    overshoots(test) = ...
        max(0, (peakValue - target) / target * 100);

    %% Settling time (+/- 2%)

    lowerLimit = 0.98 * target;
    upperLimit = 1.02 * target;

    afterStep = find(time >= stepTime);

    settleIndex = NaN;

    for k = afterStep

        remainingResponse = response(k:end);

        if all(remainingResponse >= lowerLimit & ...
               remainingResponse <= upperLimit)

            settleIndex = k;
            break;

        end
    end

    if ~isnan(settleIndex)
        % Time measured relative to when command was applied
        settlingTimes(test) = ...
            time(settleIndex) - stepTime;
    end

    %% Steady-state error

    % Average final 0.2 seconds instead of using one sample.
    finalMask = time >= (simulation_time - 0.2);

    finalValue = mean(response(finalMask));

    steadyStateErrors(test) = target - finalValue;

end

%% ---------------------------------------------------------
% Make results table
% ---------------------------------------------------------

resultsTable = table( ...
    string(testNames), ...
    gainSets(:,1), ...
    gainSets(:,2), ...
    gainSets(:,3), ...
    riseTimes, ...
    overshoots, ...
    settlingTimes, ...
    steadyStateErrors, ...
    'VariableNames', { ...
    'Controller', ...
    'Kp', ...
    'Ki', ...
    'Kd', ...
    'RiseTime_s', ...
    'Overshoot_percent', ...
    'SettlingTime_s', ...
    'SteadyStateError_rad_s'});

disp(' ');
disp('=========== PID COMPARISON ===========');
disp(resultsTable);
%--------------------------------
%Now csv file to store the data 
%% ---------------------------------------------------------
% Save numerical results as CSV
% ---------------------------------------------------------

resultsFolder = fullfile(projectRoot, 'results');

if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

csvFile = fullfile( ...
    resultsFolder, ...
    'task_B_PID_metrics.csv');

writetable(resultsTable, csvFile);

fprintf('\nMetrics saved to:\n%s\n', csvFile);