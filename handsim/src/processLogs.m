% Mikey Fernandez
% 05/26/2021
%
% Log Processing Code

%% Enter log filename
logName = 'testCSV.csv';

%% Extract Values
logDir = '/home/haptix-e15-463/haptix/haptix_controller/logs';
fullFile = fullfile(logDir, logName);

data = readmatrix(fullFile);

% default params
numElec = 16;
numPoses = 4;

% This had to be handled manually - no way to read in the CSV data in mixed
% format nicely

usingEMG = data(1, 2); usingPolhemus = data(1, 4);
numMotors = data(2, 2); numJoints = data(2, 4); numContactSensors = data(2, 6); numIMUS = data(2, 8); updateRate = data(2, 10);
jointLimits = data(5:5 + numMotors - 1, 2:3);

if (usingEMG)
    emgBase = 5 + numMotors + 1;
    Tau_a = data(emgBase, 2); Tau_d = data(emgBase, 4);
    omega_n = data(emgBase + 1, 2);

    elecBounds = data(emgBase + 4:emgBase + 4 + numElec - 1, 2:3);
    deltaBounds = data(emgBase + 6 + numElec:emgBase + 6 + 1.5*numElec - 1, 2:3);

    dataStart = emgBase + 8 + 1.5*numElec;
else
    dataStart = 5 + numMotors + 2;
end

loggedData = data(dataStart:end - 2, :);
times = loggedData(:, 1);

ref_pos = loggedData(:, 2:2:2*(numMotors - 1) + 2);
motorPos = loggedData(:, 3:2:2*(numMotors - 1) + 3);

jointStart = 2*(numMotors - 1) + 4;
jointPos = loggedData(:, jointStart:jointStart + numJoints - 1);

if (usingEMG)
    emgBase = jointStart + numJoints;
    samplingFreq = loggedData(1, emgBase);
    trigger = loggedData(:, emgBase + 1);
    switch1 = loggedData(:, emgBase + 2);
    switch2 = loggedData(:, emgBase + 3);

    rawEMG = loggedData(:, emgBase + 4:3:emgBase + 4 + 3*numElec - 1);
    normedEMG = loggedData(:, emgBase + 5:3:emgBase + 5 + 3*numElec - 1);
    muscleAct = loggedData(:, emgBase + 6:3:emgBase + 6 + 3*numElec - 1);
elseif (usingPolhemus && ~usingEMG)
    polhemusBase = jointStart + numJoints;
    trackers = zeros(size(loggedData, 1), 6, numPoses);
    for i = 1:numPoses
        trackers(:, :, i) = loggedData(:, polhemusBase + 6*(i - 1):poplhemusBase + 6*i);
    end
elseif (usingPolhemus && usingEMG)
    polhemusBase = jointStart + numJoints + 3*numElec + 6;
    trackers = zeros(size(loggedData, 1), 6, numPoses);
    for i = 1:numPoses
        trackers(:, :, i) = loggedData(:, polhemusBase + 6*(i - 1):poplhemusBase + 6*i);
    end
end

%% Make Plots