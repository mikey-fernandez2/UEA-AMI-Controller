% Mikey Fernandez
% 05/26/2021
%
% Log Processing Code

clc; clear; close all;

%% Enter log filename
logName = 'EMGLong.csv';

%% Extract Values
logDir = '/home/haptix-e15-463/haptix/haptix_controller/logs';
fullFile = fullfile(logDir, logName);

data = readmatrix(fullFile);

% default params
numElec = 16;
numPoses = 4;

% This had to be handled manually - no way to read in the CSV data in mixed format nicely
usingEMG = data(1, 2); usingPolhemus = data(1, 4);
numMotors = data(2, 2); numJoints = data(2, 4); numContactSensors = data(2, 6); numIMUS = data(2, 8); updateRate = data(2, 10);
actuatedJointLimits = data(5:5 + numMotors - 1, 2:3);

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
        trackers(:, :, i) = loggedData(:, polhemusBase + 6*(i - 1):polhemusBase + 6*i);
    end
elseif (usingPolhemus && usingEMG)
    polhemusBase = jointStart + numJoints + 3*numElec + 6;
    trackers = zeros(size(loggedData, 1), 6, numPoses);
    for i = 1:numPoses
        trackers(:, :, i) = loggedData(:, polhemusBase + 6*(i - 1):polhemusBase + 6*i);
    end
end

%% Make Plots
% plot motor reference against motor position
figure(1)
sgtitle('Motor Position and Command')
for i = 1:numMotors
    subplot(floor((numMotors + 1)/2), 2, i)
    plot(times, motorPos(:, i), 'b', times, ref_pos(:, i), 'b--')
    xlim([times(1) times(end)])
    ylabel(['Motor ' num2str(i)])
    xlabel('Time (s)')
end
legend({'Position', 'Command'})
set(gcf, 'Position', get(0, 'Screensize'));


figure(2)
sgtitle('Joint Position')
for i = 1:numJoints
    subplot(floor((numJoints + 1)/2), 2, i)
    plot(times, jointPos(:, i), 'b')
    xlim([times(1) times(end)])
    ylabel(['Joint ' num2str(i)])
    xlabel('Time (s)')
end
set(gcf, 'Position', get(0, 'Screensize'));

if (usingEMG)
    figure(3)
    sgtitle('Normalized EMG')
    for i = 1:numElec
        subplot(4, 4, i)
        plot(times, normedEMG(:, i))
        xlim([times(1) times(end)])
        xlabel('Time (s)')
        ylabel(['Electrode ' num2str(i)])
    end
    set(gcf, 'Position', get(0, 'Screensize'));
    
    figure(4)
    sgtitle('Muscle Activation')
    for i = 1:numElec
        subplot(4, 4, i)
        plot(times, muscleAct(:, i))
        xlim([times(1) times(end)])
        xlabel('Time (s)')
        ylabel(['Electrode ' num2str(i)])
    end
    set(gcf, 'Position', get(0, 'Screensize'));
end

if (usingPolhemus)
    figure(5)
    sgtitle('Tracker Positions')
    
    for i = 1:numPoses
        quat = eul2quat([trackers(:, 4, i), trackers(:, 5, i), trackers(:, 6, i)], 'ZYX');
        subplot(2, 2, i)
        quiver3(trackers(:, 1, i), trackers(:, 2, i), trackers(:, 3, i), quat(:, 1), quat(:, 2), quat(:, 3))
        xlabel('x (m)')
        ylabel('y (m)')
        zlabel('z (m)')
    end
    set(gcf, 'Position', get(0, 'Screensize'));
end