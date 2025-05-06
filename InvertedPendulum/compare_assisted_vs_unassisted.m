% Script to plot control trajectories and ankle joint kinematics for all solutions
% Assumes solution files are in 'solutions' which is at the same level as
% this file
clear; clc; close all;

import org.opensim.modeling.*
import org.opensim.modeling.opensimMoco.*

%% Configuration
solutionDirectory = 'solutions';
stateKeyValue     = '/jointset/Ankle/Ankle_Angle/value';
stateKeySpeed     = '/jointset/Ankle/Ankle_Angle/speed';

%% Locate solution files
fileList = dir(fullfile(solutionDirectory, '*.sto'));
if isempty(fileList)
    error('No solution files (*.sto) found in %s.', solutionDirectory);
end
numFiles = numel(fileList);

%% Load all trajectories
trajectories = cell(numFiles,1);
for idx = 1:numFiles
    filePath = fullfile(solutionDirectory, fileList(idx).name);
    trajectories{idx} = MocoTrajectory(filePath);
end

%% Retrieve control names
controlNames = trajectories{1}.getControlNames();
numControls  = controlNames.size();

%% Determine state indices (convert 0-based to 1-based)
stateNames  = trajectories{1}.getStateNames();
numStates   = stateNames.size();
valueIndex0 = -1;
speedIndex0 = -1;

for j = 0:numStates-1
    name = stateNames.get(j);
    if strcmp(name, stateKeyValue)
        valueIndex0 = j;
    elseif strcmp(name, stateKeySpeed)
        speedIndex0 = j;
    end
end

if valueIndex0 < 0 || speedIndex0 < 0
    error('Required state keys not found in trajectory state names.');
end

valueIndex = valueIndex0 + 1;
speedIndex = speedIndex0 + 1;

%% Plot control trajectories with color annotation
for c = 1:numControls
    figure('Name', char(controlNames.get(c-1)), 'NumberTitle', 'off');
    hold on;
    
    controlLabel = char(controlNames.get(c-1));
    for idx = 1:numFiles
        traj        = trajectories{idx};
        timeData    = traj.getTimeMat();
        controlData = traj.getControlsTrajectoryMat();
        stateData   = traj.getStatesTrajectoryMat();
        initAngle   = rad2deg(stateData(1, valueIndex));
        fileName    = fileList(idx).name;
        
        if contains(fileName, 'fail')
            color = [0.5 0.5 0.5, 0.5];
            
        elseif contains(fileName, 'exo0')
            color = [1, 0, 0, 1];
        else
            color = [0, 0, 1, 1];
        end
        
        plot(timeData, controlData(:,c), 'Color', color);
    end

    annotation('textbox', [0.65 0.75 0.2 0.15], ...
        'String', {...
            'Red   : Unassisted (exo0)', ...
            'Blue  : Assisted   (exo1)', ...
            'Gray  : Failure'       }, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white', ...
        'EdgeColor', 'black');
    
    title(controlLabel, 'Interpreter', 'none');
    xlabel('Time (s)');
    ylabel('Control Input');
    %legend('Location', 'best');
    hold off;
end

%% Plot ankle joint angle and angular speed
figure('Name', 'Ankle Kinematics', 'NumberTitle', 'off');

% Ankle joint angle (degrees)
subplot(2,1,1);
hold on;
for idx = 1:numFiles
    traj      = trajectories{idx};
    timeData  = traj.getTimeMat();
    stateData = traj.getStatesTrajectoryMat();
    initAngle = rad2deg(stateData(1, valueIndex));
    fileName  = fileList(idx).name;

    if contains(fileName, 'fail')
        color = [0.5 0.5 0.5, 0.5];
    elseif contains(fileName, 'exo0')
        color = [1, 0, 0, 1];
    else
        color = [0, 0, 1, 1];
    end

    plot(timeData, rad2deg(stateData(:, valueIndex)), ...
         'Color', color);
end
title('Ankle Joint Angle (°)', 'Interpreter', 'none');
xlabel('Time (s)');
ylabel('Angle (°)');
%legend('Location', 'best');
hold off;

% Ankle joint angular speed (°/s)
subplot(2,1,2);
hold on;
for idx = 1:numFiles
    traj      = trajectories{idx};
    timeData  = traj.getTimeMat();
    stateData = traj.getStatesTrajectoryMat();
    initAngle = rad2deg(stateData(1, valueIndex));
    fileName  = fileList(idx).name;

    if contains(fileName, 'fail')
        color = [0.5 0.5 0.5, 0.5];
    elseif contains(fileName, 'exo0')
        color = [1, 0, 0, 1];
    else
        color = [0, 0, 1, 1];
    end

    plot(timeData, rad2deg(stateData(:, speedIndex)), ...
         'Color', color);
end
title('Ankle Joint Angular Speed (°/s)', 'Interpreter', 'none');
xlabel('Time (s)');
ylabel('Angular Speed (°/s)');
%legend('Location', 'best');
hold off;

