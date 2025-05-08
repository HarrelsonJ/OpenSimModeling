% find_ankle_outliers.m
% Detects outlier ankle angles at t = 2 s and reports each trial’s initial
% ankle angle and whether an exoskeleton was present.
clear; clc; close all;

import org.opensim.modeling.*
import org.opensim.modeling.opensimMoco.*

%% Locate solution files
files   = dir('../solutions_old/*.sto');
nFiles  = numel(files);
if nFiles == 0
    error('No .sto files found in ../solutions.');
end

%% Determine index of the ankle-angle state
firstTraj  = MocoTrajectory(fullfile(files(1).folder, files(1).name));
stateNames = firstTraj.getStateNames();
angleIdx0  = -1;
for j = 0:stateNames.size()-1
    if strcmp(stateNames.get(j), '/jointset/Ankle/Ankle_Angle/value')
        angleIdx0 = j;
        break;
    end
end
if angleIdx0 < 0
    error('State "/jointset/Ankle/Ankle_Angle/value" not found.');
end
angleIdx = angleIdx0 + 1;

%% Preallocate result arrays
anglesAt2s    = nan(nFiles,1);   % Ankle angle at t = 2 s [degrees]
initialAngles = nan(nFiles,1);   % Initial ankle angle [degrees]
exoPresent    = false(nFiles,1); % True if exoskeleton was used

%% Extract data for each trial
for i = 1:nFiles
    fn   = files(i).name;
    traj = MocoTrajectory(fullfile(files(i).folder, fn));
    t    = traj.getTimeMat();
    X    = traj.getStatesTrajectoryMat();
    
    % Record ankle angle at nearest time to 2 seconds
    [~, idxT]       = min(abs(t - 2));
    anglesAt2s(i)   = rad2deg(X(idxT, angleIdx));
    
    % Record initial ankle angle
    initialAngles(i) = rad2deg(X(1, angleIdx));
    
    % Determine exoskeleton presence
    exoPresent(i) = contains(fn, 'exo1');
end

%% Identify outliers using interquartile method
isOutlier = isoutlier(anglesAt2s);

%% Display outlier information
outIdx = find(isOutlier);
if isempty(outIdx)
    fprintf('No outlier ankle angles detected at t = 2 s.\n');
else
    fprintf('Detected outliers at t = 2 s:\n');
    fprintf('%-30s %-10s %-14s %-5s\n', ...
            'Filename', 'Angle@2s', 'InitialAngle', 'Exo');
    for k = outIdx.'
        exoStr = ternary(exoPresent(k), 'Yes', 'No');
        fprintf('%-30s %8.2f° %12.2f°   %3s\n', ...
                files(k).name, anglesAt2s(k), initialAngles(k), exoStr);
    end
end

% Helper: inline ternary function
function out = ternary(cond, trueVal, falseVal)
    if cond
        out = trueVal;
    else
        out = falseVal;
    end
end
