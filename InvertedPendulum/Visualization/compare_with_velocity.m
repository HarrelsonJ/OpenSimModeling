% Script: analyze_ankle_trials.m
%  1) Computes total ankle‐torque work for each trial
%  2) Compares “no‐exo” vs. “with‐exo” at matched initial angles
%  3) Plots initial‐angle vs. initial‐velocity scatter by exo condition

clear; clc; ...close all;

import org.opensim.modeling.*
import org.opensim.modeling.opensimMoco.*

% Load all successful solution files and plot initial ankle angle vs. velocity
solutionDir = fullfile('..','solutions');
files = dir(fullfile(solutionDir,'*.sto'));

% Exclude failed trials
files = files(~contains({files.name}, 'fail'));

nFiles = numel(files);
if nFiles == 0
    error('No valid .sto files found in %s.', solutionDir);
end

% Determine state indices from the first trajectory
firstTraj    = MocoTrajectory(fullfile(solutionDir, files(1).name));
stateNames   = firstTraj.getStateNames();
nStates      = stateNames.size();
idxValue0    = -1;
idxSpeed0    = -1;

for k = 0:nStates-1
    name = stateNames.get(k);
    if strcmp(name, '/jointset/Ankle/Ankle_Angle/value')
        idxValue0 = k;
    elseif strcmp(name, '/jointset/Ankle/Ankle_Angle/speed')
        idxSpeed0 = k;
    end
end

if idxValue0<0 || idxSpeed0<0
    error('Required state labels not found in trajectory.');
end

idxValue = idxValue0 + 1;   % convert to MATLAB 1-based
idxSpeed = idxSpeed0 + 1;

% Preallocate result vectors
initialAngle    = zeros(nFiles,1);
initialVelocity = zeros(nFiles,1);

% Determine exo flags
names  = {files.name};
isExo0 = contains(names, 'exo0');
isExo1 = contains(names, 'exo1');

% Extract initial conditions
for i = 1:nFiles
    traj = MocoTrajectory(fullfile(solutionDir, files(i).name));
    X    = traj.getStatesTrajectoryMat();
    
    initialAngle(i)    = rad2deg(X(1, idxValue));
    initialVelocity(i) = rad2deg(X(1, idxSpeed));
end

% Plot
figure('Name','Initial Angle vs. Velocity','NumberTitle','off');
hold on;
scatter(initialAngle(isExo0), initialVelocity(isExo0),  50, 'r', 'filled', ...
    'MarkerFaceAlpha', 0.6, 'MarkerEdgeAlpha', 0.6);
scatter(initialAngle(isExo1), initialVelocity(isExo1),  50, 'b', 'filled', ...
    'MarkerFaceAlpha', 0.6, 'MarkerEdgeAlpha', 0.6);
hold off;

xlabel('Initial Ankle Angle (°)');
ylabel('Initial Ankle Angular Velocity (°/s)');
title('Initial Conditions by Exo Condition');
legend({'No Exo (Unassisted)','With Exo (Assisted)'}, 'Location','best');
grid on;
