% Script to compute total ankle‐torque work for each trial and compare
% “no‐exo” vs. “with‐exo” at matched initial angles.
clear; clc; ...close all;

import org.opensim.modeling.*
import org.opensim.modeling.opensimMoco.*

%% Configuration
solutionDir    = 'solutions';
torqueKey      = '/forceset/ankle_torque';
angleKey       = '/jointset/Ankle/Ankle_Angle/value';
speedKey       = '/jointset/Ankle/Ankle_Angle/speed';

%% Gather solution files
fileList = dir(fullfile(solutionDir, '*.sto'));
numFiles = numel(fileList);
if numFiles == 0
    error('No solution files (*.sto) found in "%s".', solutionDir);
end

%% Preallocate data arrays
initialAngles = zeros(numFiles,1);   % [°]
exoPresent    = false(numFiles,1);   % true if exo1
trialSuccess  = false(numFiles,1);   % true if no 'fail' in filename
totalWork     = zeros(numFiles,1);   % [J]

%% Identify indices in the first trajectory
firstTraj = MocoTrajectory(fullfile(solutionDir, fileList(1).name));

% Control index for ankle torque
ctrlNames = firstTraj.getControlNames();
nCtrls    = ctrlNames.size();
torqueIdx0 = find(cellfun(@(j) strcmp(ctrlNames.get(j), torqueKey), num2cell(0:nCtrls-1)), 1) - 1;
if isempty(torqueIdx0)
    error('Control "%s" not found.', torqueKey);
end
torqueIdx = torqueIdx0 + 1;

% State indices for angle & speed
stateNames = firstTraj.getStateNames();
nStates    = stateNames.size();
angleIdx0  = find(cellfun(@(j) strcmp(stateNames.get(j), angleKey),  num2cell(0:nStates-1)), 1) - 1;
speedIdx0  = find(cellfun(@(j) strcmp(stateNames.get(j), speedKey),  num2cell(0:nStates-1)), 1) - 1;
if isempty(angleIdx0) || isempty(speedIdx0)
    error('Required state keys not found.');
end
angleIdx = angleIdx0 + 1;
speedIdx = speedIdx0 + 1;

%% Extract data from each file
for i = 1:numFiles
    fn   = fileList(i).name;
    traj = MocoTrajectory(fullfile(solutionDir, fn));
    
    t      = traj.getTimeMat();               % [Nt×1]
    U      = traj.getControlsTrajectoryMat(); % [Nt×Nc]
    X      = traj.getStatesTrajectoryMat();   % [Nt×Ns]
    
    % initial angle [°]
    initialAngles(i) = rad2deg(X(1, angleIdx));
    
    % flags
    exoPresent(i)   = contains(fn, 'exo1');
    trialSuccess(i) = ~contains(fn, 'fail');
    
    % compute total work = ∫ τ·ω dt
    tau   = U(:, torqueIdx);
    omega = X(:, speedIdx);
    power = tau .* omega;                    
    totalWork(i) = trapz(t, power);
end

%% Pair‐wise match on identical initial angles
uniqueAngles = unique(initialAngles);
pairedAngles = [];
workNoExo    = [];
workExo      = [];
succNoExo    = [];
succExo      = [];

for ang = uniqueAngles'
    idx = find(abs(initialAngles - ang) < 1e-6);
    if numel(idx) == 2
        i0 = idx(~exoPresent(idx));  % no-exo
        i1 = idx( exoPresent(idx));  % with-exo
        pairedAngles(end+1,1) = ang;
        workNoExo(end+1,1)    = totalWork(i0);
        workExo(end+1,1)      = totalWork(i1);
        succNoExo(end+1,1)    = trialSuccess(i0);
        succExo(end+1,1)      = trialSuccess(i1);
    end
end

% Convert to logical masks
idxSuccNoExo = logical(succNoExo);
idxFailNoExo = ~idxSuccNoExo;
idxSuccExo   = logical(succExo);
idxFailExo   = ~idxSuccExo;

%% Plot total work comparison with success/failure markers
figure('Name','Total Ankle-Torque Work','NumberTitle','off');
hold on;

% Connecting lines
plot(pairedAngles, workNoExo, '-', 'Color',[1 0 0 0.25], 'LineWidth',1.5, ...
    'DisplayName','Power Curve No Exo');
plot(pairedAngles, workExo,   '-', 'Color',[0 0 1 0.25], 'LineWidth',1.5, ...
    'DisplayName','Power Curve with Exo');

% Markers: No Exo
scatter(pairedAngles(idxSuccNoExo), workNoExo(idxSuccNoExo), 100, ...
        'o', 'MarkerEdgeColor','r', 'DisplayName','No Exo Success');
scatter(pairedAngles(idxFailNoExo), workNoExo(idxFailNoExo), 100, ...
        'x', 'MarkerEdgeColor','r', 'DisplayName','No Exo Failure');

% Markers: With Exo
scatter(pairedAngles(idxSuccExo), workExo(idxSuccExo), 100, ...
        'o', 'MarkerEdgeColor','b', 'DisplayName','With Exo Success');
scatter(pairedAngles(idxFailExo), workExo(idxFailExo), 100, ...
        'x', 'MarkerEdgeColor','b', 'DisplayName','With Exo Failure');

% Labels & styling
xlabel('Initial Ankle Angle (°)');
ylabel('Total Work (J)');
title('Total Ankle-Torque Work vs. Initial Angle');
legend('Location','best');
grid on;
hold off;