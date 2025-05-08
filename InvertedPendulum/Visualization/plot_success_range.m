clear; clc; close all;

import org.opensim.modeling.*
import org.opensim.modeling.opensimMoco.*

solutionDirectory = 'solutions';
angleStateKey     = '/jointset/Ankle/Ankle_Angle/value';

% List all solution files
fileList  = dir(fullfile(solutionDirectory, '*.sto'));
numTrials = numel(fileList);
if numTrials == 0
    error('No solution files (*.sto) found in %s.', solutionDirectory);
end

% Preallocate
initialAngles = zeros(numTrials,1);   % in degrees
exoPresent    = false(numTrials,1);   % true if exo1
trialSuccess  = false(numTrials,1);   % true if no "fail" in filename

% Determine the 1‐based index of the angle state
firstTraj   = MocoTrajectory(fullfile(solutionDirectory, fileList(1).name));
stateNames  = firstTraj.getStateNames();
numStates   = stateNames.size();
angleIdx0   = -1;
for j = 0:numStates-1
    if strcmp(stateNames.get(j), angleStateKey)
        angleIdx0 = j;
        break;
    end
end
if angleIdx0 < 0
    error('State "%s" not found in first trajectory.', angleStateKey);
end
angleIdx = angleIdx0 + 1;

% Loop through each file and extract data
for i = 1:numTrials
    fn = fileList(i).name;
    traj = MocoTrajectory(fullfile(solutionDirectory, fn));
    
    % Initial ankle angle (convert radians to degrees)
    states = traj.getStatesTrajectoryMat();
    initialAngles(i) = rad2deg(states(1, angleIdx));
    
    % Exo flag
    exoPresent(i) = contains(fn, 'exo1');
    
    % Success flag
    trialSuccess(i) = ~contains(fn, 'fail');
end

yNoExo      = 0;
yExo        = 1;
markerSize  = 50;

figure; hold on;
for i = 1:numel(initialAngles)
    x = initialAngles(i);
    y = exoPresent(i)*yExo + (~exoPresent(i))*yNoExo;
    if ~trialSuccess(i)
        mkr = 'x';   clr = [0.5 0.5 0.5];
    else
        mkr = 'o';
        %clr = hasExo(i)==0 ? 'r' : 'b';
        if exoPresent(i); clr='b'; else; clr='r'; end
    end
    scatter(x, y, markerSize, mkr, 'MarkerEdgeColor', clr, 'LineWidth',1.5);
end

% Optional: connect the paired points at each angle
uniqueAngles = unique(initialAngles);
for ang = uniqueAngles'
    idx = find(abs(initialAngles-ang)<1e-6);
    if numel(idx)==2
        ys  = double(exoPresent(idx));
        xs  = repmat(ang,1,2);
        plot(xs, ys, 'k--', 'LineWidth',0.8);
    end
end

% Styling
yticks([yNoExo yExo]);
yticklabels({'No Exo','With Exo'});
ylim([yNoExo - 0.5, yExo + 0.5])
xlabel('Initial Ankle Angle (°)');
xlim([min(initialAngles)-1, max(initialAngles)+1]);
title('Success vs. Failure Across Exo Conditions');
legend( ...
    [ ...
      scatter(nan,nan,markerSize,'o','MarkerEdgeColor','r'), ...
      scatter(nan,nan,markerSize,'o','MarkerEdgeColor','b'), ...
      scatter(nan,nan,markerSize,'x','MarkerEdgeColor',[0.5 0.5 0.5]) ...
    ], ...
    {'Success, No Exo','Success, With Exo','Failure'}, ...
    'Location','best' );
grid on;
hold off;
