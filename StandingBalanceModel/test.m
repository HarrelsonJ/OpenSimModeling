close all; clear; clc;
import org.opensim.modeling.*;

model = buildStandingBalanceModel();

jointSet = model.getJointSet();

componentList = jointSet.getComponentsList();

value_string = java.lang.String('/value');
speed_string = java.lang.String('/speed');
accel_string = java.lang.String('/accel');

label_concats = [
    value_string,...
    speed_string,...
    ...accel_string
];
labels = [];
n_joints = jointSet.getSize();
for j = 0:(n_joints - 1)
    % First loop through each joint
    joint = jointSet.get(j);
    joint.getName();
    n_coords = joint.numCoordinates;
    for c = 0:(n_coords - 1)
        % Second loop through each coordinate
        coordinate = joint.get_coordinates(c);
        path = coordinate.getAbsolutePathString();
        for l = 1:length(label_concats)
            % Create all 3 path strings for each coordinate
            labels = [labels, path.concat(label_concats(l))];
        end
    end
end

for l = 1:length(labels)
    
end

disp(labels)


