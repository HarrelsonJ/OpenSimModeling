%% Plot solution data

filename = 'predictSolution.sto';

% Detect import options for a text file with tab-separated values.
% This should automatically handle the header rows.
opts = detectImportOptions(filename, 'FileType', 'text', 'Delimiter', '\t');
opts.DataLines = [19 Inf];

% (Optional) Display the detected variable names and header row number.
disp('Detected variable names:');
disp(opts.VariableNames);
disp(['Variable names are assumed to be in row: ', num2str(opts.VariableNamesLine)]);

% Read the data into a table.
data = readtable(filename, opts);
varNames = data.Properties.VariableNames;

% Extract time and data columns.
time = data{:,1};       % First column always time
terms = data{:,2:end};  % Remaining columns are the data terms

% Create a new figure for the plot.
figure;

% Plot each term using a loop (MATLAB will automatically cycle through colors).
hold on;
for i = 1:size(terms, 2)
    plot(time, terms(:, i), 'LineWidth', 2);
end
hold off;

% Use the variable names (excluding the time column) for the legend.
legend(varNames(2:end), 'Location', 'best', 'Interpreter', 'none');

% Label the axes using the variable names.
xlabel(varNames{1}, 'Interpreter', 'none');
ylabel('Data Values', 'Interpreter', 'none');

% Add a title and enable grid for better visualization.
title('Plot of Data from predictSolution.sto');
grid on;