%% Reset workspace
close all; clear; clc;
import org.opensim.modeling.*

% Set up tracking problem
study = MocoStudy();
study.setName("inverted_pendulum_balancing");

problem = study.updProblem();

problem.setModel(buildInvertedPendulumModel());


finalTime = 5.0;
problem.setTimeBounds(0, finalTime);
problem.setStateInfo("/jointset/BeamToBase/ankle_angle/value",[-90, 90], -5, 0);
problem.setStateInfo("/jointset/BeamToBase/ankle_angle/speed",[-300, 300], 0, 0);
problem.setControlInfo("/forceset/AnkleExo", [-400, 400]);

% Create a time vector from 0 to 5 seconds with 0.05 second intervals
ref_angle.time = 0:0.05:finalTime;

% Create a joint_angle field with the same length as time, filled with zeros
ref_angle.joint_angle = zeros(length(ref_angle.time), 1);
ref_angle.joint_speed = zeros(length(ref_angle.time), 1);

% Convert ref_angle struct to osim TimeSeriesTable
table = osimTableFromStruct(ref_angle);
osimlabels =  StdVectorString();
osimlabels.add("/jointset/BeamToBase/ankle_angle/value");
osimlabels.add("/jointset/BeamToBase/ankle_angle/speed");
table.setColumnLabels(osimlabels);
table.addTableMetaDataString("inDegrees", "yes");
tableProcessor = TableProcessor(table);

stateGoal = MocoStateTrackingGoal("ref_tracking");
stateGoal.setReference(tableProcessor);
problem.addGoal(stateGoal)

effortGoal = MocoControlGoal("effort", 0.001);
problem.addGoal(effortGoal);

solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(100);
solver.set_transcription_scheme("hermite-simpson")
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_convergence_tolerance(1e-3);

guess = solver.createGuess(); guess.randomizeAdd();
solver.setGuess(guess);

% If we haven't already solved.
if ~exist('predictSolution.sto', 'file')
    predictSolution = study.solve();
    if (predictSolution.success())
        fprintf("Successful solution found")
        predictSolution.write('predictSolution.sto');
        % If we successfully solved visualize
        study.visualize(predictSolution);
    else
        fprintf("No successful solution found")
        % Unlock the failed solution
        predictSolution.unseal();
        predictSolution.write('predictSolution.sto');
    end
else
    disp('Solution file already exists');
end
