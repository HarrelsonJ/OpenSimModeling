function plotStandingBalanceSolution(solFile)
% plotStandingBalanceSolution  Load a Moco .sto solution and plot:
%   • all states vs time (with legend)
%   • all controls vs time (with legend)
%   • exoskeleton torque vs time

    import org.opensim.modeling.*
    import org.opensim.modeling.opensimMoco.*

    [folder, base, ext] = fileparts(solFile);

    %% Load trajectory & model
    traj  = MocoTrajectory(solFile);
    model = Model("realisticFootBody.osim");

    states_traj = traj.getStatesTrajectory();
    
    %% Add force reporter
    reporter = ForceReporter();
    reporter.setName('ankleForces');
    model.addAnalysis(reporter);
    model.finalizeConnections();
    state = model.initSystem();

    %% Extract time, states, controls
    time         = traj.getTimeMat();               
    stateNamesJ  = traj.getStateNames();            
    X            = traj.getStatesTrajectoryMat();   
    controlNamesJ= traj.getControlNames();          
    U            = traj.getControlsTrajectoryMat(); 

    %% Extract and save ground reaction forces
    % contactSphere_heel = StdVectorString();
    % contactSphere_front = StdVectorString();
    % contactSphere_heel.add("Contact_Base_Ground_Heel");
    % contactSphere_heel.add("Contact_Base_Ground_Toe");
    % 
    % externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
    %                              traj,contactSphere_heel,contactSphere_front);
    % 
    % outfile = fullfile(folder, [base '_external_forces' ext]);
    % STOFileAdapter.write(externalForcesTableFlat, outfile);


    %% Build MATLAB labels
    nStates   = size(X,2);
    stateLabels   = cell(nStates,1);
    for i = 1:nStates
        stateLabels{i} = char(stateNamesJ.get(i-1));
    end

    nControls = size(U,2);
    controlLabels = cell(nControls,1);
    for i = 1:nControls
        controlLabels{i} = char(controlNamesJ.get(i-1));
    end

    %% Plot everything in 3 stacked subplots
    figure('Name','Standing Balance Solution','NumberTitle','off');

    % --- States ---
    ax1 = subplot(3,1,1);
    plot(ax1, time, X, 'LineWidth',1.2);
    title(ax1, 'States vs Time');
    xlabel(ax1, 'Time (s)');
    ylabel(ax1, 'State value');
    legend(ax1, stateLabels, 'Interpreter','none','Location','best');

    % --- Controls ---
    ax2 = subplot(3,1,2);
    plot(ax2, time, U, 'LineWidth',1.2);
    title(ax2, 'Controls vs Time');
    xlabel(ax2, 'Time (s)');
    ylabel(ax2, 'Control value');
    legend(ax2, controlLabels, 'Interpreter','none','Location','best');

    % --- Exo Torque ---
    ax3 = subplot(3,1,3);
    plot(ax3, time, exoForce, 'LineWidth',1.5);
    title(ax3, 'AnkleExo Torque vs Time');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'Torque (N)');
end
