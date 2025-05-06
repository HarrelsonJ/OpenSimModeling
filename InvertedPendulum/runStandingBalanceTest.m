function result=runStandingBalanceTest(has_exo, use_muscles, include_ground_contacts, init_angle, init_speed)
    import org.opensim.modeling.*

    % Build model
    model = buildRealisticFootBody(has_exo, use_muscles, include_ground_contacts);
    
    % Alternatively load prebuilt model
    % exo_opts = {'unassisted','assisted'};
    % mus_opts = {'ideal','muscles'};
    % gc_opts  = {'fixed','free'};
    % 
    % filename = sprintf('%s_%s_%s_footbody.osim', ...
    %     exo_opts{has_exo+1}, ...
    %     mus_opts{use_muscles+1}, ...
    %     gc_opts{include_ground_contacts+1});
    %     model.initSystem();
    %     working_state = model.updWorkingState();
    % 
    % model = Model(filename);

    % Set up tracking problem
    study = MocoStudy();
    if has_exo
        study.setName("standing_balance_exo");
    else
        study.setName("standing_balance");
    end
    
    problem = study.updProblem();
    problem.setModel(model);

    
    finalTime = 5.0;
    problem.setTimeBounds(0, finalTime);
    problem.setStateInfo("/jointset/Ankle/Ankle_Angle/value",[deg2rad(-90), deg2rad(90)], deg2rad(init_angle), []);
    problem.setStateInfo("/jointset/Ankle/Ankle_Angle/speed",[deg2rad(-300), deg2rad(300)], deg2rad(init_speed), 0);
    problem.setStateInfo("/jointset/FootToGround/ground_rz/value",[], 0, []);
    problem.setStateInfo("/jointset/FootToGround/ground_rz/speed",[], 0, 0);
    
    problem.setStateInfo("/jointset/FootToGround/ground_ty/value",[], 0.04765, []);
    problem.setStateInfo("/jointset/FootToGround/ground_ty/speed",[], 0, 0);
    
    problem.setStateInfo("/jointset/FootToGround/ground_tz/value",[], 0, []);
    problem.setStateInfo("/jointset/FootToGround/ground_tz/speed",[], 0, 0);
    
    if use_muscles
        problem.setControlInfo("/forceset/dorsiflexion/activation", MocoBounds(0,1));
        problem.setControlInfo("/forceset/plantar_flexion/activation", MocoBounds(0,1));
    else
        problem.setControlInfo("/forceset/ankle_torque", MocoBounds(-50, 50))
    end

    % Create a time vector with 0.05 second intervals
    ref_angle.time = 0:0.05:finalTime;
    
    % Create a joint_angle field with the same length as time, filled with zeros
    % This needs to be in radians, 0 rads = 0 deg
    ref_angle.ankle_speed = zeros(length(ref_angle.time), 1);
    %ref_angle.ground_rz = zeros(length(ref_angle.time), 1);
    ref_angle.ground_rz_speed = zeros(length(ref_angle.time), 1);
    ref_angle.ground_ty_speed = zeros(length(ref_angle.time), 1);
    ref_angle.ground_tz_speed = zeros(length(ref_angle.time), 1);
    
    % Convert ref_angle struct to osim TimeSeriesTable
    table = osimTableFromStruct(ref_angle);
    osimlabels =  StdVectorString();
    osimlabels.add("/jointset/Ankle/Ankle_Angle/speed");
    %osimlabels.add("/jointset/FootToGround/ground_rz/value");
    osimlabels.add("/jointset/FootToGround/ground_rz/speed");
    osimlabels.add("/jointset/FootToGround/ground_ty/speed");
    osimlabels.add("/jointset/FootToGround/ground_tz/speed");
    table.setColumnLabels(osimlabels);
    tableProcessor = TableProcessor(table);
    
    stateGoal = MocoStateTrackingGoal("ref_tracking");
    stateGoal.setWeight(10);
    stateGoal.setReference(tableProcessor);
    %problem.addGoal(stateGoal)
    
    effortGoal = MocoControlGoal("effort", 1);
    problem.addGoal(effortGoal);
    
    solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    solver.set_transcription_scheme("hermite-simpson")
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);
    
    guess = solver.createGuess();
    %guess.randomizeAdd();
    solver.setGuess(guess);
    
    % If we haven't already solved.
    predictSolution = study.solve();
    if (predictSolution.success())
        filename = sprintf('solutions/test_exo%d_ang%g_vel%g.sto', has_exo, init_angle, init_speed);
        fprintf("Successful solution found printing to ")
        fprintf(filename)
        fprintf('\n')
        predictSolution.write(filename);
        % If we successfully solved visualize
        % Disable for batch runs
        % study.visualize(predictSolution);
    else
        filename = sprintf('solutions/test_fail_exo%d_ang%g_vel%g.sto', has_exo, init_angle, init_speed);
        fprintf("No successful solution found printing to ")
        fprintf(filename)
        fprintf('\n')
        % Unlock the failed solution
        predictSolution.unseal();
        predictSolution.write(filename);
        % study.visualize(predictSolution);
    end
end