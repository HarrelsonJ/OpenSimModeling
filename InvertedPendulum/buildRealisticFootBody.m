function model=buildRealisticFootBody(has_exo, use_muscles, include_ground_contacts)
%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

exo_opts = {'unassisted','assisted'};
mus_opts = {'ideal','muscles'};
gc_opts  = {'fixed','free'};

% build the base name (everything except the extension)
modelName = sprintf('%s_%s_%s_footbody', ...
    exo_opts{has_exo+1}, ...
    mus_opts{use_muscles+1}, ...
    gc_opts{include_ground_contacts+1});

%% Constants as defined by "CENTER OF MASS VELOCITY-POSITION PREDICTIONS FOR BALANCE CONTROL"

body_mass = 80;                                     % kg
foot_mass = 2 * 0.0145 * body_mass;                 % kg
pendulum_mass = body_mass - foot_mass;              % kg

foot_moi = Inertia(1,1,1,0,0,0);                    % kg * m^2
pendulum_moi = Inertia(1,1,1,0,0,0);                % kg * m^2

body_height = 1.78;                                 % m
foot_length = 0.152 * body_height;                  % m
pendulum_length = 0.575 * body_height;              % m
ankle_to_heel = 0.19 * foot_length;                 % m
ankle_height = 0.039 * body_height;                 % m
ankle_to_com = 0.5 * foot_length - ankle_to_heel;   % m
ground_to_com = 0.0;                                % m


%% Build Model Bodies

model = Model();
model.setName(modelName);

% Define the acceleration of gravity
model.setGravity(Vec3(0, -9.80665, 0));

% Get a reference to the ground object
ground = model.getGround();

% Define foot
foot = Body();
foot.setName("Foot");
foot.setMass(foot_mass);
foot.setInertia(foot_moi);
 
% Define Pendulum
pendulum = Body();
pendulum.setName("Pendulum");
pendulum.setMass(pendulum_mass);
pendulum.setInertia(pendulum_moi);


%% Build Model Joints
if include_ground_contacts
    % which DOFs to keep
    has_dof = {'ground_rz','ground_ty','ground_tz'};
    
    % names for the six SpatialTransform axes
    coordNames = { ...
      'ground_rx','ground_ry','ground_rz', ...  % rotations
      'ground_tx','ground_ty','ground_tz' ...   % translations
    };
    
    % build the 6-axis SpatialTransform
    spatialTransform = SpatialTransform();
    for k = 1:6
        name = coordNames{k};
        axis = spatialTransform.updTransformAxis(k-1);
        axis.setName(name);
        
        if ismember(name, has_dof)
          % only these get a coordinate
          axis.append_coordinates(name);
          axis.setFunction( LinearFunction(1,0) );
        else
          % still need a transform function, but no coordinate
          axis.setFunction( Constant(0) );
        end
    end
    
    % create & add your CustomJoint exactly as before
    footToGround = CustomJoint( ...
        'FootToGround', ...
         ground, Vec3(0), Vec3(0),  ...
         foot,   Vec3(0), Vec3(0),  ...
         spatialTransform );
    model.addJoint(footToGround);
    
    % then configure the remaining coords (range/defaults) as usual
    for i = 1:numel(has_dof)
        coord = footToGround.upd_coordinates(i-1);
        % e.g. set ranges, defaults, etc.
        coord.setRange([-pi,pi]);
        coord.setDefaultValue(0);
        coord.setDefaultSpeedValue(0);
    end
else
% Add weld joint to model
    locationInParent    = Vec3(0,0,0);
    orientationInParent = Vec3(0,0,0);
    locationInChild     = Vec3(0,-ground_to_com,0);
    orientationInChild  = Vec3(0,0,0);
    footToGround = WeldJoint('BaseToGround', ground, locationInParent, ...
        orientationInParent, foot, locationInChild, orientationInChild);
    model.addJoint(footToGround);
end



% Add pin joint for ankle
locationInParent    = Vec3(-ankle_to_com,ankle_height,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-pendulum_length/2,0);
orientationInChild  = Vec3(0,0,0);
ankle_hinge = PinJoint("Ankle", foot, locationInParent, ...
    orientationInParent, pendulum, locationInChild, orientationInChild);

ankle_angle = ankle_hinge.upd_coordinates(0); % Rotation about z
ankle_angle.setRange([-pi/4, pi/4]);
ankle_angle.setName('Ankle_Angle');
ankle_angle.setDefaultValue(0);
ankle_angle.setDefaultSpeedValue(0);

model.addJoint(ankle_hinge);

%% Setup expression bassed force for exo
if has_exo
    ankleExo = ExpressionBasedCoordinateForce();
    ankleExo.set_coordinate(ankle_angle.getName());
    ankleExo.set_expression("(-401.6214*(-0.05392386353963117-q))-183.4599*qdot"); % kp = mgl, kd = a *sqrt(m*L*L*(mgl a = 1
    %ankleExo.set_expression("(-401.6214*(q))-183.4599*qdot"); % kp = mgl, kd = a *sqrt(m*L*L*(mgl a = 1
    ankleExo.setName('AnkleExo');
    model.addForce(ankleExo);
end

%% Add dorsiflexion and plantar flexion muscles if required, otherwise ideal torque actuators
if use_muscles
    % Use Thelen2003Muscles
    Fmax = 1500;      % [N]
    lopt = 0.1;       % [m]
    lts  = 0.2;       % [m]
    alpha = 0.0;      % [rad]
    
    dorsiflexor = Thelen2003Muscle('dorsiflexion', Fmax, lopt, lts, alpha);
    dorsiflexor.addNewPathPoint('origin', foot, Vec3(0, ankle_height / 2, 0));
    dorsiflexor.addNewPathPoint('insertion', pendulum,Vec3(0, -(8 * pendulum_length) / 20, 0));
    model.addForce(dorsiflexor);
    
    plantarflexar = Thelen2003Muscle('plantar_flexion', Fmax, lopt, lts, alpha);
    plantarflexar.addNewPathPoint('origin', foot, Vec3(-(ankle_to_com + (ankle_to_heel / 2)), ankle_height / 2, 0));
    plantarflexar.addNewPathPoint('insertion', pendulum,Vec3(0, 0, 0));
    model.addForce(plantarflexar);
else
    % Use ideal torque actuator
    ankle_torque = CoordinateActuator();
    ankle_torque.setCoordinate(ankle_angle)
    ankle_torque.setName('ankle_torque');
    ankle_torque.setOptimalForce(1.0);
    ankle_torque.setMinControl(-100);
    ankle_torque.setMaxControl(100);
    model.addForce(ankle_torque);
end



%% Add body geometries
foot_mesh = generateFootVTP( ...
    [-(ankle_to_com + ankle_to_heel),0], ...
    [foot_length-(ankle_to_com + ankle_to_heel),0], ...
    [-ankle_to_com, ankle_height], ...
    'wedge_foot.vtp');
footGeometry = Mesh(foot_mesh);
footGeometry.setColor( Vec3(0.8, 0.1, 0.1) );
foot.attachGeometry(footGeometry);

pendulumGeometry = Cylinder(0.03471, pendulum_length / 2);
pendulumGeometry.setColor( Vec3(0.8, 0.1, 0.1) );
pendulum.attachGeometry(pendulumGeometry);

model.addBody(foot);
model.addBody(pendulum);

%% Build ground contacts
if include_ground_contacts
    r_contactsphere = 0.035000000000000003; % m
    
    ground = model.getGround();
    groundContactLocation = Vec3(0,0.025,0);
    groundContactOrientation = Vec3(0,0,-1.57);
    groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                           groundContactOrientation,...
                                           ground);
    groundContactSpace.setName('GroundContact');
    model.addContactGeometry(groundContactSpace);
    
    heelSphereLocation = Vec3(-(ankle_to_heel + ankle_to_com) * 0.9, 0, 0);
    frontSphereLocation = Vec3((foot_length - (ankle_to_heel + ankle_to_com)) * 0.9, 0, 0);
    
    heelContactSphere = ContactSphere();
    heelContactSphere.setRadius(r_contactsphere);
    heelContactSphere.setLocation(heelSphereLocation);
    heelContactSphere.setFrame(foot)
    heelContactSphere.setName('Base_Ground_Heel');
    model.addContactGeometry(heelContactSphere);
    
    frontContactSphere = ContactSphere();
    frontContactSphere.setRadius(r_contactsphere);
    frontContactSphere.setLocation(frontSphereLocation);
    frontContactSphere.setFrame(foot)
    frontContactSphere.setName('Base_Ground_Toe');
    model.addContactGeometry(frontContactSphere);
     
    %% Define Contact Force Parameters - Curently all defaults
    stiffness           = 3067776;
    dissipation         = 0.4163;
    staticFriction      = 0.8;
    dynamicFriction     = 0.4;
    viscousFriction     = 0.4;
    transitionVelocity  = 0.1327;
    ConstantContactForce = 1e-5; % ConstantContactForce
    HertzSmoothing = 300; % HertzSmoothing
    HuntCrossleySmoothing = 50; % HuntCrossleySmoothing
    
    contactHeel = SmoothSphereHalfSpaceForce();
    contactHeel.setName('Contact_Base_Ground_Heel');
    contactHeel.connectSocket_sphere(heelContactSphere);
    contactHeel.connectSocket_half_space(groundContactSpace);
    
    contactHeel.set_stiffness(stiffness);
    contactHeel.set_dissipation(dissipation);
    contactHeel.set_static_friction(staticFriction);
    contactHeel.set_dynamic_friction(dynamicFriction);
    contactHeel.set_viscous_friction(viscousFriction);
    contactHeel.set_transition_velocity(transitionVelocity);
    model.upd_ComponentSet().addComponent(contactHeel);
     
    contactFront = SmoothSphereHalfSpaceForce();
    contactFront.setName('Contact_Base_Ground_Toe');
    contactFront.connectSocket_sphere(frontContactSphere);
    contactFront.connectSocket_half_space(groundContactSpace);
    
    contactFront.set_stiffness(stiffness);
    contactFront.set_dissipation(dissipation);
    contactFront.set_static_friction(staticFriction);
    contactFront.set_dynamic_friction(dynamicFriction);
    contactFront.set_viscous_friction(viscousFriction);
    contactFront.set_transition_velocity(transitionVelocity);
    model.upd_ComponentSet().addComponent(contactFront);
end

%% Initialize the System (checks model consistency).
model.finalizeConnections();

% Save the model to a file
filename = [modelName '.osim'];
model.print(filename);
fprintf([filename, ' printed!']);
end


