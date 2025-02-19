%% Clear Workspace
clear all; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%%
footLength = .26; % m
footWidth = 0.097; % m
footHeight = 0.05; % m

beamLength = 0.03; % m
beamWidth = footWidth;% m
beamHeight = 1.00; % m

%% Instantiate an (empty) OpenSim Model
osimModel = Model();
osimModel.setName('InvertedPendulumModel');

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
osimModel.setGravity(Vec3(0, -9.80665, 0));

% Define Base
base = Body();
base.setName("Base");
base.setMass(1);
base.setInertia( Inertia(1,1,1,0,0,0) );
 
% Add geometry to the body
baseGeometry = Brick( Vec3(footLength/2,footHeight/2, footWidth/2) );
baseGeometry.setColor( Vec3(0.8, 0.1, 0.1) );
base.attachGeometry(baseGeometry);

% Add Body to the Model
osimModel.addBody(base);

% Section: Create the Platform Joint
% Make and add a Weld joint for the base Body
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-footHeight/2,0);
orientationInChild  = Vec3(0,0,0);
baseToGround = WeldJoint('BaseToGround', ground, locationInParent, ...
    orientationInParent, base, locationInChild, orientationInChild);

osimModel.addJoint(baseToGround);

% Define beam
beam = Body();
beam.setName('Beam');
beam.setMass(5);
beam.setInertia( Inertia(1,1,1,0,0,0) );

% Add geometry to the body
beamGeometry = Brick( Vec3(beamLength/2,beamHeight/2, beamWidth/2) );
beamGeometry.setColor( Vec3(0.8, 0.1, 0.1) );
beam.attachGeometry(beamGeometry);

osimModel.addBody(beam);

% Make and add a pinjoint to connect the beam to the base
locationInParent    = Vec3(-footLength/2,footHeight/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-beamHeight/2,0);
orientationInChild  = Vec3(0,0,0);
beamToBase = PinJoint("BeamToBase", base, locationInParent, ...
    orientationInParent, beam, locationInChild, orientationInChild);

Angle_rz = beamToBase.upd_coordinates(0); % Rotation about z
Angle_rz.setRange([-pi/2, pi/2]);
Angle_rz.setName('Angle_rz');
Angle_rz.setDefaultValue(0);
Angle_rz.setDefaultSpeedValue(0);


osimModel.addJoint(beamToBase);

%% Ankle Exo
torqueAxis = Vec3(0, 0, 1.0);
ankleExo = TorqueActuator();
ankleExo.setName('AnkleExo');
ankleExo.setBodyA(base);
ankleExo.setBodyB(beam);
ankleExo.setAxis(Vec3(0, 0, 1)); % Torque around Z-axis
ankleExo.setOptimalForce(1.0);

% Add actuator to model
osimModel.addForce(ankleExo);


%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
osimModel.print('InvertedPendulumModel.osim');
display(['InvertedPendulumModel.osim printed!']);
