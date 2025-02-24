%% Clear Workspace
clear all; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%%
footLength = 1.0; % m
footWidth = 0.4; % m
footHeight = 0.20; % m

beamLength = 0.12; % m
beamWidth = footWidth;% m
beamHeight = 4.00; % m

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
locationInChild     = Vec3(beamLength/2,-beamHeight/2,0);
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

%%
contactSphereRadius = 0.04;

% Create a contact sphere on the beam. We attach it to the top edge of the beam
beamBackContactSphere = ContactSphere();
beamBackContactSphere.setRadius(contactSphereRadius);
beamBackContactSphere.setLocation( Vec3(-beamLength/2, beamHeight/2, 0) );
beamBackContactSphere.setFrame(beam);
beamBackContactSphere.setName('BeamBackContact');
osimModel.addContactGeometry(beamBackContactSphere);

beamFrontContactSphere = ContactSphere();
beamFrontContactSphere.setRadius(contactSphereRadius);
beamFrontContactSphere.setLocation( Vec3(beamLength/2, beamHeight/2, 0) );
beamFrontContactSphere.setFrame(beam);
beamFrontContactSphere.setName('BeamFrontContact');
osimModel.addContactGeometry(beamFrontContactSphere);

% Create a contact mesh for the base body
% baseMeshFile = 'base_mesh.STL';
% baseContactMesh = ContactMesh();
% baseContactMesh.setFilename(baseMeshFile);
% baseContactMesh.setFrame(base);
% baseContactMesh.setName('BaseContactMesh');
% baseContactMesh.setLocation(Vec3(0, 0, 0));
% osimModel.addContactGeometry(baseContactMesh);
% 
% % Create a contact mesh for the beam body
% beamMeshFile = 'beam_mesh.STL';
% beamContactMesh = ContactMesh();
% beamContactMesh.setFilename(beamMeshFile);
% beamContactMesh.setFrame(beam);
% beamContactMesh.setName('BeamContactMesh');
% beamContactMesh.setLocation(Vec3(0, 100, 0));
% 
% % Add the contact mesh to the model
% osimModel.addContactGeometry(beamContactMesh);
% 
% Make a Contact Half Space
groundContactLocation = Vec3(0,0.025,0);
groundContactOrientation = Vec3(0,0,-1.57);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       ground);
groundContactSpace.setName('GroundContact');
osimModel.addContactGeometry(groundContactSpace);

%% --- Add a Contact Force (HuntCrossleyForce) ---
% Define parameters for the HuntCrossleyForce.
stiffness         = 1e5;  % Contact stiffness
dissipation       = 2;    % Damping coefficient
staticFriction    = 0.8;
dynamicFriction   = 0.6;
viscousFriction   = 0.2;
transitionVelocity= 0.2;

HuntCrossleyBackBeam = HuntCrossleyForce();
HuntCrossleyBackBeam.setName('BeamBackContactForce');
HuntCrossleyBackBeam.addGeometry('BeamBackContact');
HuntCrossleyBackBeam.addGeometry('GroundContact');
HuntCrossleyBackBeam.setStiffness(stiffness);
HuntCrossleyBackBeam.setDissipation(dissipation);
HuntCrossleyBackBeam.setStaticFriction(staticFriction);
HuntCrossleyBackBeam.setDynamicFriction(dynamicFriction);
HuntCrossleyBackBeam.setViscousFriction(viscousFriction);
HuntCrossleyBackBeam.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyBackBeam);

HuntCrossleyFrontBeam = HuntCrossleyForce();
HuntCrossleyFrontBeam.setName('BeamFrontContactForce');
HuntCrossleyFrontBeam.addGeometry('BeamFrontContact');
HuntCrossleyFrontBeam.addGeometry('GroundContact');
HuntCrossleyFrontBeam.setStiffness(stiffness);
HuntCrossleyFrontBeam.setDissipation(dissipation);
HuntCrossleyFrontBeam.setStaticFriction(staticFriction);
HuntCrossleyFrontBeam.setDynamicFriction(dynamicFriction);
HuntCrossleyFrontBeam.setViscousFriction(viscousFriction);
HuntCrossleyFrontBeam.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyFrontBeam);

% Create the HuntCrossleyForce to compute forces between the beam and the ground.
% HuntCrossleyBeam = HuntCrossleyForce();
% HuntCrossleyBeam.setName('BeamContactForce');
% HuntCrossleyBeam.addGeometry('BeamContactMesh');
% HuntCrossleyBeam.addGeometry('GroundContact');
% HuntCrossleyBeam.setStiffness(stiffness);
% HuntCrossleyBeam.setDissipation(dissipation);
% HuntCrossleyBeam.setStaticFriction(staticFriction);
% HuntCrossleyBeam.setDynamicFriction(dynamicFriction);
% HuntCrossleyBeam.setViscousFriction(viscousFriction);
% HuntCrossleyBeam.setTransitionVelocity(transitionVelocity);
% osimModel.addForce(HuntCrossleyBeam);
% 
% % Create the HuntCrossleyForce to compute forces between the beam and the base.
% HuntCrossleyBase = HuntCrossleyForce();
% HuntCrossleyBase.setName('BeamContactForce');
% HuntCrossleyBase.addGeometry('BeamContactMesh');
% HuntCrossleyBase.addGeometry('BaseContactMesh');
% HuntCrossleyBase.setStiffness(stiffness);
% HuntCrossleyBase.setDissipation(dissipation);
% HuntCrossleyBase.setStaticFriction(staticFriction);
% HuntCrossleyBase.setDynamicFriction(dynamicFriction);
% HuntCrossleyBase.setViscousFriction(viscousFriction);
% HuntCrossleyBase.setTransitionVelocity(transitionVelocity);
% osimModel.addForce(HuntCrossleyBase);


%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
osimModel.print('InvertedPendulumModel.osim');
display(['InvertedPendulumModel.osim printed!']);
