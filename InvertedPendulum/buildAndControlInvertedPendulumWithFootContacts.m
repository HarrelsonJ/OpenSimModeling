function model=buildAndControlInvertedPendulumWithFootContacts()

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
model = Model();
model.setName('InvertedPendulumModel');

% Get a reference to the ground object
ground = model.getGround();

% Define the acceleration of gravity
model.setGravity(Vec3(0, -9.80665, 0));

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
model.addBody(base);

% Make and add a Weld joint for the base Body
%{
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-footHeight/2,0);
orientationInChild  = Vec3(0,0,0);
baseToGround = WeldJoint('BaseToGround', ground, locationInParent, ...
    orientationInParent, base, locationInChild, orientationInChild);
osimModel.addJoint(baseToGround);
%}

locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,-footHeight/2,0);
orientationInChild  = Vec3(0,0,0);
baseToGround = FreeJoint('BaseToGround', ground, locationInParent, ...
    orientationInParent, base, locationInChild, orientationInChild);
model.addJoint(baseToGround)

% Define beam
beam = Body('Beam', ... % Name
            5, ... % Mass (kg)
            Vec3(0), ... % Center of mass location in body frame)
            Inertia(1)... % Inertia
        );

% Add geometry to the body
beamGeometry = Brick( Vec3(beamLength/2,beamHeight/2, beamWidth/2) );
beamGeometry.setColor( Vec3(0.8, 0.1, 0.1) );
beam.attachGeometry(beamGeometry);

model.addBody(beam);

% Make and add a pinjoint to connect the beam to the base
locationInParent    = Vec3(-(footLength/2 - beamLength/2),footHeight/2,0);
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

% Get coordinate for ankle exo actuator
torqueCoord = beamToBase.upd_coordinates(0);
torqueCoord.setName("ankle_angle");

model.addJoint(beamToBase);

%% Setup expression bassed force
ankleExo = ExpressionBasedCoordinateForce();
ankleExo.set_coordinate(torqueCoord.getName());
ankleExo.set_expression("-500*q-50*qdot");
ankleExo.setName('AnkleExo');
model.addForce(ankleExo);

%% Build ground contacts
r_contactsphere = 0.035000000000000003;

ground = model.getGround();
groundContactLocation = Vec3(0,0.025,0);
groundContactOrientation = Vec3(0,0,-1.57);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       ground);
groundContactSpace.setName('GroundContact');
model.addContactGeometry(groundContactSpace);

heelSphereLocation = Vec3(-(footLength / 2) * 4/5, 0, 0);
frontSphereLocation = Vec3((footLength / 2) * 4/5, 0, 0);

heelContactSphere = ContactSphere();
heelContactSphere.setRadius(r_contactsphere);
heelContactSphere.setLocation(heelSphereLocation);
heelContactSphere.setFrame(base)
heelContactSphere.setName('heel');
model.addContactGeometry(heelContactSphere);

frontContactSphere = ContactSphere();
frontContactSphere.setRadius(r_contactsphere);
frontContactSphere.setLocation(frontSphereLocation);
frontContactSphere.setFrame(base)
frontContactSphere.setName('front');
model.addContactGeometry(frontContactSphere);

%% Define Contact Force Parameters - Curently all defaults
stiffness           = 3067776;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;
ConstantContactForce = 1e-5; % ConstantContactForce
HertzSmoothing = 300; % HertzSmoothing
HuntCrossleySmoothing = 50; % HuntCrossleySmoothing

contactHeel = SmoothSphereHalfSpaceForce();
contactHeel.setName('contactHeel');
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
contactFront.setName('contactHeel');
contactFront.connectSocket_sphere(frontContactSphere);
contactFront.connectSocket_half_space(groundContactSpace);

contactFront.set_stiffness(stiffness);
contactFront.set_dissipation(dissipation);
contactFront.set_static_friction(staticFriction);
contactFront.set_dynamic_friction(dynamicFriction);
contactFront.set_viscous_friction(viscousFriction);
contactFront.set_transition_velocity(transitionVelocity);
model.upd_ComponentSet().addComponent(contactFront);

%% Initialize the System (checks model consistency).
model.finalizeConnections();
model.initSystem();

% Save the model to a file
model.print('InvertedPendulumModel_Control_FootContacts.osim');
display(['InvertedPendulumModel_Control_FootContacts.osim printed!']);

end
