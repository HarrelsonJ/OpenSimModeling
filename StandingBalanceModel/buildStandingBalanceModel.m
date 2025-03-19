%% Clean up workspace
close all; clear; clc;

%%
import org.opensim.modeling.*;

% Load Rajagopal2015 model
model = Model('Rajagopal2015.osim');

ground=model.getGround();

% Make a Contact Half Space for floor contact
groundContactLocation = Vec3(0,0,0);
groundContactOrientation = Vec3(0,0,-1.57);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       ground);
groundContactSpace.setName('GroundContact');
model.addContactGeometry(groundContactSpace);

calcn_r = model.getBodySet().get('calcn_r');
calcn_l = model.getBodySet().get('calcn_l');

r_contactsphere = 0.035000000000000003;
heelSphereLocation = Vec3(0.031307527581931796,0.010435842527310599,0);
frontSphereLocation = Vec3(0.17740932296428019, -0.015653763790965898, 0.0052179212636552993);

rightHeelContactSphere = ContactSphere();
rightHeelContactSphere.setRadius(r_contactsphere);
rightHeelContactSphere.setLocation(heelSphereLocation);
rightHeelContactSphere.setFrame(calcn_r)
rightHeelContactSphere.setName('heel_r');
model.addContactGeometry(rightHeelContactSphere);

leftHeelContactSphere = ContactSphere();
leftHeelContactSphere.setRadius(r_contactsphere);
leftHeelContactSphere.setLocation(heelSphereLocation);
leftHeelContactSphere.setFrame(calcn_l)
leftHeelContactSphere.setName('heel_l');
model.addContactGeometry(leftHeelContactSphere);

rightFrontContactSphere = ContactSphere();
rightFrontContactSphere.setRadius(r_contactsphere);
rightFrontContactSphere.setLocation(frontSphereLocation);
rightFrontContactSphere.setFrame(calcn_r)
rightFrontContactSphere.setName('front_r');
model.addContactGeometry(rightFrontContactSphere);

leftFrontContactSphere = ContactSphere();
leftFrontContactSphere.setRadius(r_contactsphere);
leftFrontContactSphere.setLocation(frontSphereLocation);
leftFrontContactSphere.setFrame(calcn_l)
leftFrontContactSphere.setName('front_l');
model.addContactGeometry(leftFrontContactSphere);

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

contactHeel_r = SmoothSphereHalfSpaceForce();
contactHeel_r.setName('contactHeel_r');
contactHeel_r.connectSocket_sphere(rightHeelContactSphere);
contactHeel_r.connectSocket_half_space(groundContactSpace);

contactHeel_r.set_stiffness(stiffness);
contactHeel_r.set_dissipation(dissipation);
contactHeel_r.set_static_friction(staticFriction);
contactHeel_r.set_dynamic_friction(dynamicFriction);
contactHeel_r.set_viscous_friction(viscousFriction);
contactHeel_r.set_transition_velocity(transitionVelocity);
model.upd_ComponentSet().addComponent(contactHeel_r);

contactHeel_l = SmoothSphereHalfSpaceForce();
contactHeel_l.setName('contactHeel_l');
contactHeel_l.connectSocket_sphere(leftHeelContactSphere);
contactHeel_l.connectSocket_half_space(groundContactSpace);

contactHeel_l.set_stiffness(stiffness);
contactHeel_l.set_dissipation(dissipation);
contactHeel_l.set_static_friction(staticFriction);
contactHeel_l.set_dynamic_friction(dynamicFriction);
contactHeel_l.set_viscous_friction(viscousFriction);
contactHeel_l.set_transition_velocity(transitionVelocity);
model.upd_ComponentSet().addComponent(contactHeel_l);

contactFront_r = SmoothSphereHalfSpaceForce();
contactFront_r.setName('contactFront_r');
contactFront_r.connectSocket_sphere(rightFrontContactSphere);
contactFront_r.connectSocket_half_space(groundContactSpace);

contactFront_r.set_stiffness(stiffness);
contactFront_r.set_dissipation(dissipation);
contactFront_r.set_static_friction(staticFriction);
contactFront_r.set_dynamic_friction(dynamicFriction);
contactFront_r.set_viscous_friction(viscousFriction);
contactFront_r.set_transition_velocity(transitionVelocity);
model.upd_ComponentSet().addComponent(contactFront_r);

contactFront_l = SmoothSphereHalfSpaceForce();
contactFront_l.setName('contactHeel_l');
contactFront_l.connectSocket_sphere(leftFrontContactSphere);
contactFront_l.connectSocket_half_space(groundContactSpace);

contactFront_l.set_stiffness(stiffness);
contactFront_l.set_dissipation(dissipation);
contactFront_l.set_static_friction(staticFriction);
contactFront_l.set_dynamic_friction(dynamicFriction);
contactFront_l.set_viscous_friction(viscousFriction);
contactFront_l.set_transition_velocity(transitionVelocity);
model.upd_ComponentSet().addComponent(contactFront_l);


%% Finalize connections
model.finalizeConnections();
model.initSystem();

% Save the model to a file
model.print('Rajagopal2015_with_ground_foot_contact.osim');
display('Rajagopal2015_with_ground_foot_contact.osim printed!');