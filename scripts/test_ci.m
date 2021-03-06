%% Test CI - Mesh 3x3
%
% file:    test_ci
% authors: Fabio Bergonti
% license: BSD 3-Clause

%% Clear Workspace + MATLABPATH Configuration

clear
clc
close all
fclose('all');

src_full_path      = fullfile(fileparts(mfilename('fullpath')),'..','src');
datasets_full_path = fullfile(fileparts(mfilename('fullpath')),'..','datasets');
run(fullfile(src_full_path,'setup_sim.m'))

%% Prepare Morphing Cover Model with Motors and its Initial Configuration
fprintf(' ========= script TEST_CI ========= \n')
fprintf('- script TEST_CI: start\n')

% 1) create model.
fprintf('- script TEST_CI: creating model\n')
model = mystica.model.getModelCoverSquareLinks('n',3,'m',3,'restConfiguration','flat','linkDimension',0.0482);
% 2) evaluate morphing cover initial configuration.
% initial configuration is computed running a controlled simulation starting from flat configuration. `mBodyTwist_0` is the control variable.
fprintf('- script TEST_CI: evaluating initial configuration\n')
stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
stgs.desiredShape.fun = @(x,y,t) -5*x.^2 -5*y.^2;
[data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');
% 3) solve the motors placement problem.
fprintf('- script TEST_CI: solving motors placement problem\n')
[model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);
mBodyPosQuat_0 = data.mBodyPosQuat_0(:,end);

% Verify result motors placement problem
assert(abs(det(stateKin.getZact('model',model)))>1e-5,'selected motor placement doesn''t guarantee full actuation locally')

%% Simulation

fprintf('- script TEST_CI: simulating the kinematics\n')

% stgs: get default values
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',6);
% stgs: controller parameters
stgs.controller.costFunction.weightTaskOrientation            = 1;
stgs.controller.costFunction.weightTaskMinVariation           = 0;
stgs.controller.costFunction.weightTaskMinOptiVar             = 0;
stgs.controller.costFunction.gainLinkAngVelStarAligned        = 30;
stgs.controller.costFunction.gainLinkAngVelStarOpposite       = 100;
stgs.controller.costFunction.useFeedForwardTermLinkAngVelStar = 1;
stgs.controller.constraints.limitPassiveAngVel = 5*pi/180;  % [rad/s] it can be set up to model limit (i.e. 20*180/pi).
stgs.controller.constraints.limitMotorVel      = 5*pi/180;  % [rad/s] it can be set up to model limit (i.e. 20*180/pi).
stgs.controller.constraints.limitRoM           = 50*pi/180; % [rad]   it can be set up to model limit (i.e. 50*180/pi).
% stgs: desired Shape
stgs.desiredShape.fun = @(x,y,t) 5.*x.*y.*cos(y/2);

% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');

% Verify results kinematic simulation
assert(max(abs(data.errorPositionNormals(:,end)))<1e-2,'errorPositionNormals is bigger than expected')
assert(max(abs(data.errorOrientationNormals(:,end)))<10,'errorOrientationNormals is bigger than expected')

fprintf(' ========= TEST_CI ended ========= \n')
