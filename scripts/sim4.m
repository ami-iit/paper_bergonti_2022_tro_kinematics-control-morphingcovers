%% Simulation 4 - Mesh 4x8
%
% file:    sim4
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

%% User Parameters

% config.run_only_controller
%   - true  => load model with motors and its initial configuration.
%              if this option is true,  the running time is  ~60s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
%   - false => create model, evaluate the initial configuration, and solve the motors placement problem.
%              if this option is false, the running time is ~160s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
config.run_only_controller   = false;

%% Prepare Morphing Cover Model with Motors and its Initial Configuration

if config.run_only_controller
    % load model with motors and morphing cover initial configuration.
    load(fullfile(datasets_full_path,'initSim4.mat'),'model','mBodyPosQuat_0')
    stgs.saving.workspace.name = 'initSim4';
else
    % 1) create model and state object.
    model = mystica.model.getModelCoverSquareLinks('n',4,'m',8,'restConfiguration','cylinder','linkDimension',0.0460,'baseIndex',5,'fixedLinksIndexes',[5,13,21,29],'tform_0_bBase',mystica.rbm.getTformGivenPosRotm([0 0 0.0555]',mystica.rbm.getRotmGivenEul('rz',pi/2)));
    % 2) evaluate morphing cover initial configuration.
    stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',0.02);
    stgs.desiredShape.fun = @(x,y,t) 2*x.^2 + 2*y.^2;
    [data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');
    % 3) solve the motors placement problem.
    [model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);
    mBodyPosQuat_0 = data.mBodyPosQuat_0(:,end);
end

%% Simulation

% stgs: get default values
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',35);
% stgs: controller parameters
stgs.controller.costFunction.weightTaskOrientation            = 1;
stgs.controller.costFunction.weightTaskMinVariation           = 500;
stgs.controller.costFunction.weightTaskMinOptiVar             = 0;
stgs.controller.costFunction.gainLinkAngVelStarAligned        = 30;
stgs.controller.costFunction.gainLinkAngVelStarOpposite       = 100;
stgs.controller.costFunction.useFeedForwardTermLinkAngVelStar = 1;
stgs.controller.constraints.limitPassiveAngVel = 5*pi/180;  % [rad/s] it can be set up to model limit (i.e. 20*180/pi).
stgs.controller.constraints.limitMotorVel      = 5*pi/180;  % [rad/s] it can be set up to model limit (i.e. 20*180/pi).
stgs.controller.constraints.limitRoM           = 50*pi/180; % [rad]   it can be set up
% stgs: desired Shape
f1 = @(x,y) 3*(x-0.07).^2-3*(y).^2; t1 = 10;
f2 = @(x,y) 7*(x-0.07).^2;          t2 = 15;
f3 = @(x,y) 3*(y.^2);               t3 = 25;
f4 = @(x,y) -3.5*(y.^2);
stgs.desiredShape.fun = @(x,y,t) (t<=t1)*f1(x,y) + (t>t1 & t<=t2)*f2(x,y) + (t>t2 & t<=t3)*f3(x,y) + (t>t3)*f4(x,y);
stgs.desiredShape.fun = @(x,y,t) stgs.desiredShape.fun(x,y,t)-stgs.desiredShape.fun(0,0,t)+0.055; clear f1 f2 f3 f4 t1 t2 t3;
% stgs: integrator/state/noise
stgs.stateKin.nullSpace.toleranceRankRevealing = [10 1e-8];
% stgs: visualizer
stgs.visualizer.origin.dimCSYS                           = 0.01;
stgs.visualizer.cameraView.mBodySimulation.values        = [230,40];
stgs.visualizer.cameraView.initialRotation.run           = 1;
stgs.visualizer.cameraView.initialRotation.values        = [0,90];
stgs.visualizer.cameraView.finalRotation.run             = 1;
stgs.visualizer.cameraView.finalRotation.values          = [90,0];
stgs.visualizer.background{1}.stlName = 'iRonCub_Leg.stl';
stgs.visualizer.background{1}.tform_0_originSTL = mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('rx',0));
stgs.visualizer.background{1}.scale     = [1 1 1]/1e3;
stgs.visualizer.background{1}.FaceColor = [0.7 0.7 0.7];
stgs.visualizer.background{1}.EdgeColor = 'none';
stgs.visualizer.background{1}.FaceAlpha = 0.3;

% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');

% visualize simulation
if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end
