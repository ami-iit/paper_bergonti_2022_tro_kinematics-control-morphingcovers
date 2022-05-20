%% Simulation 2 - Mesh 8x8
%
% file:    sim2
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

% config.simulation_with_noise
%   - true  => simulation with Gaussian noise in the estimated state used by the controller.
%   - false => ideal simulation
config.simulation_with_noise = false;

% config.run_only_controller
%   - true  => load model with motors and its initial configuration.
%              if this option is true,  the running time is ~300s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
%   - false => create model, evaluate the initial configuration, and solve the motors placement problem.
%              if this option is false, the running time is ~600s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
config.run_only_controller   = true;

%% Prepare Morphing Cover Model with Motors and its Initial Configuration

if config.run_only_controller
    % load model with motors and morphing cover initial configuration.
    load(fullfile(datasets_full_path,'initSim2.mat'),'model','mBodyPosQuat_0')
    stgs.saving.workspace.name = 'initSim2';
else
    % 1) create model.
    model = mystica.model.getModelCoverSquareLinks('n',8,'m',8,'restConfiguration','flat','linkDimension',0.0482);
    % 2) evaluate morphing cover initial configuration.
    % initial configuration is computed running a controlled simulation starting from flat configuration. `mBodyTwist_0` is the control variable.
    stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
    stgs.desiredShape.fun = @(x,y,t) -2*x.^2 -2*y.^2;
    [data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');
    if stgs.visualizer.run
        mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
    end
    % 3) solve the motors placement problem.
    [model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);
    mBodyPosQuat_0 = data.mBodyPosQuat_0(:,end);
end

%% Simulation

% stgs: get default values
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',40);
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
stgs.desiredShape.fun = @(x,y,t) cos(t/8 + 10*x - 20*y)/40 - cos(t/8)/40 + cos(t/8 - 10*x + 2)/40 - cos(t/8 + 2)/40;
% stgs: integrator/state/noise
stgs.noise.errorStateEstimation.bool = config.simulation_with_noise;
% stgs: visualizer
stgs.visualizer.origin.dimCSYS                           = 0.01;
stgs.visualizer.cameraView.mBodySimulation.values        = [-37.5,30];
stgs.visualizer.cameraView.initialRotation.run           = 1;
stgs.visualizer.cameraView.initialRotation.values        = [0,90];
stgs.visualizer.cameraView.finalRotation.run             = 1;
stgs.visualizer.cameraView.finalRotation.values          = [45,20];

% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');

% visualize simulation
if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end
