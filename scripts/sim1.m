%% Simulation 1 - Mesh 3x3
%
% file:    sim1
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
%   - true  => simulation with Gaussian noise to motor velocity
%   - false => ideal simulation
config.simulation_with_noise = false;

% config.run_only_controller
%   - true  => load model with motors and its initial configuration.
%              if this option is true,  the running time is ~10s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
%   - false => create model, evaluate the initial configuration, and solve the motors placement problem.
%              if this option is false, the running time is ~30s with a PC with Intel Xeon Gold 6128 3.40GHz and RAM 128GB.
config.run_only_controller   = false;

%% Prepare Morphing Cover Model with Motors and its Initial Configuration
fprintf(' ========= script SIM1 ========= \n')
fprintf('- script SIM1: start\n')

if config.run_only_controller
    % load model with motors and morphing cover initial configuration.
    fprintf('- script SIM1: loading model\n')
    load(fullfile(datasets_full_path,'initSim1.mat'),'model','mBodyPosQuat_0')
    stgs.saving.workspace.name = 'initSim1';
else
    % 1) create model.
    fprintf('- script SIM1: creating model\n')
    model = mystica.model.getModelCoverSquareLinks('n',3,'m',3,'restConfiguration','flat','linkDimension',0.0482);
    % 2) evaluate morphing cover initial configuration.
    % initial configuration is computed running a controlled simulation starting from flat configuration. `mBodyTwist_0` is the control variable.
    fprintf('- script SIM1: evaluating initial configuration\n')
    stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
    stgs.desiredShape.fun = @(x,y,t) -5*x.^2 -5*y.^2;
    [data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');
    if stgs.visualizer.run
        mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
    end
    % 3) solve the motors placement problem.
    fprintf('- script SIM1: solving motors placement problem\n')
    [model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);
    mBodyPosQuat_0 = data.mBodyPosQuat_0(:,end);
end

%% Simulation

fprintf('- script SIM1: simulating the kinematics\n')

% stgs: get default values
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',8);
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
% stgs: integrator/state/noise
stgs.noise.inputCompression.bool         = config.simulation_with_noise;
stgs.noise.inputCompression.maxValue     = 0.2;
stgs.noise.inputCompression.probMaxValue = 0.1;
% stgs: visualizer
stgs.visualizer.origin.dimCSYS                           = 0.01;
stgs.visualizer.mBody.bodyCSYS.show                      = 1;
stgs.visualizer.mBody.bodyCSYS.dim                       = 0.025;
stgs.visualizer.desiredShape.normal.show                 = 1;
stgs.visualizer.desiredShape.normal.color                = [0.5, 0.7, 0.9];
stgs.visualizer.desiredShape.normal.dim                  = 0.016;
stgs.visualizer.cameraView.mBodySimulation.values        = [-37.5,30];
stgs.visualizer.cameraView.initialRotation.run           = 1;
stgs.visualizer.cameraView.initialRotation.values        = [0,90];
stgs.visualizer.cameraView.finalRotation.run             = 1;
stgs.visualizer.cameraView.finalRotation.values          = [45,20];

% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');

% visualize simulation
if stgs.visualizer.run
    fprintf('- script SIM1: running visualizer\n')
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end

fprintf(' ========= SIM1 ended ========= \n')
