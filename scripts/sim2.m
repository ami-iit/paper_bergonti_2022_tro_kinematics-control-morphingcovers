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

src_full_path = fullfile(fileparts(mfilename('fullpath')),'..','src');
run(fullfile(src_full_path,'setup_sim.m'))

%% User Parameters

% config.simulation_with_noise
%   - true  => simulation with Gaussian noise in the estimated state used by the controller.
%   - false => ideal simulation
config.simulation_with_noise = 0;

% config.run_only_controller
%   - true  => load model with motors and its initial configuration.
%   - false => create model, evaluate the initial configuration, and solve the motors placement problem.
config.run_only_controller   = 0;

%% Prepare Morphing Cover Model with Motors and its Initial Configuration

if config.run_only_controller
    % load model with motors and morphing cover initial configuration.
    load(fullfile(src_full_path,'datasets','initSim2.mat'),'model','mBodyPosQuat_0')
    stgs.saving.workspace.name = 'initSim2';
else
    % 1) create model.
    model = mystica.model.getModelCoverSquareLinks('n',8,'m',8,'restConfiguration','flat','linkDimension',0.0482);
    % 2) evaluate morphing cover initial configuration.
    % initial configuration is computed running a controlled simulation starting from flat configuration. `mBodyTwist_0` is the control variable.
    stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
    stgs.desiredShape.fun = @(x,y,t) -2*x.^2 -2*y.^2;
    stgs.integrator.dxdtOpts.assumeConstant = true;
    stgs.saving.workspace.run = 0;
    [data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');
    if stgs.visualizer.run
        mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
    end
    % 3) solve the motors placement problem.
    [model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);
    mBodyPosQuat_0 = data.mBodyPosQuat_0(:,end);
end

%% Simulation

% define stgs
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',40);
stgs.desiredShape.fun = @(x,y,t) cos(t/8 + 10*x - 20*y)/40 - cos(t/8)/40 + cos(t/8 - 10*x + 2)/40 - cos(t/8 + 2)/40;
stgs.integrator.dxdtOpts.assumeConstant = true;
if config.simulation_with_noise
    stgs.noise.errorStateEstimation.bool = 1;
end
stgs.saving.workspace.run                                = 0;
stgs.visualizer.origin.dimCSYS                           = 0.01;
stgs.visualizer.cameraView.mBodySimulation.values        = [-37.5,30];
stgs.visualizer.cameraView.initialRotation.run           = 1;
stgs.visualizer.cameraView.initialRotation.values        = [0,90];
stgs.visualizer.cameraView.initialRotation.durationTotal = 3;
stgs.visualizer.cameraView.initialRotation.pause.start   = 0;
stgs.visualizer.cameraView.initialRotation.pause.end     = 0;
stgs.visualizer.cameraView.finalRotation.run             = 1;
stgs.visualizer.cameraView.finalRotation.values          = [45,20];
stgs.visualizer.cameraView.finalRotation.durationTotal   = 3;
stgs.visualizer.cameraView.finalRotation.pause.start     = 0;
stgs.visualizer.cameraView.finalRotation.pause.end       = 0;
% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');
% visualize simulation
if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end
