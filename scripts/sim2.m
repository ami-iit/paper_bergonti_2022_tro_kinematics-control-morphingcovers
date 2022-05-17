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

run(fullfile(fileparts(mfilename('fullpath')),'..','src','setup_sim.m'))

%% User Parameters

% config.simulation_with_noise
%   - true  => simulation with Gaussian noise in the estimated state used by the controller.
%   - false => ideal simulation
config.simulation_with_noise = 1;

% config.run_only_controller
%   - true  => load model with motors and its initial configuration.
%   - false => create model, evaluate the initial configuration, and solve the motors placement problem.
config.run_only_controller   = 0;

%% Prepare Morphing Cover Model with Motors and its Initial Configuration

if config.run_only_controller
    % load model with motors and morphing cover initial configuration.
    load('initSim2.mat','model','mBodyPosQuat_0')
    stgs.saving.workspace.name('initSim2')
else
    % 1) create model.
    model = mystica.model.getModelCoverSquareLinks('n',3,'m',3','restConfiguration','flat','linkDimension',0.0482);
    % 2) evaluate morphing cover initial configuration.
    % initial configuration is computed running a controlled simulation starting from flat configuration. `mBodyTwist_0` is the control variable.
    stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
    stgs.desiredShape.fun = @(x,y,t) -5*x.^2 -5*y.^2;
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
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model,'startFile',stgs.saving.workspace.name,'stgs_integrator_limitMaximumTime',8);
stgs.desiredShape.fun = @(x,y,t) 5.*x.*y.*cos(y/2);
stgs.integrator.dxdtOpts.assumeConstant = true;
if config.simulation_with_noise
    stgs.noise.inputCompression.bool = 1;
end
stgs.saving.workspace.run                                = 0;
stgs.visualizer.origin.dimCSYS                           = 0.01;
stgs.visualizer.mBody.bodyCSYS.show                      = 1;
stgs.visualizer.mBody.bodyCSYS.dim                       = 0.025;
stgs.visualizer.desiredShape.normal.show                 = 1;
stgs.visualizer.desiredShape.normal.color                = [0.5, 0.7, 0.9];
stgs.visualizer.desiredShape.normal.dim                  = 0.016;
stgs.visualizer.cameraView.mBodySimulation.values        = [-37.5,30];
stgs.visualizer.cameraView.initialRotation.run           = 1;
stgs.visualizer.cameraView.initialRotation.values        = [0,90];
stgs.visualizer.cameraView.initialRotation.durationTotal = 13;
stgs.visualizer.cameraView.initialRotation.pause.start   = 2;
stgs.visualizer.cameraView.initialRotation.pause.end     = 7;
stgs.visualizer.cameraView.finalRotation.run             = 1;
stgs.visualizer.cameraView.finalRotation.values          = [45,20];
stgs.visualizer.cameraView.finalRotation.durationTotal   = 6;
stgs.visualizer.cameraView.finalRotation.pause.start     = 1;
stgs.visualizer.cameraView.finalRotation.pause.end       = 2;
% run simulation
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',mBodyPosQuat_0,'nameControllerClass','ControllerKinRel');
% visualize simulation
if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end
