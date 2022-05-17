clear
clc
close all
fclose('all');

run(fullfile(fileparts(mfilename('fullpath')),'..','src','setup_sim.m'))

%% Usr Parameters

config.simulation_with_noise = 0;
config.only_controller       = 1;

%%

model = mystica.model.getModelCoverSquareLinks('n',3,'m',3','restConfiguration','flat','linkDimension',0.0482);
stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model,'stgs_integrator_limitMaximumTime',4);
stgs.desiredShape.fun = @(x,y,t) -5*x.^2 -5*y.^2;
stgs.integrator.dxdtOpts.assumeConstant = true;
stgs.saving.workspace.run = 0;
[data,stateKin]  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');

if stgs.visualizer.run
    mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
end

%% motor positioning

[model,sensitivity,genAlgrthm] = selectMotorPositioning('model',model,'state',stateKin,'stgs',stgs);

%%

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

data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',data.mBodyPosQuat_0(:,end),'nameControllerClass','ControllerKinRel');

if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs);
end
