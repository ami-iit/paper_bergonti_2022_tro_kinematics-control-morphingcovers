clear
clc
close all
fclose('all');

addpath(fullfile(fileparts(mfilename('fullpath')),'..','..','controller'))
addpath(fullfile(fileparts(mfilename('fullpath')),'..','..','motor-positioning'))

%% Usr Parameters

config.simulation_with_noise = 0;
config.only_controller       = 1;

%%

model = mystica.model.getModelCoverSquareLinks('n',3,'m',3','restConfiguration','flat','linkDimension',0.0482);
stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model);
data  = mystica.runSimKinAbs('model',model,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'stgs',stgs,'nameControllerClass','ControllerKinAbs');

if stgs.visualizer.run
    mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
end

