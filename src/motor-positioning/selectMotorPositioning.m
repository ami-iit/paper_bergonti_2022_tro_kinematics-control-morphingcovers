function [model,sensitivity,genAlgrthm] = selectMotorPositioning(input)
    arguments
        input.model       mystica.model.Model
        input.state
        input.stgs
        input.GA_probabilityMutation                (1,1) double  = 0.01
        input.GA_probabilityCrossover               (1,1) double  = 0.6
        input.GA_populationSize                     (1,1) double  = 100
        input.GA_populationRatioChildrenOverParents (1,1) double  = 1
        input.GA_limitGenerationNumber              (1,1) double  = 1e5
        input.GA_limitCounterConstantPopulation     (1,1) double  = 100
        input.GA_thresholdFitValues                 (1,1) logical = true
        input.GA_populationWithDuplicates           (1,1) logical = false
        input.SA_desAngleVariation                  (1,1) double = 5*pi/180;
        input.SA_timeIncrement                      (1,1) double = 0.01;
    end

    model       = input.model;
    stateKin    = copy(input.state);
    stgs        = input.stgs;

    %% GA

    candidateApproachQR.combination = mystica.utils.findLinearIndipendentRows_algorithmQR(stateKin.nullJc_jointsAngVel_PJ,size(stateKin.nullJc_jointsAngVel_PJ,2));
    candidateApproachQR.determinant = abs(det(stateKin.nullJc_jointsAngVel_PJ(candidateApproachQR.combination,:)));

    % Fitness Function

    fitnessFunction = @(x) abs(det(stateKin.nullJc_jointsAngVel_PJ(x,:))) + eps + ...
        (abs(det(stateKin.nullJc_jointsAngVel_PJ(x,:))) > candidateApproachQR.determinant/10) * ...
        (sum(diff(ceil(x/model.constants.linkAngVel))==0)+1)^-1;

    % Settings

    genAlgrthm = GeneticAlgorithm(...
        'probabilityMutation',input.GA_probabilityMutation,...
        'probabilityCrossover',input.GA_probabilityCrossover,...
        'populationSize',input.GA_populationSize,...
        'populationRatioChildrenOverParents',input.GA_populationRatioChildrenOverParents,...
        'limitGenerationNumber',input.GA_limitGenerationNumber,...
        'limitCounterConstantPopulation',input.GA_limitCounterConstantPopulation,...
        'thresholdFitValues',input.GA_thresholdFitValues,...
        'fitnessFunction',fitnessFunction,...
        'populationWithDuplicates',input.GA_populationWithDuplicates,...
        'initialGuessDec',candidateApproachQR.combination,...
        'combinationDecLength',size(stateKin.nullJc_jointsAngVel_PJ,2),...
        'combinationDecRange',[1 size(stateKin.nullJc_jointsAngVel_PJ,1)]);

    genAlgrthm.runAlgorithm()
    genAlgrthm.plotStatsFitValues()

    %% Sensitivity Analysis

    sensitivity = SensitivityAnalysis(...
        'desAngleVariation',input.SA_desAngleVariation,...
        'timeIncrement',input.SA_timeIncrement,...
        'model',model,...
        'stateKinMBody',stateKin,...
        'population',genAlgrthm.population.combinationDec,...
        'kBaum',stgs.integrator.dxdtParam.baumgarteIntegralCoefficient,...
        'regTermDampPInv',0);

    sensitivity.runAlgorithm()
    sensitivity.plotStats()

    %% Update model

    booleanActuatedJointAngVel = zeros(model.constants.jointsAngVel,1);
    booleanActuatedJointAngVel(sensitivity.result) = 1;
    booleanActuatedJointAngVel = reshape(booleanActuatedJointAngVel,model.constants.linkEul,[]);
    for j = 1:model.nJoint
        model.actuateJoint('jointIndex',j,'axesActuated',booleanActuatedJointAngVel(:,j),'byPassWarning',1);
    end

    %% Saving

    if stgs.saving.workspace.clearCasadi
        stateKin.clearProperties();
    end
    sensitivity.clearProperties();
    if stgs.saving.workspace.run
        stgs.saving.workspace.name = [stgs.saving.workspace.name(1:end-4),'_motorPos.mat'];
        save(stgs.saving.workspace.name)
    end

end
