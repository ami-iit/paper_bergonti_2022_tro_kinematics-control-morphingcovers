function computeCostFunctionAndConstraints(obj,input)
    arguments
        obj
        input.model
        input.stateKinMBody
    end
    
    obj.csdSy.motorsAngVel         = obj.opti.variable( input.model.constants.motorsAngVel,1);
    obj.csdSy.motorsAngVel_lastSol = obj.opti.parameter(input.model.constants.motorsAngVel,1);
    
    obj.csdSy.nullJc_mBodyTwist_0  = obj.opti.parameter(input.model.constants.mBodyTwist,input.model.constants.motorsAngVel);
    obj.csdSy.invZact              = obj.opti.parameter(input.model.constants.motorsAngVel,input.model.constants.motorsAngVel);
    
    rC_from_mBodyTwist0_2_jointsAngVelPJ = input.stateKinMBody.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ(obj.csdSy.mBodyPosQuat_0);
    
    nullJc_jointsAngVel_PJ = rC_from_mBodyTwist0_2_jointsAngVelPJ * obj.csdSy.nullJc_mBodyTwist_0;
    
    mBodyTwist_0     = obj.csdSy.nullJc_mBodyTwist_0 * obj.csdSy.invZact * obj.csdSy.motorsAngVel;
    jointsAngVel_pj  = nullJc_jointsAngVel_PJ        * obj.csdSy.invZact * obj.csdSy.motorsAngVel;
    mBodyAngVel_0    = input.model.selector.matrix_mBodyAngVel_from_mBodyTwist * mBodyTwist_0;
    passiveAngVel_pj = jointsAngVel_pj(input.model.selector.indexes_passiveAngVel_from_jointsAngVel);
    
    %% Cost Function
    
    E1 = mBodyAngVel_0 - obj.csdFn.mBodyAngVelStar(obj.csdSy.mBodyPosQuat_0,obj.csdSy.time);
    
    E2 = obj.csdSy.motorsAngVel - obj.csdSy.motorsAngVel_lastSol;
    
    E3 = transpose(obj.csdSy.motorsAngVel)*obj.csdSy.motorsAngVel;
    
    obj.opti.minimize(...
        transpose(E1)*E1 * obj.stgsController.costFunction.weightTaskOrientation + ...
        transpose(E2)*E2 * obj.stgsController.costFunction.weightTaskMinVariation + ...
        transpose(E3)*E3 * obj.stgsController.costFunction.weightTaskMinOptiVar);
    
    %%  Limit Angular velocity
    
    if obj.stgsController.constraints.byPassModelLimits
        kModelLimit = inf;
    else
        kModelLimit = 1;
    end
    
    for j = 1 : input.model.nJoint
        vectorJointsAngVelLimit(input.model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel,1) = min(...
            input.model.joints{j}.limitJointVel*kModelLimit , ...
            obj.stgsController.constraints.limitMotorVel * input.model.joints{j}.axesActuated +  ...
            obj.stgsController.constraints.limitPassiveAngVel * (input.model.joints{j}.axesRotation & ~input.model.joints{j}.axesActuated));
    end
    
    vectorMotorsAngVelLimit  = vectorJointsAngVelLimit(input.model.selector.indexes_motorsAngVel_from_jointsAngVel);
    obj.opti.subject_to(-vectorMotorsAngVelLimit  <= obj.csdSy.motorsAngVel <= vectorMotorsAngVelLimit)
    
    vectorPassiveAngVelLimit = vectorJointsAngVelLimit(input.model.selector.indexes_passiveAngVel_from_jointsAngVel);
    if size(passiveAngVel_pj,1)>0
        obj.opti.subject_to(-vectorPassiveAngVelLimit <= passiveAngVel_pj <= vectorPassiveAngVelLimit)
    else
        warning('no passive joint')
    end
    
    %%  Limit Range of Motion
    
    contraintRoM.bLow = zeros(input.model.nJoint,1);
    contraintRoM.bUpp = zeros(input.model.nJoint,1);
    contraintRoM.versorsProduct = {};
    
    e = [1;0;0];
    
    for j = 1 : input.model.nJoint
        
        limitRoM = min(input.model.joints{j}.limitRoM*kModelLimit , obj.stgsController.constraints.limitRoM);
        
        index = input.model.joints{j}.getJointConnectionDetails();
        
        tform_0_p   = input.stateKinMBody.linksState{index.parent}.csdFn.tform_0_b(obj.csdSy.mBodyPosQuat_0);
        tform_p_pj  = input.model.linksAttributes{index.parent}.tform_b_j{index.cPointParent};
        tform_0_pj  = tform_0_p * tform_p_pj;
        tform_0_c   = input.stateKinMBody.linksState{index.child }.csdFn.tform_0_b(obj.csdSy.mBodyPosQuat_0);
        tform_c_cj  = input.model.linksAttributes{index.child }.tform_b_j{index.cPointChild };
        tform_0_cj  = tform_0_c * tform_c_cj;
        tform_pj_cj = transpose(tform_0_pj) * tform_0_cj;
        rotm_pj_cj  = mystica.rbm.getRotmGivenTform(tform_pj_cj);
        
        indexes_jAngVel = input.model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel;
        
        contraintRoM.versorsProduct{end+1} = transpose(e)*rotm_pj_cj*e - transpose(e)*mystica.utils.skew(rotm_pj_cj*e*obj.dt)*jointsAngVel_pj(indexes_jAngVel);
        
        contraintRoM.bLow(j) = -1;
        contraintRoM.bUpp(j) = -cos(limitRoM);
        
    end
    
    contraintRoM.versorsProduct = vertcat(contraintRoM.versorsProduct{:});
    
    
    obj.opti.subject_to(contraintRoM.versorsProduct <= contraintRoM.bUpp)
    
end
