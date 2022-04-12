function computeCostFunctionAndConstraints(obj,input)
    arguments
        obj
        input.model
        input.stateKinMBody
    end

    obj.csdSy.mBodyTwist_0         = obj.opti.variable(input.model.constants.mBodyTwist,1);
    obj.csdSy.mBodyTwist_0_lastSol = obj.opti.parameter(input.model.constants.mBodyTwist,1);

    Jc = input.stateKinMBody.csdFn.Jc(obj.csdSy.mBodyPosQuat_0);
    rC_from_mBodyTwist0_2_jointsAngVelPJ = input.stateKinMBody.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ(obj.csdSy.mBodyPosQuat_0);

    mBodyAngVel_0   = input.model.selector.matrix_mBodyAngVel_from_mBodyTwist * obj.csdSy.mBodyTwist_0;
    jointsAngVel_pj = rC_from_mBodyTwist0_2_jointsAngVelPJ * obj.csdSy.mBodyTwist_0;

    %% Cost Function

    E1 = mBodyAngVel_0 - obj.csdFn.mBodyAngVelStar(obj.csdSy.mBodyPosQuat_0,obj.csdSy.time);

    E2 = obj.csdSy.mBodyTwist_0 - obj.csdSy.mBodyTwist_0_lastSol;

    obj.opti.minimize(transpose(E1)*E1*obj.stgsController.costFunction.weightTaskOrientation + ...
                      transpose(E2)*E2*obj.stgsController.costFunction.weightTaskMinVariation);

    %% Jacobian of Constraints

    obj.opti.subject_to( Jc * obj.csdSy.mBodyTwist_0 == 0);

    %%  Limit Angular velocity

    for j = 1 : input.model.nJoint
        vectorJointsAngVelLimit(input.model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel,1) = input.model.joints{j}.limitJointVel;
    end

    obj.opti.subject_to(-vectorJointsAngVelLimit <= jointsAngVel_pj <= vectorJointsAngVelLimit)

    %%  Limit Range of Motion

    contraintRoM.bLow = zeros(input.model.nJoint,1);
    contraintRoM.bUpp = zeros(input.model.nJoint,1);
    contraintRoM.versorsProduct = {};

    e = [1;0;0];

    for j = 1 : input.model.nJoint

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
        contraintRoM.bUpp(j) = -cos(input.model.joints{j}.limitRoM);

    end

    contraintRoM.versorsProduct = vertcat(contraintRoM.versorsProduct{:});

    obj.opti.subject_to(contraintRoM.versorsProduct <= contraintRoM.bUpp)

end
