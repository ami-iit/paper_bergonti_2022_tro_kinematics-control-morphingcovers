function runAlgorithm(obj)
    arguments
        obj
    end

    nameTTF = [];

    for indexCombination = 1 : obj.stgs.numberCombinations

        booleanActuatedJointAngVel = zeros(obj.model.constants.jointsAngVel,1);
        booleanActuatedJointAngVel(obj.population(indexCombination,:)) = 1;
        booleanActuatedJointAngVel = reshape(booleanActuatedJointAngVel,obj.model.constants.linkEul,[]);
        for j = 1:obj.model.nJoint
            obj.model.actuateJoint('jointIndex',j,'axesActuated',booleanActuatedJointAngVel(:,j),'byPassWarning',1);
        end

        Zact_iniConf        = obj.stateKinMBody_iniConf.getZact('model',obj.model);
        obj.stats.determinant_iniConf(indexCombination,1) = obj.getDeterminant(Zact_iniConf);

        for indexDirection = 1 : 2*obj.stgs.numberMotorConf
            motorAngVel = zeros(obj.stgs.numberMotorConf,1);
            if indexDirection <= obj.stgs.numberMotorConf
                indexActuatedMotor = indexDirection;
                motorAngVel(indexActuatedMotor) = obj.stgs.desMotorsAngVel;
            else
                indexActuatedMotor=indexDirection-obj.stgs.numberMotorConf;
                motorAngVel(indexActuatedMotor) = -obj.stgs.desMotorsAngVel;
            end

            mBodyVelQuat_0 = obj.stateKinMBody_iniConf.get_mBodyVelQuat0_from_motorsAngVel(...
                'motorsAngVel',motorAngVel,'model',obj.model,'kBaum',obj.stgs.kBaum,'regTermDampPInv',obj.stgs.regTermDampPInv);

            obj.stateKinMBody_endConf.setMBodyPosQuat('model',obj.model,...
                'mBodyPosQuat_0',obj.stateKinMBody_iniConf.mBodyPosQuat_0 + mBodyVelQuat_0 * obj.stgs.timeIncrement)

            Zact_endConf = obj.stateKinMBody_endConf.getZact('model',obj.model);
            determinant_endConf = obj.getDeterminant(Zact_endConf);

            if obj.stats.determinant_iniConf(indexCombination,1) == 0 && determinant_endConf == 0
                obj.stats.dDeterminant_dAngle(indexCombination,indexDirection) = 1e5;
            else
                obj.stats.dDeterminant_dAngle(indexCombination,indexDirection) = (determinant_endConf - obj.stats.determinant_iniConf(indexCombination,1))/obj.stgs.desAngleVariation;
            end

        end
        if  mod(indexCombination,round(obj.stgs.numberCombinations/100)) == 0
            fprintf('SA | Percentage of completion: %.0f%%\n',(indexCombination/obj.stgs.numberCombinations)*100);
            nameTTF = mystica.utils.createTimeTrackerFile(nameTTF,'SA',(indexCombination/obj.stgs.numberCombinations)*100,100);
        end
    end

    obj.stats.divergenceDeterminant = sum(abs(obj.stats.dDeterminant_dAngle),2);

    [~,obj.indexBestCandidate]=min(obj.stats.divergenceDeterminant);
    obj.result = obj.population(obj.indexBestCandidate,:);

end
