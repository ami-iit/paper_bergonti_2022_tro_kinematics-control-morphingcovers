classdef SensitivityAnalysis < handle
    %SENSITIVITYANALYSIS Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=protected)
        stateKinMBody_iniConf
        stateKinMBody_endConf
        model
        population
        indexBestCandidate
    end
    properties (SetAccess=protected,GetAccess=public)
        stgs
        stats
        result
    end

    methods
        function obj = SensitivityAnalysis(input)
            arguments
                input.stateKinMBody
                input.model
                input.population
                input.desAngleVariation = 5*pi/180
                input.timeIncrement     = 0.01
                input.kBaum             = 0
                input.regTermDampPInv   = 0
            end

            obj.model                 = copy(input.model);
            obj.population            = input.population;

            obj.stateKinMBody_iniConf = copy(input.stateKinMBody);
            obj.stateKinMBody_endConf = copy(input.stateKinMBody);

            obj.stgs.desAngleVariation = input.desAngleVariation;
            obj.stgs.timeIncrement     = input.timeIncrement;
            obj.stgs.desMotorsAngVel   = obj.stgs.desAngleVariation/obj.stgs.timeIncrement;

            obj.stgs.numberCombinations = size(obj.population,1);
            obj.stgs.numberMotorConf    = size(obj.population,2);

            obj.stgs.kBaum             = input.kBaum;
            obj.stgs.regTermDampPInv   = input.regTermDampPInv;
            
            obj.stgs.unitMeas          = obj.model.unitMeas;

            obj.stats.dDeterminant_dAngle   = zeros(obj.stgs.numberCombinations,2*obj.stgs.numberMotorConf);
            obj.stats.divergenceDeterminant = sum(abs(obj.stats.dDeterminant_dAngle),2);
            obj.stats.determinant_iniConf   = zeros(obj.stgs.numberCombinations,1);
        end

        function clearProperties(obj)
            obj.stateKinMBody_iniConf = [];
            obj.stateKinMBody_endConf = [];
            obj.model                 = [];
            obj.population            = [];
            obj.indexBestCandidate    = [];
        end

    end

    methods (Static,Access=protected)
        function detA = getDeterminant(A)
            if size(A,1) == size(A,2)
                detA = det(A);
            else
                detA = 0;
            end
        end
    end

end
