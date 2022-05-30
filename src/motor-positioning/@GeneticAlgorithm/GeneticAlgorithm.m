classdef GeneticAlgorithm < handle
    %GENETICALGORITHM Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=immutable,GetAccess=public)
        stgs
        combinationDecLength
        combinationDecRange
        fitnessFunction
    end
    properties (SetAccess=protected,GetAccess=public)
        population
        stats
        indexGen = 1
        endCondition
    end
    properties (SetAccess=protected,GetAccess=protected)
        children
        combinationBinGeneLength
        combinationBinLength
        initialGuessDec
        counterConstantPopulation = 0
        time
    end
    properties (SetAccess=private,GetAccess=private)
        matrixFromBinaryNumber2DecimalNumber
        matrixFromCombinationBin2CombinationDec
    end

    methods
        function obj = GeneticAlgorithm(input)
            arguments
                input.probabilityCrossover               = 0.6
                input.probabilityMutation                = 0.01
                input.populationWithDuplicates   logical = 0
                input.limitGenerationNumber              = 1e2
                input.limitCounterConstantPopulation     = 5e1
                input.thresholdFitValues
                input.populationSize                     = 10
                input.populationRatioChildrenOverParents = 1
                input.combinationDecLength
                input.combinationDecRange (2,1) double
                input.fitnessFunction
                input.initialGuessDec                    = []
            end

            obj.time = tic;

            obj.stgs.probabilityCrossover     = input.probabilityCrossover;
            obj.stgs.probabilityMutation      = input.probabilityMutation;
            obj.stgs.populationWithDuplicates = input.populationWithDuplicates;
            obj.stgs.populationSize.parents   = input.populationSize;
            obj.stgs.populationSize.children  = floor(input.populationSize*input.populationRatioChildrenOverParents/2)*2;

            obj.endCondition.limitGenerationNumber          = input.limitGenerationNumber;
            obj.endCondition.limitCounterConstantPopulation = input.limitCounterConstantPopulation;
            obj.endCondition.thresholdFitValues             = input.thresholdFitValues;

            obj.combinationDecLength     = input.combinationDecLength;
            obj.combinationDecRange      = sort(input.combinationDecRange);
            obj.fitnessFunction          = input.fitnessFunction;
            obj.initialGuessDec          = input.initialGuessDec;

            obj.combinationBinGeneLength = ceil(log2(obj.combinationDecRange(2)));
            obj.combinationBinLength     = obj.combinationBinGeneLength * obj.combinationDecLength;

            obj.matrixFromBinaryNumber2DecimalNumber = 2.^((obj.combinationBinGeneLength - 1) : -1 : 0);

            obj.matrixFromCombinationBin2CombinationDec = zeros(obj.combinationDecLength,obj.combinationBinLength);
            for i = 1 : obj.combinationDecLength
                obj.matrixFromCombinationBin2CombinationDec(i,(1:obj.combinationBinGeneLength)+(i-1)*obj.combinationBinGeneLength) = obj.matrixFromBinaryNumber2DecimalNumber;
            end

            obj.population.combinationBin         = zeros(obj.stgs.populationSize.parents,obj.combinationBinLength);
            obj.population.combinationDec         = zeros(obj.stgs.populationSize.parents,obj.combinationDecLength);
            obj.population.fitValues              = zeros(obj.stgs.populationSize.parents,1);
            obj.population.probabilityOfSelection = zeros(obj.stgs.populationSize.parents,1);

            obj.children.combinationBin           = zeros(obj.stgs.populationSize.children,obj.combinationBinLength);
            obj.children.combinationDec           = zeros(obj.stgs.populationSize.children,obj.combinationDecLength);
            obj.children.parentsIndexes           = zeros(obj.stgs.populationSize.children,1);
            obj.children.fitValues                = zeros(obj.stgs.populationSize.children,1);

            obj.createPopulation();

        end

        runAlgorithm(obj,input)
        plotStatsFitValues(obj)
    end

    methods (Access=protected)

        numDec = getDecimalNumber(obj,numBin);
        combinationDec = getDecimalCombination(obj,combinationBin);
        combinationBin = getBinaryCombination(obj,combinationDec);
        populationDec = getDecimalPopulation(obj,populationBin);

        mergePopulations(obj,pop1,pop2)
        createPopulation(obj)
        updateProbabilityOfSelection(obj)
        selectParents(obj)
        performSinglePointCrossover(obj)
        performMutation(obj)
        updateFitValues(obj)
        getStats(obj,phase)

    end
end
