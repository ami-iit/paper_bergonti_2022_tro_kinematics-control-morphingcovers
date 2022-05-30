function mergePopulations(obj,pop1,pop2)
%UPDATEPOPULATION Summary of this function goes here
%   Detailed explanation goes here

    popMerge.combinationDec = [pop1.combinationDec ; pop2.combinationDec];
    popMerge.combinationBin = [pop1.combinationBin ; pop2.combinationBin];
    popMerge.fitValues      = [pop1.fitValues      ; pop2.fitValues     ];

    if obj.stgs.populationWithDuplicates == 0
        boolDuplicates = ones(size(popMerge.combinationDec,1),1);
        [~,indexesUnique,~] = unique(popMerge.combinationDec,'rows','stable');
        boolDuplicates(indexesUnique)=0;
        boolDuplicates = (boolDuplicates==1);

        popMerge.fitValues(boolDuplicates) = 0;
    end

    [~,order] = sort(popMerge.fitValues,'descend');

    selIndexes = order(1:obj.stgs.populationSize.parents);

    populationCDbeforeMerge = obj.population.combinationDec;

    obj.population.combinationBin = popMerge.combinationBin(selIndexes,:);
    obj.population.combinationDec = popMerge.combinationDec(selIndexes,:);
    obj.population.fitValues      = popMerge.fitValues(selIndexes,:);

    populationCDafterMerge = obj.population.combinationDec;

    obj.updateProbabilityOfSelection();

    if sum(abs(populationCDbeforeMerge-populationCDafterMerge),'all') == 0
        obj.counterConstantPopulation = obj.counterConstantPopulation + 1;
    else
        obj.counterConstantPopulation = 0;
    end

end
