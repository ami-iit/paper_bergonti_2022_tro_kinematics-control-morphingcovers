function combinationDec = getDecimalCombination(obj,combinationBin)
    arguments
        obj
        combinationBin (:,1)
    end
    combinationDec = sort(transpose(obj.matrixFromCombinationBin2CombinationDec * combinationBin));
end
