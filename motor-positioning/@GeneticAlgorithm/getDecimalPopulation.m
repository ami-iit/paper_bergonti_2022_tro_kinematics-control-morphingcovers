function populationDec = getDecimalPopulation(obj,populationBin)
    arguments
        obj
        populationBin
    end
    populationDec = sort(transpose(obj.matrixFromCombinationBin2CombinationDec * transpose(populationBin)),2);
end
