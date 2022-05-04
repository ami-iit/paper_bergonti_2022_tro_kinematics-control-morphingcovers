function selectParents(obj)
    rotatingWheelResults = rand(1,obj.stgs.populationSize.children);

    matrixDiff = rotatingWheelResults - obj.population.rouletteWheelCumProbability;
    matrixDiff(matrixDiff>0) = -2;
    [~,obj.children.parentsIndexes] = max(matrixDiff);

end
