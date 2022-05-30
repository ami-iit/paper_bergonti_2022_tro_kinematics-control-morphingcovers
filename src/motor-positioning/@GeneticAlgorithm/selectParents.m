function selectParents(obj)
    vectorRandomNumbers = rand(1,obj.stgs.populationSize.children);

    matrixDiff = vectorRandomNumbers - cumsum(obj.population.probabilityOfSelection);
    matrixDiff(matrixDiff>0) = -2;
    [~,obj.children.parentsIndexes] = max(matrixDiff);

end
