function updateProbabilityOfSelection(obj)

    sumFitValues = sum(obj.population.fitValues);

    obj.population.probabilityOfSelection = obj.population.fitValues/sumFitValues;

end
