function updateRouletteWheel(obj)

    sumFitValues = sum(obj.population.fitValues);

    obj.population.rouletteWheel = obj.population.fitValues/sumFitValues;
    obj.population.rouletteWheelCumProbability = cumsum(obj.population.rouletteWheel);

end
