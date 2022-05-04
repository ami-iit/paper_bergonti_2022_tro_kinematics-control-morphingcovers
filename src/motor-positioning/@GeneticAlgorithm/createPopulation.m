function createPopulation(obj)

    for i = 1 : obj.stgs.populationSize.parents
        obj.population.combinationDec(i,:) = sort(randi(obj.combinationDecRange,1,obj.combinationDecLength));
        obj.population.combinationBin(i,:) = obj.getBinaryCombination(obj.population.combinationDec(i,:));
        obj.population.fitValues(i)        = obj.fitnessFunction(obj.population.combinationDec(i,:));
    end

    if isempty(obj.initialGuessDec) == 0

        obj.initialGuessDec(any( obj.initialGuessDec > obj.combinationDecRange(2) ,2 ),:) = [];
        obj.initialGuessDec(any( obj.initialGuessDec < obj.combinationDecRange(1) ,2 ),:) = [];

        nGuess = size(obj.initialGuessDec,1);
        if nGuess > 0
            initialGuess.combinationDec              = obj.initialGuessDec;
            initialGuess.combinationBin              = zeros(nGuess,obj.combinationBinLength);
            initialGuess.fitValues                   = zeros(nGuess,1);

            for i = 1 : nGuess
                initialGuess.combinationBin(i,:) = obj.getBinaryCombination(initialGuess.combinationDec(i,:));
                initialGuess.fitValues(i)        = obj.fitnessFunction(initialGuess.combinationDec(i,:));
            end

             obj.mergePopulations(obj.population,initialGuess)
        end
    end

    obj.updateRouletteWheel();

end
