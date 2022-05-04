function runAlgorithm(obj,input)
    arguments
        obj
        input.limitGenerationNumber          = obj.endCondition.limitGenerationNumber
        input.limitCounterConstantPopulation = obj.endCondition.limitCounterConstantPopulation
    end

    obj.endCondition.limitGenerationNumber = input.limitGenerationNumber;

    continueWhileLoop = 0;

    if obj.indexGen < obj.endCondition.limitGenerationNumber
        obj.getStats('config')
        continueWhileLoop = 1;
    end

    if obj.indexGen == 1
        obj.getStats('update')
    end

    nameTTF = [];

    while continueWhileLoop
        obj.indexGen = obj.indexGen + 1;

        obj.selectParents()
        obj.performSinglePointCrossover()
        obj.performMutation()
        obj.updateFitValues()
        obj.mergePopulations(obj.population,obj.children)

        obj.getStats('update')

        if  mod(obj.indexGen,round(obj.endCondition.limitGenerationNumber/100)) == 0
            fprintf('GA | Percentage of completion: %.0f%%\n',(obj.indexGen/obj.endCondition.limitGenerationNumber)*100);
            nameTTF = mystica.utils.createTimeTrackerFile(nameTTF,'GA',(obj.indexGen/obj.endCondition.limitGenerationNumber)*100,100);
        end

        if (obj.counterConstantPopulation > obj.endCondition.limitCounterConstantPopulation) && ...
                obj.stats.minFitValues(obj.indexGen) > obj.endCondition.thresholdFitValues
            continueWhileLoop = 0;
            fprintf('GA | Algorithm converged after %i generations (time %is)\n',obj.indexGen,round(toc(obj.time)));
        end

        if obj.indexGen >= obj.endCondition.limitGenerationNumber
            continueWhileLoop = 0;
            fprintf('GA | Algorithm terminaned after %i generations (time %is)\n',obj.endCondition.limitGenerationNumber,round(toc(obj.time)));
        end

    end

    if obj.indexGen < obj.endCondition.limitGenerationNumber
        obj.getStats('delete')
    end
    obj.getStats('print')
    delete(nameTTF)

end
