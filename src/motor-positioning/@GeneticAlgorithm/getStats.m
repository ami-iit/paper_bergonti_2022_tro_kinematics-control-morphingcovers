function getStats(obj,phase)

    switch phase
        case 'config'
            obj.stats.time(           obj.endCondition.limitGenerationNumber) = 0;
            obj.stats.maxFitValues(   obj.endCondition.limitGenerationNumber) = 0;
            obj.stats.minFitValues(   obj.endCondition.limitGenerationNumber) = 0;
            obj.stats.meanFitValues(  obj.endCondition.limitGenerationNumber) = 0;
            obj.stats.medianFitValues(obj.endCondition.limitGenerationNumber) = 0;
            obj.stats.stdFitValues(   obj.endCondition.limitGenerationNumber) = 0;
        case 'update'
            obj.stats.time(           obj.indexGen) = toc(   obj.time);
            obj.stats.maxFitValues(   obj.indexGen) = max(   obj.population.fitValues);
            obj.stats.minFitValues(   obj.indexGen) = min(   obj.population.fitValues);
            obj.stats.meanFitValues(  obj.indexGen) = mean(  obj.population.fitValues);
            obj.stats.medianFitValues(obj.indexGen) = median(obj.population.fitValues);
            obj.stats.stdFitValues(   obj.indexGen) = std(   obj.population.fitValues);
        case 'delete'
            obj.stats.time(           obj.indexGen+1:end) = [];
            obj.stats.maxFitValues(   obj.indexGen+1:end) = [];
            obj.stats.minFitValues(   obj.indexGen+1:end) = [];
            obj.stats.meanFitValues(  obj.indexGen+1:end) = [];
            obj.stats.medianFitValues(obj.indexGen+1:end) = [];
            obj.stats.stdFitValues(   obj.indexGen+1:end) = [];
        case 'print'
            fprintf('GA | Final Population fitValues \n   | max : %g \n   | min : %g \n   | mean: %g\n',...
                obj.stats.maxFitValues( obj.indexGen),...
                obj.stats.minFitValues( obj.indexGen),...
                obj.stats.meanFitValues(obj.indexGen))
    end

end
