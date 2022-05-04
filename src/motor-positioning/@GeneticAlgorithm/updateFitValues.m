function updateFitValues(obj)


    obj.children.combinationDec = obj.getDecimalPopulation(obj.children.combinationBin);

    for i = 1 : obj.stgs.populationSize.children

        if  all( obj.combinationDecRange(2) >= obj.children.combinationDec(i,:) ) && ...
            all( obj.combinationDecRange(1) <= obj.children.combinationDec(i,:) )

            obj.children.fitValues(i) = obj.fitnessFunction(obj.children.combinationDec(i,:));
        else
            obj.children.fitValues(i) = 0;
        end
    end

end
