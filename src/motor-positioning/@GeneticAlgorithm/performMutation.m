function performMutation(obj)

    indexesMutation = rand(obj.stgs.populationSize.children,obj.combinationBinLength) <= obj.stgs.probabilityMutation;

    obj.children.combinationBin(indexesMutation) = not(obj.children.combinationBin(indexesMutation));

end
