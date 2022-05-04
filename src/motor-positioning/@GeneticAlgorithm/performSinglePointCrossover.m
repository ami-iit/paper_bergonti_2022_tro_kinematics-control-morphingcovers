function performSinglePointCrossover(obj)

    indexCrossover = randi([1 obj.combinationBinLength-1],1,obj.stgs.populationSize.children/2);
    probCrossover  = rand(obj.stgs.populationSize.children/2,1);

    % p1=[0 0 1 | 0 0 0 1]
    % p2=[0 1 0 | 1 0 1 1]
    %           x
    %
    % indexCrossover = 3
    %
    % c1=[0 0 1 | 1 0 1 1]
    % c2=[0 1 0 | 0 0 0 1]

    indexParents = reshape(obj.children.parentsIndexes,[],2);
    parent1 = obj.population.combinationBin(indexParents(:,1),:);
    parent2 = obj.population.combinationBin(indexParents(:,2),:);

    B = zeros(size(parent1));
    % Perform crossover
    B(sub2ind(size(parent1),1:size(parent1,1),indexCrossover)) = 1;
    B = cumsum(B,2);
    B = (B == 0);
    % No crossover => the children are equal to the parents
    B(obj.stgs.probabilityCrossover < probCrossover,:) = 1;

    obj.children.combinationBin = [parent1.*B       + parent2.*not(B);...
                                   parent1.*not(B)  + parent2.*B];

end
