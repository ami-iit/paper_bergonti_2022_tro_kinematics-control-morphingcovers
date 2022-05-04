function combinationBin = getBinaryCombination(obj,combinationDec)
    arguments
        obj
        combinationDec (:,1)
    end
    combinationBin = transpose(str2num(reshape(dec2bin(combinationDec,obj.combinationBinGeneLength)',[],1)));
end
