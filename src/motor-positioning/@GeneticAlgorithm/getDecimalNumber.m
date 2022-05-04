function numDec = getDecimalNumber(obj,numBin)
    arguments
        obj
        numBin (:,1)
    end
    numDec = transpose(obj.matrixFromBinaryNumber2DecimalNumber * numBin);
end
