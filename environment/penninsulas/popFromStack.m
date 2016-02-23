function [intPointA, intPointB, fractionA, fractionB, checkPointStack] = popFromStack(checkPointStack)
    intPointA = checkPointStack(1,1:2);
    intPointB = checkPointStack(1,3:4);
    fractionA = checkPointStack(1,5);
    fractionB = checkPointStack(1,6);
    checkPointStack = checkPointStack(2:end,:);
end