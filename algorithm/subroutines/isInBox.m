function inBox = isInBoundingBox(M,B)
    inBox = (M(:,1)>=B(1)).*(M(:,1)<=B(3)).*(M(:,2)>=B(2)).*(M(:,2)<=B(4));
end