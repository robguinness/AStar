load depth
depth = depth(1403:1500,3275:3450);
maskValue = -10;
[numRows, numCols] = size(depth);
depthMask = true(numRows, numCols);
for i = 1:numCols
    [rows, cols] = find(depth(:,i)<=maskValue);
    depthMask(rows,i)=0;
end

[boundaries labels] = bwboundaries(depthMask, 'noholes');

numberOfObjects = length(boundaries);
for k = 1:numberOfObjects
    plot(boundaries{k,1})
end

save(['depthMaskArch' num2str(abs(maskValue))],'depthMask');