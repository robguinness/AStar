function [depthMask,continentMask] = depthMaskEvaluation(D,depth)
%DEPTHMASKEVALUATION determnines two masks one for depth the other for continents. 
% INPUT
%   depth - GEBCO 30 arc-second global grid of elevations in meters
%   obtained as an array for the Baltic and North Sea from http://www.gebco.net/data_and_products/gridded_bathymetry_data/
%   D - safe depth of water in meters, determined by the user

%OUTPUT
%   depthMask
%   continentMask

load depth
D=readtable('INsafeDepth.txt');
D=table2array(D);
maskValue = -D;

[numRows, numCols] = size(depth);
depthMask = true(numRows, numCols);
% sufficient depth = 0; insufficient depth = 1
for i = 1:numCols
    [rows, cols] = find(depth(:,i)<=maskValue);
    depthMask(rows,i)=0;
end

[numRows, numCols] = size(depth);
continentMask = false(numRows, numCols);
% continent is defined as mass of land at the elevation of sea level
% and higher (=> 0m)
for i = 1:numCols
    [rows, cols] = find(depth(:,i)>=0);
    continentMask(rows,i)=1;
end

save(['environment/maskFull' num2str(abs(maskValue))],'depthMask','continentMask');

end
