function [pathCoordinates,speedAlongPath,timeAlongPath] = processResults(speed,drawResults,longitude,latitude,pathArray)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path

[latout,longout] = reducem(pathArray(:,2),pathArray(:,1));
pathReduced=[latout, longout];

if drawResults
    % 0.5 is subtracted for visualization purposes
    plot(pathArray(:,1)-0.5,pathArray(:,2)-0.5,'Color','r','LineWidth',3);
    plot(pathReduced(:,2)-0.5,pathReduced(:,1)-0.5,'x');
end

sizePathReduced=size(pathReduced);
for i=1:sizePathReduced
    speedAlongPath(i)=speed(pathReduced(i,2),pathReduced(i,1));
end
speedAlongPath=speedAlongPath.*1.852; % Speed converted to knots from original m/s

for i=1:1:sizePathReduced
    COORD(i)=longitude(pathReduced(i,2),pathReduced(i,1));
    COORD2(i)=latitude(pathReduced(i,2),pathReduced(i,1));
end
pathCoordinates=[COORD', COORD2'];

for i=1:1:sizePathReduced-1
    [arclen,az] = distance('rh',COORD2(i),COORD(i),COORD2(i+1),COORD(i+1));
    dist(i)=arclen;
    AZ(i)=az;
end
pathLength=sum(dist)*60; % distance expressed in Nautical Miles

for i=1:1:sizePathReduced-1
    dist2(i)=sqrt((pathReduced(i+1,2)-pathReduced(i,2))^2+(pathReduced(i+1,1)-pathReduced(i,1))^2);
    timeAlongPath(i)=(60.*dist(i))./speedAlongPath(i+1);
end
timeAlongPath=sum(timeAlongPath); %Time in hours


end