

%% Starting A* algorithm
fprintf('Starting A* algorithm...')
[pathMatrix, pathArray] = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime);

%update figure
% drawnow;
%% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path
[pathCoordinates,speedAlongPath,timeAlongPath] = processResults(speed,drawResults,longitude,latitude,pathArray);

%% Saving relevant information as text files
[pC,sAP,tAP] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath);
datetime('now','InputFormat','YY-MM-DD-HH-MM-SS');


%clear all
