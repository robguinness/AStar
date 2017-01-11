clc
clear all
close all
tic
prof=0;

if prof==1
    profiler on
    profile on
end
    
%% Create figure
%h = figure;

%% Initializing routing algorithm
[search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime,speed] = init();

%% Starting A* algorithm
fprintf('Starting A* algorithm...')
[pathMatrix, pathArray, drawResults,timeCoordinateSpeedMatrix] = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime);

%update figure
% drawnow;
%% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path
[pathCoordinates,speedAlongPath,timeAlongPath,pathLength] = processResults(speed,drawResults,longitude,latitude,pathArray);

%% Saving relevant information as text files,calculating overall computation time and clearing the workspace
[pC,sAP,tAP,pL] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath,pathLength);
datetime('now','InputFormat','YY-MM-DD-HH-MM-SS');

timeOverall=toc/60;
t=datestr(now,'mmddyyyy_HHMM');
save(['results/' t '_timeOverall.txt'], 'timeOverall', '-ascii');

%% Saving relevant information as kml files, for visualization purposes
kmlwriteline(['results/' t '_path.kml'],pathCoordinates(:,2),pathCoordinates(:,1),speedAlongPath(:,1));
waypointsLatLong = readtable('INwaypointsIB');
waypointsLatLong=table2array(waypointsLatLong);
kmlwrite('results/IBWPs.kml',waypointsLatLong(:,1),waypointsLatLong(:,2));

%% Profiler results
if prof==1
    profile viewer
end