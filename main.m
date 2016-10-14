clc
clear all
close all
tic
% profiler on
profile on

%% Create figure
%h = figure;

%% Initializing routing algorithm
[search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime,speed] = init();

%% Starting A* algorithm
fprintf('Starting A* algorithm...')
[pathMatrix, pathArray, drawResults] = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime);

%update figure
% drawnow;
%% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path
[pathCoordinates,speedAlongPath,timeAlongPath] = processResults(speed,drawResults,longitude,latitude,pathArray);

%% Saving relevant information as text files,calculating overall computation time and clearing the workspace
[pC,sAP,tAP] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath);
datetime('now','InputFormat','YY-MM-DD-HH-MM-SS');

timeOverall=toc/60;
t=datestr(now,'mmddyyyy_HHMM');
save(['results/' t '_timeOverall.txt'], 'timeOverall', '-ascii');

profile viewer
%clear all