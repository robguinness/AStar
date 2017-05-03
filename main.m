function main(env_path, in_path, out_path, optimize, evaluateRoute)

if (nargin == 0)
    env_path = 'environment';
    in_path = '.';
    out_path = 'results';
end

if nargin < 4
    optimize = '1';
    evaluateRoute = '0';
end

if (isdeployed == 0) 
    clc
    %clear all
    close all
end
tic
prof=0;

if prof==1
    profiler on
    profile on
end
    
%% Create figure
%h = figure;

%% Initializing routing algorithm
[search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime,speed,stuck,iceDataTimes] = init(env_path, in_path, out_path);

writeWeatherXML(speed, stuck, whichList, iceDataTimes, latitude, longitude, strcat(out_path,'/OUTSpeedStuckData.xml'));

if (optimize == '1')

    %% Starting A* algorithm
    fprintf('Starting A* algorithm...')
    [pathMatrix, pathAndSpeedArray, drawResults,timeCoordinateSpeedMatrix] = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime, env_path, in_path);

    %update figure
    % drawnow;
    %% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path
    [pathCoordinates,speedAlongPath,timeAlongPath,pathLength] = processResults(speed,drawResults,longitude,latitude,pathAndSpeedArray);

    %% Saving relevant information as text files,calculating overall computation time and clearing the workspace
    [pC,sAP,tAP,pL] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath,pathLength, out_path);
    datetime('now','InputFormat','YY-MM-DD-HH-MM-SS');

    timeToComputeRoute=toc/60;
    %t=datestr(now,'mmddyyyy_HHMM');
    save(strcat(out_path,'/timeToComputeRoute.txt'), 'timeToComputeRoute', '-ascii');

    %% Saving relevant information as kml files, for visualization purposes
    kmlwriteline(strcat(out_path,'/path.kml'),pathCoordinates(:,2),pathCoordinates(:,1),speedAlongPath(1,:));
    waypointsLatLong = readtable(strcat(in_path,'/INwaypointsIB'));
    waypointsLatLong=table2array(waypointsLatLong);
    kmlwrite(strcat(out_path,'/IBWPs.kml'),waypointsLatLong(:,1),waypointsLatLong(:,2));

    %% Profiler results
    if prof==1
        profile viewer
    end

end

%% Additions by Lauri Seitsonen Feb 2017

% Minimum speed used in calculations (used if actual speed below it)
minSpeed = 0.1;

% Read route start time from input file
voyageStartTime = datetime(table2array(readtable(strcat(in_path,'/INvoyageStartTime'))), 'TimeZone','UTC');

% Calculate time estimate for the optimized route using speed from all the
% grid cells that are on the path
if (optimize == '1')
    % Calculate time estimate for the original two-point route
    [ arrivalTime, totalDistanceOriginalNm, timeEstimateOriginal, avgSpeedOriginalKn ] = calculateTime2( search, speed, minSpeed, voyageStartTime, iceDataTimes, latitude, longitude );
    
    timeEstimateOptimized = 0;
    currentTime = voyageStartTime;
    speedEstimateOptimized = zeros(1,size(pathCoordinates,1)-1);
    for i=1:1:size(pathCoordinates,1)-1
        tmpSearch.originLat = pathCoordinates(i,2);       % Lat coordinate
        tmpSearch.originLong = pathCoordinates(i,1);      % Long coordinate
        tmpSearch.destinationLat = pathCoordinates(i+1,2);       % Lat coordinate
        tmpSearch.destinationLong = pathCoordinates(i+1,1);      % Long coordinate
        [tmpSearch.originX,tmpSearch.originY]=calcXY(latitude, longitude, tmpSearch.originLat,tmpSearch.originLong);
        [tmpSearch.destinationX,tmpSearch.destinationY]=calcXY(latitude, longitude, tmpSearch.destinationLat,tmpSearch.destinationLong);
        
%        [ xdistances, xspeeds, xtimes, xjammings ] = calculateTime(tmpSearch,speed,0.1,2,1, voyageStartTime, times);
%        legTime = sum(xtimes);
        [ currentTime, totalDistanceNm, legTime, avgSpeedKn ] = calculateTime2( tmpSearch, speed, minSpeed, currentTime, iceDataTimes, latitude, longitude );
        
%        if (legTime ~= 0)
%            legSpeeds(i) = sum(xdistances) / legTime;
%        end
        timeEstimateOptimized = timeEstimateOptimized + legTime;
        speedEstimateOptimized(i) = avgSpeedKn;

        clearvars tmpSearch
    end
    
    T = table(timeEstimateOriginal,avgSpeedOriginalKn,timeEstimateOptimized);
    writetable(T,strcat(out_path,'/OUTtimesAlongPath.txt'));
    
    S=array2table(speedEstimateOptimized','VariableNames',{'knots'}); 
    writetable(S,strcat(out_path,'/OUTspeedsAlongPath.txt'));
end

% Calculate time estimate for multi-leg route if one given
mytime = 0;
currentTime = voyageStartTime;
if evaluateRoute == '1'
    routeInput=readtable(strcat(in_path,'/INrouteCoordinates'));
    routeInputArray = table2array(routeInput);
    clearvars routeInput
    legSpeeds = zeros(1,size(routeInputArray,1)-1);
    
    for i=1:1:size(routeInputArray,1)-1
        tmpSearch.originLat = routeInputArray(i,1);       % Lat coordinate
        tmpSearch.originLong = routeInputArray(i,2);      % Long coordinate
        tmpSearch.destinationLat = routeInputArray(i+1,1);       % Lat coordinate
        tmpSearch.destinationLong = routeInputArray(i+1,2);      % Long coordinate
        [tmpSearch.originX,tmpSearch.originY]=calcXY(latitude, longitude, tmpSearch.originLat,tmpSearch.originLong);
        [tmpSearch.destinationX,tmpSearch.destinationY]=calcXY(latitude, longitude, tmpSearch.destinationLat,tmpSearch.destinationLong);
        
        %[ distances, speeds, times, jammings ] = calculateTime(tmpSearch,speed,minSpeed,jammingSpeedThreshold,minJammingInterval);
        [ currentTime, legDistanceNm, legTime, legAvgSpeedKn ] = calculateTime2( tmpSearch, speed, minSpeed, currentTime, iceDataTimes, latitude, longitude );

        legSpeeds(i) = legAvgSpeedKn;
        mytime = mytime + legTime;

        clearvars tmpSearch
    end
    
    T=array2table(legSpeeds','VariableNames',{'knots'});
    writetable(T,strcat(out_path,'/OUTspeedsAlongRoute.txt'));
end

%clear all

end
