tic 
addpath environment environment/penninsulas
addpath ships
addpath utilities
addpath algorithm

disp('Initializing the program...')

%close all
%figure('units','normalized','outerposition',[0 0 1 1])
smoothingOn = false;

pennPoints = [];

%Define startPos
startPos =  [126 65];
%startPos = [735 241];
%startPos(1) = 1074; % x-coordinate
%startPos(2) = 260;  % y-coordinate

%% Define finishPos
finishPos = [1011 267];
% finishPos(1) = 39; % x-coordinate
% finishPos(2) = 20;  % y-coordinate

% Define ice breaker waypoints
%waypoints = [340, 290; 330, 280; 320, 275; 300, 270];
%cIB = 0.5;
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	1	59.43333333	23
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	2	59.55	24.13333333
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	3	59.73333333	24.61666667
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	4	59.83333333	25.43333333
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	5	59.91666667	26
%URH	Urho 2602	26.2.2011 18:37	5.3.2011 10:17	6	60.08333333	26.26666667
waypointsLatLong = [59.4333333333333, 23; ...
    59.55, 24.1333333333333; ...
    59.73333333, 24.61666667; ...
    59.83333333,	25.43333333; ...
    59.91666667,	26;...
    60.08333333,	26.26666667;];


drawOpt = false;
useSaved = false;

%load paths
%paths = ones(50,50);
%[mapRows, mapCols] = size(paths);
fprintf('Loading depth and speed data...')
load environment/masksFull10
load environment/speedFull10
fprintf('done.\n')

fprintf('Calculating geographic coordinates...')
[longitude, latitude] = calculateCoordinates;
fprintf('done.\n')

%1000:1600,2800:3600
%1050:1350,2900:3300

%minY = 900;
%maxY = 1400;
%minX = 2700;
%maxX = 3600;

minY = 1100;
maxY = 1450;
minX = 3100;
maxX = 4400;

%% Create masks and speed matrices

fprintf('Resizing matrices...')
depthMask = depthMask(minY:maxY,minX:maxX);
continentMask = continentMask(minY:maxY,minX:maxX);
speed = speed(minY:maxY,minX:maxX);
longitude = fliplr(longitude(minY:maxY,minX:maxX)');
latitude  = fliplr(latitude(minY:maxY,minX:maxX)');
fprintf('done.\n')

fprintf('Loading ship tracks...')
sh = loadShipTrack(latitude, longitude);
fprintf('done.\n')



fprintf('Plotting ship tracks...')
numInTrack = size(sh,1);
trackPoints = zeros(numInTrack,2);
prevX = 0;
prevY = 0;
j=1;
for i=1:numInTrack
    [X, Y] = calcXY(latitude,longitude,sh(i,1),sh(i,2));
    if (X~=prevX || Y~=prevY)
        trackPoints(j,:) = [X, Y];
        j=j+1;
        prevX = X;
        prevY = Y;
    end

end
trackPoints = trackPoints(1:j-1,:);
plot(trackPoints(:,1),trackPoints(:,2),'y*-');
fprintf('done.\n')


speed = fliplr(speed');
inverseSpeed = 1 ./speed;

[mapRows, mapCols] = size(depthMask);

fprintf('Loading "boundary" data...')
load(['boundaries', num2str(minY), '-', num2str(maxY), '-', num2str(minX), '-', num2str(maxX)])
fprintf('done.\n')

hold on

%% plot speed values
%fprintf('Plotting speed data...')
%plotSpeedValues(speed);
%fprintf('done.\n')

% Set walkable and unwalkable 'constantsclose all'
walkable = 1;
unwalkable = 4;
continent = 5;

% Create whichList array
whichList = sparse(mapCols, mapRows);


fprintf('Assigning unnavigable nodes as "unwalkable"...')
% Get "unnavigable" indices from depthMask
indexObstacles = find(fliplr(depthMask')==0);

% Assign these as unnavigable in whichList
whichList(indexObstacles) = unwalkable;
fprintf('done.\n')

%% Plot these
%fprintf('Plotting obstacles...')
%for i = 1:size(indexObstacles,1)
%   [m, n] = ind2sub(size(whichList),indexObstacles(i));
%   plotPoint([m, n],'k');
%end
%fprintf('done.\n')

% Get "continent" indices from paths
indexContinents = find(fliplr(continentMask')==1);

% Assign these as unnavigable in whichList
whichList(indexContinents) = continent;

fprintf('Plotting start and finish points...')
plotPoint(finishPos, 'r');

hold on

plotPoint(startPos,'r');
fprintf('done.\n')

fprintf('Plotting ice breaker waypoints...')
% Calculate ice breaker waypoints in [X, Y]
numWaypoints = size(waypointsLatLong,1);
waypoints = zeros(numWaypoints,2);
for i=1:numWaypoints
    [X, Y] = calcXY(latitude,longitude,waypointsLatLong(i,1),waypointsLatLong(i,2));
    waypoints(i,:) = [X, Y];
end
scatter(waypoints(:,1),waypoints(:,2),'b*');
fprintf('done.\n')

% full speed
speedOpt = 1;

%% plot boundaries
% fprintf('Plotting boundaries...')
% s=1;
% numberOfObjects = length(boundaries);
% continents = cell(1,1);
% for k = 1:numberOfObjects
%         object = boundaries{k,1};
%         boundary = [object(:,2), mapRows - object(:,1)];
%         plot(boundary(:,1),boundary(:,2))
%         penn = boundaries{k,2};
%         if (penn == 1)
%             continents(s,1) = {boundary};
%             s = s+1;
%         end
% end
% fprintf('done.\n')


%% find penninsula points
% comment the below line out, if you have already calculated pennPoints and
% hList (speeds up the execution)
%[pennPoints, hList] = findPenninsulaPoints4([0, mapCols, 0, mapRows], ...
%               continents,startPos, finishPos);

%numHandles = size(hList,1);
%for i=2:numHandles
%    if ishandle(hList(i))
%        delete(hList(i))
%    end
%end        
%pennPoints = sortrows(pennPoints,3);
%pennPoints = pennPoints(:,1:2);

if ~isempty(pennPoints)
    usePennPoints = true;
else
    usePennPoints = false;
end

%%
fprintf('Starting A* algorithm...')
if usePennPoints
    %pennPoints = [pennPoints; flipud(waypoints)];
    %define P values

    %Pvalues = findPvalues3(pennPoints, startPos, finishPos, whichList,boundaries, [0, mapCols, 0, mapRows]);
    toc
    % Call the Astar function
    [pathMatrix, pathArray, Gcost, dist] = AstarPenn3(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
       pennPoints, latitude, longitude, Pvalues, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn);
else
    pathMatrix = Astar(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
       latitude, longitude, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn);
end
%update figure
drawnow;



    
%pathMatrix = AstarPenn(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
%     pennPoints, Pvalues, whichList, speedOpt, drawOpt, smoothingOn); 
save('pathOutput20140307','pathMatrix','pathArray','Pvalues');

