clear all
close all
tic 
%close all
%figure('units','normalized','outerposition',[0 0 1 1])
smoothingOn = false;
pennPoints = [];

%Define startPos
startPos =  [200 150]; % [x-coordinate y-coordinate]
%startPos = [735 241];
%startPos(1) = 1074; % x-coordinate
%startPos(2) = 260;  % y-coordinate

%% Define finishPos
finishPos = [400 500];
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
load masksFull10 % (1,1) is southwest corner of region
load SpeedLauri

% load speedFull10

[longitude, latitude] = calculateCoordinates2;

%1000:1600,2800:3600
%1050:1350,2900:3300

%minY = 900;
%maxY = 1400;
%minX = 2700;
%maxX = 3600;

minY = 1;
maxY = 556;
minX = 1;
maxX = 827;

%% Create masks and speed matrices

depthMask = depthMask(minY:maxY,minX:maxX);    % 1 = depth meets the requirements 0= not meeting requirements, (1,1) is southwest corner
continentMask = continentMask(minY:maxY,minX:maxX); % 1 = continent, 0 = not a continent, (1,1) is southwest corner
speed = speed(minY:maxY,minX:maxX);
%longitude = fliplr(longitude(minY:maxY,minX:maxX)');
%latitude  = fliplr(latitude(minY:maxY,minX:maxX)');
min(longitude(:));
min(latitude(:));
max(latitude(:));

% sh = loadShipTrack(latitude, longitude);
% 
% numInTrack = size(sh,1);
% trackPoints = zeros(numInTrack,2);
% prevX = 0;
% prevY = 0;
% j=1;
% for i=1:numInTrack
%     [X, Y] = calcXY(latitude,longitude,sh(i,1),sh(i,2));
%     if (X~=prevX || Y~=prevY)
%         trackPoints(j,:) = [X, Y];
%         j=j+1;
%         prevX = X;
%         prevY = Y;
%     end
% 
% end
% trackPoints = trackPoints(1:j-1,:);
% plot(trackPoints(:,1),trackPoints(:,2),'y*-');

% Note: the below line transforms the speed matrix to match the same
% definition as whichList, i.e. rows=x-coordinate, columns=y-coordinate
% with (1,1) as the southwest corner (in other words, a normal map
% projection)
speed = speed'; 
inverseSpeed = 1 ./speed;

[mapRows, mapCols] = size(depthMask);

load(['boundaries', num2str(minY), '-', num2str(maxY), '-', num2str(minX), '-', num2str(maxX)])


%% plot speed values
hold on
plotSpeedValues(speed);

%% Set walkable and unwalkable 'constantsclose all'
walkable = 1;
unwalkable = 4;
continent = 5;

% Create whichList array    note: choose sparse or non-sparse representation below 
% Note that whichList is defined differently than depthMask and
% continentMask. whichList is defined with columns representing x-coordinates
% and rows representing y-coordinates
% whichList = sparse(mapCols, mapRows);
whichList = zeros(mapCols,mapRows);

% Get "unnavigable" indices from depthMask
indexObstacles = find(depthMask'==0);

% Get navigable indices from depthMask
indexWaterRegions = find(depthMask'==1);

% Assign these as unnavigable in whichList
whichList(indexObstacles) = unwalkable;


% Plot whichList
% tempWhichList = whichList;
% indicesLand = find(whichList==4);
% tempWhichList(indicesLand)=1;
% imshow(full(tempWhichList))

%% Plot water regions with black
%figure
% for i = 1:size(indexWaterRegions,1)
%    [m, n] = ind2sub(size(whichList),indexWaterRegions(i));
%    plotPoint([m, n],'k');
% end

% Get "continent" indices from paths
indexContinents = find(continentMask'==1);

% Assign these as unnavigable in whichList
whichList(indexContinents) = continent;

plotPoint(finishPos, 'r');

hold on

plotPoint(startPos,'r');

% Calculate ice breaker waypoints in [X, Y]
numWaypoints = size(waypointsLatLong,1);
waypoints = zeros(numWaypoints,2);
for i=1:numWaypoints
    [X, Y] = calcXY2(latitude,longitude,waypointsLatLong(i,1),waypointsLatLong(i,2));
    waypoints(i,:) = [X, Y];
end
% scatter(waypoints(:,1),waypoints(:,2),'b*');

% full speed
speedOpt = 1;

%% plot boundaries
s=1;
numberOfObjects = length(boundaries);
continents = cell(1,1);
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



%% find penninsula points
%[pennPoints, hList] = findPenninsulaPoints4([0, mapCols, 0, mapRows], ...
%               continents,startPos, finishPos);

% numHandles = size(hList,1);
% for i=2:numHandles
%     if ishandle(hList(i))
%         delete(hList(i))
%     end
% end        
%pennPoints = sortrows(pennPoints,3);
%pennPoints = pennPoints(:,1:2);

if ~isempty(pennPoints)
    usePennPoints = true;
else
    usePennPoints = false;
end

%%
if usePennPoints
    %pennPoints = [pennPoints; flipud(waypoints)];
    %define P values

    %Pvalues = findPvalues3(pennPoints, startPos, finishPos, whichList,boundaries, [0, mapCols, 0, mapRows]);
    
    % Call the Astar function
    [pathMatrix, pathArray, Gcost, dist] = AstarPenn3(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
     pennPoints, latitude, longitude, Pvalues, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn);
else
    [pathMatrix, pathArray, Gcost] = Astar(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
       latitude, longitude, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn); 
end
%update figure
drawnow;
toc


    
%pathMatrix = AstarPenn(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
%     pennPoints, Pvalues, whichList, speedOpt, drawOpt, smoothingOn); 
save('pathOutput20140307','pathMatrix','pathArray');

