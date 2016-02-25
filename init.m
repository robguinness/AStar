%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the necesary data structures and run-time
% parameters needed to execute the A* algorithm
% 
% Version History
% (ver. #)   (date)        (author of new version)    (see notes)
%  v0.1      24.2.2016      R. Guinness                none
%
% DEFINITIONS: (change definitions only with great caution!
% 
% search         data structure containing information about the origin point and
%                destination point for the ship, defining the desired starting and
%                ending points between which a route should be found. Contains
%                following:
%
%                .originX       x-coordinate of the origin
%                .originY       y-coordinate of the origin
%                .destinationX  x-coordinate of the destination
%                .destinationY  y-coordinate of the destination
%
% depthMask      a binary matrix where the elements in the matrix represent 
%                whether or not the depth requirements for a particular ships 
%                are met at the specified location. A value of "0" indicates
%                the requirements are met, whereas a value of "1" indicates
%                the depth requirements are not met. Rows in the matrix
%                represent y-coordinates and columns represent x-coordiantes.
%                Element (1,1) is the Northwest corner of the area
%                
%                --------------------------------------------
%                  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10|
%                --------------------------------------------
%                |1| NW   0   0   0   0   0   0   0   0   NE|
%                |2| 0    0   0   0   0   0   0   0   1   1 |
%                |3| 0    0   0   0   0   1   1   1   1   1 |
%                |4| 0    0   0   0   0   1   1   1   1   1 |  <- land in SE
%                |5| SW   0   0   0   1   1   1   1   1   SE|     corner of region
%                 --------------------------------------------
%                 
% continentMask  a binary matrix where the elements in the matrix represent 
%                whether or not there is a continent present at the specified location
%                A value of "0" indicates no continent whereas a value of "1" indicates
%                a contient. Otherwise, the definition is the same as depthMask
%
% waypointsLatLong
% waypointsXY
% longitude
% latitude
% inverseSpeed
% speed
% whichList
% shipTrackLatLong
% shipTrackXY

disp('Initializing the program...')

%% Misc. initialization
clc
clear all
close all
hold on
startTime = tic;

%% Set-up path
addpath environment environment/penninsulas
addpath ships
addpath utilities
addpath algorithm

%% Define data structures

% Define search structure
search = struct('originX',0,'originY',0,'destinationX',0,'destinationY',0);

% Used for penninsula points
pennPoints = [];

%% Define constants
global UNAVIGABLE; UNAVIGABLE = 4;
global CONTINENT; CONTINENT= 5;


%% Run-time parameters
smoothingOn = false;
drawOpt = true;
useSaved = false;
speedOpt = 1;   % 1 = full speed

%% Define search parameters

% Define startPos
search.originX = 200;       % x-coordinate
search.originY = 30;        % y-coordinate

% Define finishPos
search.destinationX = 1000;  % x-coordinate
search.destinationY = 150;  % y-coordinate

% Define search area
minX = 2800;
maxX = 4000;
minY = 1050;
maxY = 1350;

% Define ice breaker waypoints

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

%% Load saved data

fprintf('Loading depth and speed data...')
load environment/masksFull10                                       % This loads a "depth mask" for the whole Baltic sea
load environment/speedFull10                                       % This loads a speed grid for the whole Baltic sea 
%speed = ones(5000,5000);                                          % temporary solution with uniform speed
fprintf('done.\n')

fprintf('Loading "boundary" data...')
load(['boundaries', num2str(minY), '-', num2str(maxY), '-', num2str(minX), '-', num2str(maxX)])  %TODO: Reorder so that X comes before Y
fprintf('done.\n')

%% Calculate geographic coordinates

fprintf('Calculating geographic coordinates...')
[longitude, latitude] = calculateCoordinates;
longitude = fliplr(longitude(minY:maxY,minX:maxX)');
latitude  = fliplr(latitude(minY:maxY,minX:maxX)');
fprintf('done.\n')

%% Create masks and speed matrices

fprintf('Resizing matrices...')
depthMask = depthMask(minY:maxY,minX:maxX);
[mapRows, mapCols] = size(depthMask);
continentMask = continentMask(minY:maxY,minX:maxX);
speed = speed(minY:maxY,minX:maxX);
speed = fliplr(speed');
inverseSpeed = 1 ./speed;
fprintf('done.\n')

%% Create whichList array
whichList = sparse(mapCols, mapRows);

%% Historical ship data

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

%% Define obstacles

fprintf('Assigning unnavigable nodes in whichList...')
% Get "unnavigable" indices from depthMask
indexObstacles = find(fliplr(depthMask')==1);

% Assign these as unnavigable in whichList
whichList(indexObstacles) = UNAVIGABLE;
fprintf('done.\n')

%% Plot sea environment

% Normalize the speed values to be between 0 and 63;
speedNormalized = normalize(speed)*63;

% Assign obstacles a value of 64 (highest in colormap
speedNormalized(indexObstacles) = 64;

% set up colormap
colormap cool
cmap = colormap;
cmap(64,:) = [0 0 0];
colormap(cmap);

% plot the matrix and do other setup
image(speedNormalized')
axis([1 mapCols 1 mapRows])
colorbar

%% Set up continent data

% Get "continent" indices from paths
indexContinents = find(continentMask'==1);

% Assign these as unnavigable in whichList
whichList(indexContinents) = CONTINENT;

%% Plot origin and destination points

fprintf('Plotting start and finish points...')
plotPoint([search.originX, search.originY],'r');
plotPoint([search.destinationX, search.destinationY], 'r');
fprintf('done.\n')

%% Plot ice breaker waypoints

fprintf('Plotting ice breaker waypoints...')

% Calculate ice breaker waypoints in [X, Y]
numWaypoints = size(waypointsLatLong,1);
waypoints = zeros(numWaypoints,2);
parfor i=1:numWaypoints
    [X, Y] = calcXY(latitude,longitude,waypointsLatLong(i,1),waypointsLatLong(i,2));
    waypoints(i,:) = [X, Y];
end
scatter(waypoints(:,1),waypoints(:,2),'b*');
fprintf('done.\n')

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

timeForSetup = toc;
fprintf('Time for set-up was %.2f seconds.\n',timeForSetup);

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
    pathMatrix = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn, startTime);
end
%update figure
drawnow;

   
%pathMatrix = AstarPenn(startPos(1), startPos(2), finishPos(1), finishPos(2), ...
%     pennPoints, Pvalues, whichList, speedOpt, drawOpt, smoothingOn); 
save('pathOutput20140307','pathMatrix','pathArray','Pvalues');

