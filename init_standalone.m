%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the necesary data structures and run-time
% parameters needed to execute the A* algorithm
% 
% Version History
% (ver. #)   (date)        (author of new version)    (see notes)
%   v0.1      24.2.2016      R. Guinness                none
%   v0.2      24.05.2016     J. Montewka 
%   v0.3      26.08.2016     J. Montewka              attempts to make the standalone version without plotting 

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
% depthMask      a binary matrix, m by n (as opposite to other matrices),  
%                where the elements in the matrix represent whether or not 
%                the depth requirements for a particular ships 
%                are met at the specified location. A value of "0" indicates
%                the requirements are met, whereas a value of "1" indicates
%                the depth requirements are not met. Rows in the matrix
%                represent y-coordinates and columns represent x-coordiantes.
%                Element (1,1) is the Northwest corner of the area
%                
%                --------------------------------------------
%                  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | n|
%                --------------------------------------------
%                |1| NW   0   0   0   0   0   0   0   0   NE|
%                |2| 0    0   0   0   0   0   0   0   1   1 |
%                |3| 0    0   0   0   0   1   1   1   1   1 |
%                |4| 0    0   0   0   0   1   1   1   1   1 |  <- land in SE
%                |m| SW   0   0   0   1   1   1   1   1   SE|     corner of region
%                 --------------------------------------------
%                 
% continentMask  a binary matrix where the elements in the matrix represent 
%                whether or not there is a continent present at the specified location
%                A value of "0" indicates no continent whereas a value of "1" indicates
%                a contient. Otherwise, the definition is the same as depthMask
%
% waypointsLatLong  a two-column matrix storing coordinates of waypoints in ice given by Icebreaker in Lat and Long.  
%
% waypoints      a two-column matrix storing coordinates of waypoints in ice given by Icebreaker in X and Y, 
%                where X corresponds to Longitude and Y corresponds to Latitude.
%
% longitude      n by m matrix containing values of longitude for a given search area. Each row corresponds to a particular longitude 
%                (there are n longitudes in search area) that crosses all latitudes (m), represented by columns. Rows can be
%                seen as meridians and coluns as parallels.              
%
% latitude       n by m matrix containing values of latitude for a given search area. Each row corresponds to a particular longitude 
%                (there are n longitudes in search area) that crosses all latitudes (m), represented by columns. Rows can be
%                seen as meridians and coluns as parallels.    
%
% speed          n by m matrix where the elements in the matrix represent speed for a particular ship, based on ship performance model developed by AALTO. 
%                Rows in the matrix represent longitude (x-coordinates) and columns represent latitude (y-coordiantes).
%                Element (1,1) is the Southwest corner of the area - N.B. the difference in orientation between speed and depthMask matrices.
%
%                --------------------------------------------
%                  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | n|
%                --------------------------------------------
%                |1| SW   0   0   0   0   0   0   0   0   NW|
%                |2| 0    0   0   0   0   0   0   0   1   1 |
%                |3| 0    0   0   0   0   1   1   1   1   1 |
%                |4| 0    0   0   0   0   1   1   1   1   1 |  
%                |m| SE   0   0   0   1   1   1   1   1   NE| 
%                 --------------------------------------------
% inverseSpeed   a matrix containing inverse values of those stored in speed matrix (1/speed).
%
% whichList      n by m matrix containing information about each and every cell in the search area. The cells are given numbers,
%                describing their status (walkable = 1; onOpenList = 2; onClosedList = 3; unwalkable = 4; found = 1; nonexistent = 2) 
%
% shipTrackLatLong
%
% shipTrackXY
%
% stuckThresholds   if the probability of a ship getting stuck in ice exceeds certian threshold, the attainable speeds
%                   drops to a certain fraction of the initial speed. For example if P(beset)>0.3 (0.3 as a threshold), then
%                   speed=0.1*speed. This needs some more scientific justification, but can be implemented as a first try now.
%
% Margin            the value used to enlarge the search area for the algorithm. It adds a certain number of cells to all directions, based on LAT and 
%                   LONG of point of origin and point of desitnation.

% speedAalto        matrix contains speed and stuck files, both of size 556x830

disp('Initializing the program...')

%% Misc. initialization
tic
clc
clear all
close all
%hold on
startTime = tic;

%% Set-up path
addpath environment environment/penninsulas
addpath ships
addpath utilities
addpath algorithm

%% Define data structures

% Define search structure
search = struct('originX',0,'originY',0,'destinationX',0,'destinationY',0);
HELMI=struct('originX',0,'originY',0,'destinationX',0,'destinationY',0);

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

%% Define search parameters and IB waypoints

% Define arrival, departure positions and the threshold for the probability for a ship being stuck from external .txt file
departureInput=readtable('departureCoordinates');
search.originLat = table2array(departureInput(1,1));       % Lat coordinate
search.originLong = table2array(departureInput(1,2));      % Long coordinate
clearvars departureInput

arrivalInput=readtable('arrivalCoordinates');
search.destinationLat = table2array(arrivalInput(1,1));   % Lat coordinate
search.destinationLong = table2array(arrivalInput(1,2));  % Long coordinate
clearvars arrivalInput

stuckInput=readtable('stuckThreshold');
stuckThreshold=table2array(stuckInput(1,1));

% Define ice breaker waypoints
waypointsLatLong = readtable('waypointsIB');
waypointsLatLong=table2array(waypointsLatLong);

% GEBCO depth matrix definition
LatN = 70;                  % the northernmost latitude of the GEBCO depth matrix
LongW = -6;                 % the weternmost longitude of the GEBCO depth matrix (negative is west)
LatRows = 2400;             % the number of rows in the GEBCO grid
LongCols = 4800;            % the number of rows in the GEBCO grid
% LAT,LONG coordinates of HELMI grid: point of origin SW(56.74N, 016.72E), point of desitnation NE(65.99,030.48)
HELMI.originLat=56.74;
HELMI.originLong=16.72;
HELMI.destinationLat=65.99;
HELMI.destinationLong=30.48;

%% Calculate geographic coordinates
fprintf('Calculating geographic coordinates...')
[longitude, latitude] = calculateCoordinates(LatN, LongW, LatRows, LongCols);
% x,y-coordinates of origin are calculated based on Lat, Long input
[search.originX,search.originY]=calcXY(latitude, longitude, search.originLat,search.originLong);
[search.destinationX,search.destinationY]=calcXY(latitude, longitude, search.destinationLat,search.destinationLong);
[HELMI.originX,HELMI.originY]=calcXY(latitude, longitude, HELMI.originLat,HELMI.originLong);
[HELMI.destinationX,HELMI.destinationY]=calcXY(latitude, longitude, HELMI.destinationLat,HELMI.destinationLong);

%subset of GEBCO defining the search area
% XY coordinates of HELMI grid: SW(2727,809), NE(4369,1919)
MARGIN=200;
[minX, maxX, minY, maxY]=calculateSearchArea(search.originX, search.originY, search.destinationX, search.destinationY,MARGIN);

fprintf('done.\n')

%% Load saved data

fprintf('Loading depth and speed data...')
load environment/masksFull10                                          % This loads a "depth mask" for the whole Baltic sea
load environment/speedAalto                                           % This loads a speed grid for the area covered by HELMI model, calculated at AALTO. 
                                                                      % It originates in SW, and needs to be flipped to conform with the requirements - the origin needs to be in NW.
speed=flipud(speed);
stuck=flipud(stuck);

fprintf('done.\n')

% fprintf('Loading "boundary" data...')
% load(['boundaries', num2str(minY), '-', num2str(maxY), '-', num2str(minX), '-', num2str(maxX)])  %TODO: Reorder so that X comes before Y
% fprintf('done.\n')

%% Reducing the size of latitude and longitude matrices according to the already defined limits of the search area (minX-maxX, minY-maxY)

fprintf('Calculating geographic coordinates...')
longitude = longitude(minX:maxX,minY:maxY);
latitude  = latitude(minX:maxX,minY:maxY);
fprintf('done.\n')

%% Create masks and speed matrices

fprintf('Resizing matrices...')
sizeDepthMask=size(depthMask);
depthMask = depthMask((sizeDepthMask(1,1)-maxY):(sizeDepthMask(1,1)-minY),minX:maxX);
[mapRows, mapCols] = size(depthMask);
continentMask = continentMask(minY:maxY,minX:maxX);

% Here a HELMI grid-based speed matrix is embeded into GEBCO-based grid
% The X,Y coordinates of HELMI grid are hard-coded here (810:1921,2724:4383)
% The aim is to have speed matrix oriented so, that its originates in SW corner
% of a map, and its rows correspond to Longitude and column to Latitude

speed2=repelem(speed,2,2);
S=sparse(2400,4800);
sizeS=size(S);
S((sizeS(1,1)-HELMI.destinationY):(sizeS(1,1)-HELMI.originY+1),HELMI.originX:HELMI.destinationX+7)=speed2;
%S(810:1921,2724:4383)=speed2;
speed=S;
speed = speed((sizeS(1,1)-maxY):(sizeS(1,1)-minY),minX:maxX);
speed=fliplr(speed');
speed=full(speed);
inverseSpeed = 1 ./speed;
clear S;

% Here the matrix determining the probability of ship getting best in ice is
% introduced, based on AALTO's model

stuck2=repelem(stuck,2,2);
S=sparse(2400,4800);
sizeS=size(S);
S((sizeS(1,1)-HELMI.destinationY):(sizeS(1,1)-HELMI.originY+1),HELMI.originX:HELMI.destinationX+7)=stuck2;
stuck=S;
stuck = stuck((sizeS(1,1)-maxY):(sizeS(1,1)-minY),minX:maxX);
stuck=fliplr(stuck');

% Combining the speed of a ship with the probability of getting beset in
% ice, if the probability exceeds certian threshold, the attainable speeds
% drops to a certain fraction of the initial speed. For example if P(beset)>0.3 (0.3 as a threshold), then
% speed=0.1*speed.
% This needs some more scientific justification, but can be implemented as
% a first try now.
indStuck=find(stuck>stuckThreshold);
speedStuck=speed;
speedStuck(indStuck)=0.1*speed(indStuck);
inverseSpeed = 1 ./speedStuck;

% search.originX,Y is made into a new coordinate system, defined by minXY-maxXY to align with the size of whichList
search.originX=search.originX-minX;
search.originY=search.originY-minY;
search.destinationX=search.destinationX-minX;
search.destinationY=search.destinationY-minY;

fprintf('done.\n')

%% Create whichList array
whichList = sparse(mapCols, mapRows);

%% Historical ship data

% fprintf('Loading ship tracks...')
% sh = loadShipTrack(latitude, longitude);
% fprintf('done.\n')
% 
% fprintf('Plotting ship tracks...')
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
% fprintf('done.\n')

%% Define obstacles

fprintf('Assigning unnavigable nodes in whichList...')
% Get "unnavigable" indices from depthMask
indexObstacles = find(fliplr(depthMask')==1);

% Assign these as unnavigable in whichList
whichList(indexObstacles) = UNAVIGABLE;
fprintf('done.\n')

%% Plot sea environment - this section is commented for Matlab standalone version 

% % Normalize the speed values to be between 0 and 63;
% speedNormalized = normalize(speed)*63;
% 
% % Assign obstacles a value of 64 (highest in colormap
% speedNormalized(indexObstacles) = 64;
% 
% % set up colormap
% colormap cool
% cmap = colormap;
% cmap(64,:) = [0 0 0];
% colormap(cmap);
% 
% % plot the matrix and do other setup
% image(speedNormalized')
% axis([1 mapCols 1 mapRows])
% colorbar

%% Set up continent data

% Get "continent" indices from paths
indexContinents = find(continentMask'==1);

% Assign these as unnavigable in whichList
whichList(indexContinents) = CONTINENT;

%% Plot origin and destination points - this section is commented for Matlab standalone version 
% fprintf('Plotting start and finish points...')
% plotPoint([search.originX, search.originY],'r');
% plotPoint([search.destinationX, search.destinationY], 'r');
% fprintf('done.\n')

%% Plot ice breaker waypoints - plotting WP disabled for standalone version

%fprintf('Plotting ice breaker waypoints...')

% Calculate ice breaker waypoints in [X, Y]
numWaypoints = size(waypointsLatLong,1);
waypoints = zeros(numWaypoints,2);
for i=1:numWaypoints
    [X, Y] = calcXY(latitude,longitude,waypointsLatLong(i,1),waypointsLatLong(i,2));
    waypoints(i,:) = [X, Y];
end
%scatter(waypoints(:,1),waypoints(:,2),'b*');
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
toc
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
    [pathMatrix, pathArray] = Astar(search, latitude, longitude, inverseSpeed, whichList, waypoints, speedOpt, drawOpt, smoothingOn, startTime);
end
%update figure
% drawnow;
%% Reducing number of points in a path, calculating path length, speed along the path and travel time along the path
[latout,longout] = reducem(pathArray(:,2),pathArray(:,1));
pathReduced=[latout, longout];
%plot(pathReduced(:,2),pathReduced(:,1),'x');

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
%% Saving relevant information in RESULTS directory

% FileName=['path',datestr(now, 'ddmmyyyy')];
% filename=['/Users/montewka/Dropbox (MSG)/Scientific/Models/VORIC route optimization/AStar/results/' num2str(FileName) '.mat'];
% save(num2str(filename),'pathMatrix','pathArray', 'pathCoordinates', 'timeAlongPath', 'speedAlongPath');

%% Saving relevant information as text files

pC=array2table(pathCoordinates,'VariableNames',{'Long' 'Lat'}); 
writetable(pC,'pathCoordinates.txt');

sAP=array2table(speedAlongPath','VariableNames',{'knots'}); 
writetable(sAP,'speedAlongPath.txt');

tAP=array2table(timeAlongPath','VariableNames',{'hours'}); 
writetable(tAP,'timeAlongPath.txt');
