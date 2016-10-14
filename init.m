function [search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime, speed] = init()
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the necesary data structures and run-time
% parameters needed to execute the A* algorithm
% 
% Version History
% (ver. #)   (date)        (author of new version)    (see notes)
%   v0.1      24.2.2016      R. Guinness                none
%   v0.2      24.05.2016     J. Montewka 
%   v0.3      26.08.2016     J. Montewka              attempts to make the standalone version without plotting 
%   v0.4      08.09.2016     J. Montewka              input output as text files
%   v0.5      14.10.2016                              drawing options
%                                                     cleared, code made into blocks


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

%hold on
startTime = tic;

%% Set-up path
addpath environment environment/penninsulas
addpath ships
addpath utilities
addpath algorithm
addpath geographiclib_toolbox

%% Define data structures

% Define search structure
search = struct('originX',0,'originY',0,'destinationX',0,'destinationY',0);
HELMI=struct('originX',0,'originY',0,'destinationX',0,'destinationY',0);

%% Define constants
global UNAVIGABLE; UNAVIGABLE = 4;
global CONTINENT; CONTINENT= 5;

%% Run-time parameters
smoothingOn = false;
drawAll=false;

% if drawAll set to true, above options are all set to true
if drawAll
    drawInitial = true; % don't modify
    drawUpdates = true; % don't modify
    drawResults = true; % don't modify
    hold on
else
    drawInitial = false; % don't modify
    drawUpdates = false; % don't modify
    drawResults = false; % don't modify
end

%drawInitial = true; %drawing initial set up for the algorithm
%drawUpdates = false; %updating figure while the algorithm is running
%drawResults = true; %drawing the reduced path obtained form A*


useSaved = false;

%% Define search parameters and IB waypoints

% Define arrival, departure positions and the threshold for the probability for a ship being stuck from external .txt file
departureInput=readtable('INdepartureCoordinates');
search.originLat = table2array(departureInput(1,1));       % Lat coordinate
search.originLong = table2array(departureInput(1,2));      % Long coordinate
clearvars departureInput

arrivalInput=readtable('INarrivalCoordinates');
search.destinationLat = table2array(arrivalInput(1,1));   % Lat coordinate
search.destinationLong = table2array(arrivalInput(1,2));  % Long coordinate
clearvars arrivalInput

stuckInput=readtable('INstuckThreshold');
stuckThreshold=table2array(stuckInput(1,1));

% Define ice breaker waypoints
waypointsLatLong = readtable('INwaypointsIB');
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

%% Set up continent data
% Get "continent" indices from paths
indexContinents = find(continentMask'==1);

% Assign these as unnavigable in whichList
whichList(indexContinents) = CONTINENT;
%% Calculate ice breaker waypoints in [X, Y]
numWaypoints = size(waypointsLatLong,1);
waypoints = zeros(numWaypoints,2);
for i=1:numWaypoints
    [X, Y] = calcXY(latitude,longitude,waypointsLatLong(i,1),waypointsLatLong(i,2));
    waypoints(i,:) = [X, Y];
end

%% Plot sea environment - this section is commented for Matlab standalone version
if drawInitial
    
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

    % Plot origin and destination points - this section is commented for Matlab standalone version 
    fprintf('Plotting start and finish points...')
    plotPoint([search.originX, search.originY],'r');
    plotPoint([search.destinationX, search.destinationY], 'r');
    fprintf('done.\n')

    % Plot ice breaker waypoints
    fprintf('Plotting ice breaker waypoints...')
    scatter(waypoints(:,1),waypoints(:,2),'b*');
    fprintf('done.\n')

end

timeForSetup = toc;
fprintf('Time for set-up was %.2f seconds.\n',timeForSetup);

end

