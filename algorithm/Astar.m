
% Function to find path, gets start and finish coordinates
function [pathMatrix, pathArray, Gcost] = AStar(search, latitude, longitude, inverseSpeed, whichList, waypoints, drawUpdates, smoothingOn, startTime)
    addpath algorithm/subroutines
    addpath algorithm/classes
    
    
    updateInterval = 10;
    
    if drawUpdates
        cm = colormap;
    end

    pathArray = [];
    
    iceBreakerDistThreshold = 1.5;
    
    speed = 1 ./ inverseSpeed;
    maxSpeed = max(max(speed));
    
    minFcost = 10*sqrt((search.originX - search.destinationX)^2 + (search.originY - search.destinationY)^2) / maxSpeed;
    
    mapWidth = size(whichList, 1);
    mapHeight = size(whichList, 2);

    parentX = sparse(mapWidth,mapHeight);        % 2D array stores x coord of parent for every cell
    parentY = sparse(mapWidth,mapHeight);        % 2D array stores y coord of parent for every cell
    
    
    Gcost = sparse(mapWidth,mapHeight);          % 2D array stores G cost of each cell
    pathLength = 0;                             % Will store pathlength                

    % Create variables
    
    %openListNew = struct('size',1,'sortedIndex',[],'nextIdValue',1,'x',[],'y');
    openListNew = OpenList(mapHeight,mapWidth);
    
    numberOfOpenListItems = 0;
    path = 0;
    
    % Variables used as constants
    walkable = 1;
    onOpenList = 2;
    onClosedList = 3;
    unwalkable = 4;
    found = 1;
    nonexistent = 2;
    
    
    fprintf('------------------------------- \n');
    fprintf('##### Astar Path Planning ##### \n');
    fprintf('------------------------------- \n\n');
    fprintf('Starting the simulation \n');
    fprintf('Using the following options: \n');
    fprintf('Start position: %d, %d \n', search.originX, search.originY);
    fprintf('Finish position: %d, %d \n', search.destinationX, search.destinationY);
    fprintf('Map size: %d \n\n\n', size(whichList,1));
    fprintf('Progress:\n');
    
    % Check if start and finish coordinates are the same
    if ( (search.originX == search.destinationX) && (search.originY == search.destinationY) )
        disp('Start and finish positions are the same');
        return;
    end
    
    % Add starting position to the open list
    openListNew.add(struct('x',search.originX,'y',search.originY),Inf,0);
   
    % Counter used for printing
    i = 1;
    
    % Create an inifinite loop (until goal is reached or defined unreachable
    while (1)
        
               
        % Check if there are any members in the open list
        if (openListNew.size ~= 0)
            
            %% Get next node in open list and close it
            
            % Get the values of the first item in the open list (lowest
            % Fcost) into current node
            currentNode = openListNew.getFirstAndRemove();
            
            % put the current node on the closed list
            whichList(currentNode.x, currentNode.y) = onClosedList;
       
            % remove the current node from the open list
            %[openList, numberOfOpenListItems] = removeCurrentNodeFromOpenList(openList,numberOfOpenListItems);
            
            % Sort the open list
            %openList = sortopenList(openList, Fcost, numberOfOpenListItems);
            %openListNew.sort();
            
            %% Update stuff
            
            if (mod(i,100)==0)
                % Print out current position being analysed
                fprintf('%d. Current position being analysed: %d, %d \n', i, currentNode.x, currentNode.y);
                fprintf('   Size of open list: %d\n', openListNew.size);
                fprintf('   Elapsed time: %.2f seconds\n',toc(startTime));
            end
                       
            % Draw current position in blue (unless it's the start or
            % target cell.
            
            if (drawUpdates && ~((currentNode.x == search.originX) && (currentNode.y == search.originY) || (currentNode.x == search.destinationX) && (currentNode.y == search.destinationY)) )
               plotPoint([currentNode.x, currentNode.y], 'b');
            end
            
            % Update drawing
            if (mod(i,updateInterval)==0)
                drawnow
            end
            
            % Increment counter
            i = i + 1;
            
            %% Check the neighbours
          
            %for j=1:56
            for j=1:56
                neighbor = getNextNeighbor(j, currentNode);
                
                % Check if it is within bounds of a map
                if ( (neighbor.x > 0) && (neighbor.y > 0) && (neighbor.x <= mapWidth) && (neighbor.y <= mapHeight) )
                    % Check if not on closed list
                    if (whichList(neighbor.x,neighbor.y) ~= onClosedList)
                        % Check if it is possible to navigate directly
                        % to this neighbor cell from the parent cell
                        if isNavigable(neighbor.x, neighbor.y, currentNode.x, currentNode.y, whichList)

                            % If not on the open list, add it
                            if (whichList(neighbor.x,neighbor.y) ~= onOpenList)

                                % Calculate its G cost
                                addedGCost = calculateGCost3(currentNode.x, currentNode.y,neighbor.x,neighbor.y,j,inverseSpeed, waypoints, maxSpeed,iceBreakerDistThreshold);

                                % Update Gcost map
                                Gcost(neighbor.x,neighbor.y) = Gcost(currentNode.x,currentNode.y) + addedGCost;

                                % Get H and F costs and parent
                                tmpHcost = 10*sqrt((neighbor.x- search.destinationX)^2 + (neighbor.y- search.destinationY)^2) / maxSpeed; %Euclidean distance
                                tmpFcost = Gcost(neighbor.x,neighbor.y) + tmpHcost;
                                
                                % Add neighbor to open list
                                openListNew.add(neighbor,tmpFcost,tmpHcost);
                                % Change value of current node in whichList
                                % to 'onOpenList'
                                whichList(neighbor.x,neighbor.y) = onOpenList;

                                parentX(neighbor.x,neighbor.y) = currentNode.x;
                                parentY(neighbor.x,neighbor.y) = currentNode.y;

                                % Draw current position, shade of
                                % yellow/orange indicates Fcost
                                

                                if (drawUpdates && ~((neighbor.x== search.originX) && (neighbor.y== search.originY) || (neighbor.x== search.destinationX) && (neighbor.y== search.destinationY)) )
                                    colorIndex = getColorIndex(tmpFcost, minFcost);
                                    plotPoint([neighbor.x, neighbor.y], cm(colorIndex,:));
                                end
                                
                            else % i.e. whichList(neighbor.x,neighbor.y) == onOpenList
                                
                                % Calculate its G cost
                                addedGCost = calculateGCost3(currentNode.x, currentNode.y, neighbor.x, neighbor.y, j , inverseSpeed, waypoints, maxSpeed, iceBreakerDistThreshold);
                                tempGcost = Gcost(currentNode.x,currentNode.y) + addedGCost;

                                % If this path is shorter, change Gcost, Fcost and the parent cell
                                if (tempGcost < Gcost(neighbor.x,neighbor.y)),
                                    parentX(neighbor.x,neighbor.y) = currentNode.x;
                                    parentY(neighbor.x,neighbor.y) = currentNode.y;
                                    Gcost(neighbor.x,neighbor.y) = tempGcost;

                                    % Changing G cost also changes F cost, sothe open list has to be updated
                                    % Change F cost
                                    openListNew.updateByCoordinates(neighbor,tempGcost);

                                end   % updating GCost 

                            end % if-else statement checking whether neighbor is on open list    

                        end % if statement checking if neighbor cell is navigable
                    end % if statement checking if neighbor is on closed list
                end % if statement checking that neighbor is on the map
            end %End of loop through the neighbors
                                
        % If open list is empty     
        else
            path = nonexistent;
            % Print out failure
            fprintf('%d. Path to the target could not be found \n', i);
            i = i + 1;
            break;
        end
        % If target is added to open list, path has been found
        if (whichList(search.destinationX,search.destinationY) == onOpenList)
            path = found;
            % Print out success
            fprintf('%d. Path to the target found! \n', i);
            i = i + 1;
            break;
        end
        
    end
    
    % If path was found
    if (path == found)
        
        % Initialize data structures
        pathMatrix = zeros(mapHeight,mapWidth);
        pathLength = 0;
        
        % Backtrack the path using parents
        pathX = search.destinationX;
        pathY = search.destinationY;
        % Print out backtracking
        fprintf('%d. Backtracing to find the shortest route \n', i);
        i = i + 1;
        
        % Pre-allocate pathArray to reasonable maximum size
        pathArray = zeros(mapWidth*mapHeight,2);
        
        % Loop until starting position is reached
        while(1)
            % Lookup parent of current cell
            tempx = parentX(pathX,pathY);
            pathY = parentY(pathX,pathY);
            pathX = tempx;
            
            % Increment the path length
            pathLength = pathLength + 1;
            
            pathMatrix(pathY,pathX) = 1;
            
            pathArray(pathLength,1) = pathX;
            pathArray(pathLength,2) = pathY;

            % Draw return path in magenta
            if (drawUpdates && ~((pathX == search.originX) && (pathY == search.originY) || (pathX == search.destinationX) && (pathY == search.destinationY)) )
                plotPoint([pathX, pathY], 'c');
            end
            
            
            % If starting position reached, break out of the loop
            if ( (pathX == search.originX) && (pathY == search.originY) )
                break;
            end
        end
        
        % Print out result
        fprintf('%d. Shortest route is shown in cyan. Total length: %d steps \n', i, pathLength);
        
    end
    
    pathArray = [search.destinationX search.destinationY; pathArray(1:pathLength,:)]; 
    
    
    if (smoothingOn)
       performSmoothing(whichList, pathArray); 
    end

   %pathMatrix = flipud(pathMatrix);
end


function colorIndex = getColorIndex(fCost, minFcost)
  colorIndex = round((minFcost / fCost )^3*63)+1;
end
