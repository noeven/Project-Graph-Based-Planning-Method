%% Define a small map

%% Define a small map
map = false(10);

% Add an obstacle
%map (2:9, 6) = true;
%map (1:9,7)=true;
%map (9,8:9)=true;
%map (7,9:10)=true;
%map (5,8:9)=true;
%map (3,9:10)=true;
%map (2,9)=true;
map (1:5, 6) = true;

start_coords = [6, 2];
dest_coords  = [8, 9];

%%
[route, numExpanded] = DijkstraGrid(map, start_coords, dest_coords, true);


function [route,numExpanded] = DijkstraGrid(input_map, start_coords, dest_coords, drawMap)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = drawMap;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows, ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distanceFromStart = Inf(nrows, ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows, ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break
    end
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    
   % ********************************************************************* 
    % YOUR CODE BETWEEN THESE LINES OF STARS
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
       
if i<= nrows && i > 0
     if j+1<=ncols && j>0
        if map(i,j+1)~=2 && map(i,j+1)~=3 && map(i,j+1)~=5 && map(i,j+1)~=6
            map(i,j+1)=4   % for grid which is free, put it in the list
                if map(i,j)~=5    % if the current is not start noede, change it from on list to visited 
                    map(i,j)=3;
                end
            distanceFromStart(i,j+1)=min_dist+1;  %upgdate the distance of new visited node 
            parent(i,j+1)=sub2ind(size(map),i,j);  %record the parent
          %=numExpanded+1; % if sucessfully visited the new node, update the number of expanded
        elseif map(i,j+1)==6   % if the new visited node is the destination, mark the parent as visted node and update the parameters.
        map(i,j)==3;
        parent(i,j+1)=sub2ind(size(map),i,j);
        distanceFromStart(i,j+1)=min_dist+1;
     
           break
              
        end
     end
end
 
  if i<= nrows && i > 0

     if j<=ncols && j-1>0 
            if map(i,j-1)~=2 && map(i,j-1)~=3 && map(i,j-1)~=5 && map(i,j-1)~=6
            map(i,j-1)=4   % for grid which is free, put it in the list
                if map(i,j)~=5
                    map(i,j)=3;
                end
            distanceFromStart(i,j-1)=min_dist+1;  %record the distance
            parent(i,j-1)=sub2ind(size(map),i,j);  %record the parent
                   elseif map(i,j-1)==6
            distanceFromStart(i,j-1)=min_dist+1;  %record the distance
            parent(i,j-1)=sub2ind(size(map),i,j);  %record the parent
                       break
            end
     end
  end

if i+1<=nrows && i > 0

     if j<=ncols && j>0 
            if map(i+1,j)~=2 && map(i+1,j)~=3 && map(i+1,j)~=5 && map(i+1,j)~=6
            map(i+1,j)=4   % for grid which is free, put it in the list
                if map(i,j)~=5
                    map(i,j)=3;
                end
            distanceFromStart(i+1,j)=min_dist+1;  %record the distance
            parent(i+1,j)=sub2ind(size(map),i,j);  %record the parent
                   elseif map(i+1,j)==6
             distanceFromStart(i+1,j)=min_dist+1;  %record the distance
            parent(i+1,j)=sub2ind(size(map),i,j);  %record the parent
           
                      break
            end
     end
end

 if i<= nrows && i-1 > 0

     if j<=ncols && j>0 
            
            if map(i-1,j)~=2 && map(i-1,j)~=3 && map(i-1,j)~=5 && map(i-1,j)~=6
            map(i-1,j)=4   % for grid which is free, put it in the list
                if map(i,j)~=5
                    map(i,j)=3;
                end
            distanceFromStart(i-1,j)=min_dist+1;  %record the distance
            parent(i-1,j)=sub2ind(size(map),i,j);  %record the parent
                   elseif map(i-1,j)==6
            distanceFromStart(i-1,j)=min_dist+1;  %record the distance
            parent(i-1,j)=sub2ind(size(map),i,j);  %record the parent

                       break
            end
     end
 end
 



    
    %*********************************************************************

end
parent
distanceFromStart
map
numExpanded=sum(map(:) == 3)+sum(map(:) == 5)+sum(map(:) == 4);
%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = dest_node;
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
    % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end
