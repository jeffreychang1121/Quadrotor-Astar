function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
    
    % effort ratio
    ratio = 0.0;
else
    astar = true;
    
    % effort ratio
    ratio = 0.5;
    
end

start_xyz = start;
goal_xyz = goal;

% start point
start_sub = map.xyzToSub(start_xyz);
start_index = map.xyzToInd(start_xyz);

% goal point
goal_sub = map.xyzToSub(goal_xyz);
goal_index = map.xyzToInd(goal_xyz);

% current point
cur_xyz = start_xyz;
cur_sub = start_sub;
cur_index = start_index;

% obstacles check
% 1: obstacle / 0: clear
obstacle = map.occgrid();

% dimension & size of map
dim = ndims(map.occgrid);

if dim == 3
    [x_len, y_len, z_len] = size(map.occgrid);
else
    [x_len, y_len] = size(map.occgrid);
    z_len = 1;
end

number = x_len * y_len * z_len;
% index array & coordinate
index_array = 1:number;
index_cord = reshape(index_array,[x_len,y_len,z_len]);

% index to XYZ
xyz_cord = map.indToXYZ(index_array');

% index to sub
[sub_x, sub_y, sub_z] = ind2sub([x_len,y_len,z_len], index_array);

% path found
path_found = false;

% chosen set (open)
% 0 : unchosen , 1 : chosen
chosen = zeros(1,number);
chosen(cur_index) = 1;

% visited set (closed)
% 0 : unvisited , 1 : visited
visited = zeros(1,number);
visited(cur_index) = 1;

% cost set (open index)
cost = [];
cur_cost = 0;

% dijkstra cost set
dijk_cost = zeros(1,number);

% total cost set (min)
total_cost = zeros(1,number);

% parent set
parent = zeros(1,number);

while path_found == false
    
    
    % walk through all possibilities
    candidate = [cur_sub(1),   cur_sub(2)+1, cur_sub(3)   ;...
                 cur_sub(1)+1, cur_sub(2),   cur_sub(3)   ;...
                 cur_sub(1),   cur_sub(2),   cur_sub(3)+1 ;...
                 cur_sub(1),   cur_sub(2)-1, cur_sub(3)   ;...
                 cur_sub(1)-1, cur_sub(2),   cur_sub(3)   ;...
                 cur_sub(1),   cur_sub(2),   cur_sub(3)-1 ;...
                 cur_sub(1)+1, cur_sub(2)+1, cur_sub(3)+1 ;...
                 cur_sub(1)+1, cur_sub(2)+1, cur_sub(3)-1 ;...
                 cur_sub(1)+1, cur_sub(2)-1, cur_sub(3)+1 ;...
                 cur_sub(1)+1, cur_sub(2)-1, cur_sub(3)-1 ;...
                 cur_sub(1)-1, cur_sub(2)+1, cur_sub(3)+1 ;...
                 cur_sub(1)-1, cur_sub(2)-1, cur_sub(3)+1 ;...
                 cur_sub(1)-1, cur_sub(2)-1, cur_sub(3)-1 ;...
                 cur_sub(1)-1, cur_sub(2)-1, cur_sub(3)-1 ;...
                 cur_sub(1)+1, cur_sub(2)+1, cur_sub(3)   ;...
                 cur_sub(1)+1, cur_sub(2)-1, cur_sub(3)   ;...
                 cur_sub(1)-1, cur_sub(2)+1, cur_sub(3)+1 ;...
                 cur_sub(1)-1, cur_sub(2)-1, cur_sub(3)   ;...
                 cur_sub(1)+1, cur_sub(2),   cur_sub(3)+1 ;...
                 cur_sub(1)+1, cur_sub(2),   cur_sub(3)-1 ;
                 cur_sub(1)-1, cur_sub(2),   cur_sub(3)+1 ;...
                 cur_sub(1)-1, cur_sub(2),   cur_sub(3)-1 ;...
                 cur_sub(1),   cur_sub(2)+1, cur_sub(3)+1 ;...
                 cur_sub(1),   cur_sub(2)+1, cur_sub(3)-1 ;...
                 cur_sub(1),   cur_sub(2)-1, cur_sub(3)+1 ;...
                 cur_sub(1),   cur_sub(2)-1, cur_sub(3)-1];
    
    % check all possibilities
    for i = 1:26
        
        % candidate within boundary
        if(     candidate(i,1) <= x_len && candidate(i,1) >= 1 ...
             && candidate(i,2) <= y_len && candidate(i,2) >= 1 ...
             && candidate(i,3) <= z_len && candidate(i,3) >= 1)
            
            % get index if within boundary
            temp_index = index_cord(candidate(i,1),candidate(i,2),candidate(i,3));
            
            % check if chosen as candidate before (prevent detecting collision again)
            if chosen(temp_index) == 0
               
                chosen(temp_index) = 1;                
                
                
                % candidate doesn't collide with obstacle
                if obstacle(temp_index) ~= 1
                    
                    % get xyz coordinate to check cost
                    temp_xyz = xyz_cord(temp_index,:);
                    
                    % path cost
                    C = consideration(cur_xyz,temp_xyz, cur_cost);
                    % dijkstra current cost
                    dijk_cost(temp_index) = C;
                    % heuristic cost
                    H = heuristic(goal_xyz, temp_xyz);
                    % total cost
                    total_cost(temp_index) = ratio * H + (1-ratio) * C;
                    % store temporary cost
                    cost = [cost, temp_index];
                    
                    % set current point's parent
                    parent(temp_index) = cur_index;
                end
                
            % if chosen before check if the cost was min
            else
                
                temp_xyz = xyz_cord(temp_index,:);
                C = consideration(cur_xyz, temp_xyz, cur_cost);
                H = heuristic(goal_xyz, temp_xyz);
                
                % store temporary final cost
                temp_cost = ratio * H + (1-ratio) * C;
                
                
                % if final cost isn't min replace with temporary final cost
                if temp_cost < total_cost(temp_index)
                    if visited(temp_index) ~= 1
                        if C < dijk_cost(temp_index)
                            dijk_cost(temp_index) = C;
                        end
                    end
                    total_cost(temp_index) = temp_cost;
                    
                    parent(temp_index) = cur_index;
                    
                end
                
                
            end
        end
    end

    % find min cost
    [~, index] = min(total_cost(cost));
    
    % get the correct min index
    cur_index = cost(index);
    cur_cost = dijk_cost(cur_index);
    cost(index) = [];
    
    % set current point to visited
    visited(cur_index) = 1;
    
    % check if reached goal point
    cur_xyz = xyz_cord(cur_index,:);
    cur_sub = [sub_x(cur_index), sub_y(cur_index), sub_z(cur_index)];
    
    if cur_index == goal_index
        path_found = true;
    end
end

% reconstruct path
old_path = [goal_xyz];
old_index = [cur_index];
leaf = cur_index;

while leaf ~= 0
    cur_index = parent(leaf);
    leaf = cur_index;
    if leaf == 0
        old_path(1,:) = start_xyz;
        old_index(1,:) = start_index;
        break;
    end
    cur_xyz = xyz_cord(leaf,:);
    old_path = [cur_xyz; old_path];
    old_index = [cur_index; old_index];
end

num_expanded = nnz(chosen);

% choose less points as new path
path = [start_xyz];
pre_pos = start_xyz;
pre_seg = 1;
for i = 2:size(old_path,1)-1
    cur_pos = old_path(i,:);
    cur_seg = i;
    px = linspace(pre_pos(1),cur_pos(1),cur_seg-pre_seg+1);
    py = linspace(pre_pos(2),cur_pos(2),cur_seg-pre_seg+1);
    pz = linspace(pre_pos(3),cur_pos(3),cur_seg-pre_seg+1);
    test_points = [px',py',pz'];
    % if no collision between two points move to the next one
    if all(map.collide(test_points) == 0)
        continue;
    else
        path = [path;old_path(i-1,:)];
        pre_pos = old_path(i-1,:);
        pre_seg = cur_seg-1;
    end
end

% new path
path = [path;goal_xyz];

% smooth path
for i = 3 : size(path,1)
    for j = 0.1 : 0.1 : 0.5
        cut = ((path(i-2,:) - path(i-1,:)) / norm(path(i-2,:) - path(i-1,:)) + (path(i,:) - path(i - 1,:)) / norm(path(i,:) - path(i-1,:))) / 2;
        if map.collide(path(i-1,:) + j * cut) == 0
            path(i-1,:) = path(i-1,:) + j * cut;
        else
            break
        end
    end
end

end
