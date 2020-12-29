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
end
function cost = euclidean_heuristic(start_pos, goal_pos)
    cost = sqrt(sum((double(start_pos) - double(goal_pos)).^2));
end
% astar = 1;
path = [];
num_expanded = 0;

x_new = start;
x_goal = goal;

start = points_to_idx(map, start);
goal = points_to_idx(map, goal);

epsilon = 0.9 ;

% initialize map
hold on
% map = init_map(x_new, x_goal);
map.occ_map(start(1),start(2),start(3)) = 0;
map.occ_map(goal(1),goal(2),goal(3)) = 0;
% imagesc(map)
% set(gca,'YDir','normal')

%% RRT ALGORITHM
% initialize graph tree
node_index = 1;
source_node = [node_index];
target_node = [];
nodes_x(node_index) = x_new(1);
nodes_y(node_index) = x_new(2);
nodes_z(node_index) = x_new(3);
rrt_graph = graph(source_node,target_node);
rrt_plot = plot(rrt_graph, 'r','XData', nodes_y, 'YData', nodes_x, 'ZData', nodes_z,'NodeLabel',{});

disp('Press any key to start the RRT algorithm (also close the plot that poped up just before this)')
pause
iterations = 1;
% check stopping condition (goal reached)
while (any(x_new ~= x_goal))
    iterations = iterations + 1;
    
    % select direction state
    x_rand = select_state(x_goal,epsilon,1);
%     disp(x_rand)
    % select nearest neighbor to this current random state ('seuclidean', 'minkowski', or 'mahalanobis')
    for node = 1:node_index
%         disp(node)
%         disp(nodes_x(node))
%         disp(nodes_y(node))
%         disp(nodes_z(node))
        neighbors(node) = pdist([nodes_x(node),nodes_y(node),nodes_z(node);x_rand(1),x_rand(2), x_rand(3)],'euclidean');
    end
    [dist, nearest_node] = min(neighbors);
    
    % state of the nearest neighbor
    x_near = [nodes_x(nearest_node), nodes_y(nearest_node), nodes_z(nearest_node)];

    % move towards x_rand position
    x_new = x_near + move(x_near,x_rand);

    % check if position is occupied
    mid = points_to_idx(map, x_new);
    if map.occ_map(mid(1), mid(2), mid(3)) ~= 1
        % check if the node already exists
        exists_node = false;
        for i=1:node_index
            if x_new(1) == nodes_x(node) && x_new(2) == nodes_y(node) && x_new(3) == nodes_z(node)
               exists_node = true;
               break
            end
        end

        if exists_node == false
            % add current state as a node to the graph tree
            node_index = node_index + 1;
            rrt_graph = addnode(rrt_graph,1);
            rrt_graph = addedge(rrt_graph,nearest_node,node_index);
            nodes_x(node_index) = x_new(1);
            nodes_y(node_index) = x_new(2);
            nodes_z(node_index) = x_new(3);
        end
    end
    
    delete(rrt_plot)
    rrt_plot = plot(rrt_graph, 'r','XData', nodes_y, 'YData', nodes_x, 'ZData', nodes_z, 'NodeLabel',{}, 'LineWidth', 2, 'MarkerSize', 7);
    grid on
    pbaspect([1 1 1])
%     xlim([1 50])
%     ylim([1 50])
    pause(0.1)
end
hold off

% Use A* to retrieve the shortest path
spath = shortestpath(rrt_graph,1,node_index);
highlight(rrt_plot,spath,'NodeColor','k','EdgeColor','k');
num_expanded = length(spath)
path = [];
for node = 1:length(spath)
    curr_p = [nodes_x(spath(node)), nodes_y(spath(node)), nodes_z(spath(node))];
    path = [path; curr_p];
end
disp('Press any key to start the simulation (Close the plot on the screen)')
pause
end

%% AUXILIARY FUNCTIONS

function x = select_state(x_goal,epsilon,dist)
    if rand<epsilon
        if dist == 1
            % from a uniform distribution
            x = [randi([0,10]), randi([0,20]), randi([0,6])];
        elseif dist == 2
            x(1) = random('Normal',25,7.5);
            x(2) = random('Normal',25,7.5);
            x(3) = random('Normal',25,7.5);
            for i=1:3
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        elseif dist == 3
            x(1) = random('Rayleigh',x_goal(1));
            x(2) = random('Rayleigh',x_goal(2));
            x(3) = random('Rayleigh',x_goal(3));
            for i=1:3
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        end
    else
        x = x_goal;
    end
end

function angle = find_orientation(source,target)
    target(1) = target(1)-source(1);
    target(2) = target(2)-source(2);
    angle = atan2(target(1),target(2));
    if angle < 0
        angle = angle + 2*pi;
    end
end

function delta = move(source,target)
    angle = find_orientation(source,target);
    delta(1) = sin(angle);
    delta(2) = cos(angle);
    if source(3) ~= target(3)
        if source(3) > target(3)
            delta(3) = -2;
        else
            delta(3) = 2;
        end
    else
        delta(3) = 0;
    end
    
    for i = 1:2
        if 0 < delta(i) && delta(i) < 0.3535
            delta(i) = 0;
        elseif 0.3535 <= delta(i) && delta(i) < 1
            delta(i) = 1;
        elseif -0.3535 < delta(i) && delta(i) < 0
            delta(i) = 0;
        elseif -1 < delta(i) && delta(i) <= -0.3535
            delta(i) = -1;
        end
    end
end