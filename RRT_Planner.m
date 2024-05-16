% Define the grid size and obstacle map
grid_size = 200;
obstacle_map = zeros(grid_size, grid_size);

% Define obstacles (e.g., rectangles, circles, etc.)
obstacle_map(50:70, 50:70) = 1;  % obstacle 1
obstacle_map(100:120, 100:120) = 1;  % obstacle 2
obstacle_map(150:170, 150:170) = 1;  % obstacle 3

% Define start and goal regions
start_region = [10, 10];
goal_region = [180, 180];

% Define RRT parameters
max_iterations = 1000;
step_size = 5;
goal_bias = 0.1;

% Initialize the tree
tree = struct('nodes', [], 'edges', []);

% Add the start node to the tree
tree.nodes = [tree.nodes; start_region];

% RRT loop
for i = 1:max_iterations
    % Sample a random point in the free space
    x_rand = randi(grid_size);
    y_rand = randi(grid_size);
    while obstacle_map(x_rand, y_rand) == 1
        x_rand = randi(grid_size);
        y_rand = randi(grid_size);
    end
    
    % Find the nearest node in the tree
    dist = sqrt((tree.nodes(:, 1) - x_rand).^2 + (tree.nodes(:, 2) - y_rand).^2);
    [min_dist, idx] = min(dist);
    x_near = tree.nodes(idx, 1);
    y_near = tree.nodes(idx, 2);
    
    % Calculate the new node position
    x_new = x_near + step_size * (x_rand - x_near) / min_dist;
    y_new = y_near + step_size * (y_rand - y_near) / min_dist;
    
    % Check if the new node is in the free space and within grid boundaries
    if round(x_new + 0.5) >= 1 && round(x_new + 0.5) <= grid_size && ...
       round(y_new + 0.5) >= 1 && round(y_new + 0.5) <= grid_size && ...
       obstacle_map(round(x_new + 0.5), round(y_new + 0.5)) == 0
        % Add the new node to the tree
        tree.nodes = [tree.nodes; [x_new, y_new]];
        tree.edges = [tree.edges; [idx, size(tree.nodes, 1)]];
        
        % Check if the goal region is reached
        if sqrt((x_new - goal_region(1))^2 + (y_new - goal_region(2))^2) < 5
            break;
        end
    end
    
    % Goal biasing
    if rand < goal_bias
        x_rand = goal_region(1);
        y_rand = goal_region(2);
    end
end

% Plot the obstacles and tree
figure;
imshow(obstacle_map, 'InitialMagnification', 'fit');
hold on;
plot(tree.nodes(:, 1), tree.nodes(:, 2), 'b-');
plot(start_region(1), start_region(2), 'go');
plot(goal_region(1), goal_region(2), 'ro');

% Find the path from the start to the goal
path = [];
idx = size(tree.nodes, 1);
while idx ~= 1
    path = [path; tree.nodes(idx, :)];
    idx = tree.edges(idx - 1, 1);
end
path = [path; start_region];
plot(path(:, 1), path(:, 2), 'r-');

% Check if the path is valid
if isempty(path) || size(path, 1) < 2
    error('Path not found');
end

% Check if the path is in the free space
for i = 1:size(path, 1) - 1
    if obstacle_map(round(path(i, 1)), round(path(i, 2))) == 1
        error('Path intersects obstacle');
    end
end

% Print the path length
fprintf('Path length: %f\n', sum(sqrt((path(1:end - 1, 1) - path(2:end, 1)).^2 + (path(1:end - 1, 2) - path(2:end, 2)).^2)));

% Plot the path
axis equal;
xlabel('X');
ylabel('Y');
title('RRT Planner');