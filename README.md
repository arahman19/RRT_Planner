# RRT Planner

This project implements a Rapidly-exploring Random Tree (RRT) planner for a point robot navigating in a 2D plane with obstacles. The algorithm is designed to find a path from a specified start region to a goal region within a grid, avoiding obstacles along the way.
## Features

Obstacle Representation: The environment is represented by a 200x200 grid. Obstacles are indicated by 1s in the grid, while free spaces are indicated by 0s.
Tree Expansion: The RRT algorithm grows a tree by randomly sampling points in free space, finding the nearest existing node, and extending towards the sampled point.
Goal Biasing: To expedite convergence, a goal bias is used, where a certain percentage of samples are biased towards the goal region.
Path Finding: The algorithm tracks the path from the start to the goal once the goal region is reached.
Visualization: The program plots the obstacles, the RRT tree, and the resulting path from the start to the goal.

## Code Overview
### Initialize the Environment

The grid size and obstacle map are defined, where obstacles are manually set within the grid:

   ```bash
   grid_size = 200;
   obstacle_map = zeros(grid_size, grid_size);

   obstacle_map(50:70, 50:70) = 1;  % obstacle 1
   obstacle_map(100:120, 100:120) = 1;  % obstacle 2
   obstacle_map(150:170, 150:170) = 1;  % obstacle 3
   ```

### Define Start and Goal Regions

The start and goal regions are specified as coordinates within the grid:

   ```bash
   start_region = [10, 10];
   goal_region = [180, 180];
   ```

### RRT Parameters

Parameters for the RRT algorithm, including maximum iterations, step size, and goal bias, are set:

   ```bash
   max_iterations = 1000;
   step_size = 5;
   goal_bias = 0.1;
   ```

### Tree Initialization

The RRT tree is initialized with the start node:

   ```bash
   tree = struct('nodes', [], 'edges', []);
   tree.nodes = [tree.nodes; start_region];
   ```

### RRT Loop

The main loop of the RRT algorithm samples points, extends the tree, and checks for goal reach:

   ```bash
   for i = 1:max_iterations
       % Sampling, finding nearest node, calculating new node position
       % Adding new node to tree and checking for goal region
       % (See full code for detailed implementation)
   end
   ```

### Visualization

Obstacles, the RRT tree, and the path are plotted:

   ```bash
   figure;
   imshow(obstacle_map, 'InitialMagnification', 'fit');
   hold on;
   plot(tree.nodes(:, 1), tree.nodes(:, 2), 'b-');
   plot(start_region(1), start_region(2), 'go');
   plot(goal_region(1), goal_region(2), 'ro');

   % Path plotting and validation
   % (See full code for detailed implementation)
   ```

### Path Validation

The code includes checks to ensure the path does not intersect any obstacles:

   ```bash
   for i = 1:size(path, 1) - 1
       if obstacle_map(round(path(i, 1)), round(path(i, 2))) == 1
           error('Path intersects obstacle');
       end
   end
   ```

### Path Length

The length of the found path is printed:

   ```bash
   fprintf('Path length: %f\n', sum(sqrt((path(1:end - 1, 1) - path(2:end, 1)).^2 + (path(1:end - 1, 2) - path(2:end, 2)).^2)));
   ```

## Running the Code

To run the RRT planner, simply execute the MATLAB script provided. Ensure you have MATLAB installed and properly configured to visualize the output.
