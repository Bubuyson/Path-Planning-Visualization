% main.m

clear; close all; clc;

%% Step 1: Define Hyperparameters of the Environment
rng(5)
params_env.N = 20;                % Grid rows
params_env.M = 50;                % Grid columns
params_env.start = [1,1];         % Start cell (row, col)
params_env.goal = [20,30];        % Goal cell (row, col)
params_env.obstacleDensity = 0.1; % 10% of cells are obstacles
params_env.pathType = 'randomWalk'; % 'randomWalk' or 'vectorField'

%% Step 2: Create the environment object
env = GridEnvironment(params_env);

%% Step 3: Select the algorithm
% Choose between 'Astar', 'Dijkstra', 'RRT'
selectedAlgorithm = 'RRT'; % Change as needed: 'Astar', 'Dijkstra', 'RRT'

% Define algorithm parameters
params_algo.visualization = true;
params_algo.pauseTime = 0; % Adjust to control visualization speed

switch selectedAlgorithm
    case 'Astar'
        params_algo.heuristicWeight = 1.0;
        algo = AstarAlgorithm(params_algo);
    case 'Dijkstra'
        algo = DijkstraAlgorithm(params_algo);
    case 'RRT'
        params_algo.maxIterations = 1000;
        params_algo.stepSize = 1;
        algo = RRTAlgorithm(params_algo);
    otherwise
        error('Unknown algorithm selected');
end

%% Step 4: Call the solver method of the algorithm
[path, info] = algo.solve(env);

%% Step 5: Visualize the final path
if ~isempty(path)
    env.plot(path, info, false);
    title([selectedAlgorithm, ' Path Planning Result']);
else
    title([selectedAlgorithm, ' Path Planning Result: No Path Found']);
end
