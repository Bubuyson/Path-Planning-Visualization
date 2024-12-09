classdef RRTAlgorithm < PathPlanner
    properties
        maxIterations = 1000;
        stepSize = 1;
        visualization = true;
        pauseTime = 0.01;
    end
    
    methods
        function obj = RRTAlgorithm(params)
            if isfield(params, 'maxIterations')
                obj.maxIterations = params.maxIterations;
            end
            if isfield(params, 'stepSize')
                obj.stepSize = params.stepSize;
            end
            if isfield(params, 'visualization')
                obj.visualization = params.visualization;
            end
            if isfield(params, 'pauseTime')
                obj.pauseTime = params.pauseTime;
            end
        end
        
        function [path, info] = solve(obj, env)
            start = env.start;
            goal = env.goal;
            
            tree.nodes = start;
            tree.parent = 0; % 0 indicates no parent for the root
            
            found = false;
            path = []; % Initialize as empty
            info.iterations = 0;
            info.visitedCells = [];
            
            % Initialize visualization
            if obj.visualization
                env.plot(path, info, true); % Initial plot with empty path and info
                hold on;
                drawnow;
            end
            
            for i = 1:obj.maxIterations
                info.iterations = i;
                % Sample random node
                randNode = [randi(env.N), randi(env.M)];
                if env.grid(randNode(1), randNode(2)) == 1
                    continue; % Skip obstacles
                end
                
                % Find nearest node in the tree
                nearestIdx = obj.nearestNeighbor(tree.nodes, randNode);
                nearestNode = tree.nodes(nearestIdx, :);
                
                % Steer towards random node
                direction = randNode - nearestNode;
                distance = norm(direction, 1); % Manhattan distance
                if distance == 0
                    continue;
                end
                step = sign(direction); % Move one step in the direction
                newNode = nearestNode + step;
                
                % Check if new node is free
                if newNode(1) < 1 || newNode(1) > env.N || newNode(2) < 1 || newNode(2) > env.M
                    continue; % Out of bounds
                end
                if env.grid(newNode(1), newNode(2)) == 1
                    continue; % Obstacle
                end
                
                % Check if node is already in the tree
                if any(all(bsxfun(@eq, tree.nodes, newNode), 2))
                    continue;
                end
                
                % Add new node to the tree
                tree.nodes = [tree.nodes; newNode]; %#ok<AGROW>
                tree.parent = [tree.parent; nearestIdx]; %#ok<AGROW>
                info.visitedCells = [info.visitedCells; newNode]; %#ok<AGROW>
                
                % Visualization update
                if obj.visualization
                    env.plot(path, info, false, info.visitedCells);
                    plot([nearestNode(2), newNode(2)], [nearestNode(1), newNode(1)], 'c-');
                    rectangle('Position',[newNode(2)-0.5, newNode(1)-0.5,1,1],...
                        'FaceColor','y','EdgeColor','none');
                    drawnow;
                    pause(obj.pauseTime);
                end
                
                % Check if goal is reached
                if isequal(newNode, goal)
                    path = obj.reconstructPath(tree.parent, tree.nodes, start, goal);
                    found = true;
                    break;
                end
            end
            
            if ~found
                disp('RRT: Path not found within the maximum iterations.');
            end
            
            info.found = found;
        end
    end
    
    methods (Access = private)
        function idx = nearestNeighbor(obj, nodes, randNode)
            % Find the nearest node in the tree to the random node
            distances = sum(abs(nodes - randNode), 2); % Manhattan distance
            [~, idx] = min(distances);
        end
        
        function path = reconstructPath(obj, parent, nodes, start, goal)
            path = goal;
            currentIdx = find(all(bsxfun(@eq, nodes, goal), 2));
            while currentIdx ~= 0
                parentIdx = parent(currentIdx);
                if parentIdx == 0
                    break;
                end
                currentNode = nodes(parentIdx, :);
                path = [currentNode; path]; %#ok<AGROW>
                currentIdx = parentIdx;
            end
            % Ensure the path starts at the start node
            if ~isequal(path(1,:), start)
                path = [start; path];
            end
        end
    end
end
