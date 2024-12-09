classdef DijkstraAlgorithm < PathPlanner
    properties
        heuristicWeight = 0.0; % No heuristic
        visualization = true;
        pauseTime = 0.01;
    end
    
    methods
        function obj = DijkstraAlgorithm(params)
            if isfield(params,'heuristicWeight')
                obj.heuristicWeight = params.heuristicWeight;
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

            % Initialize costs
            gCost = inf(env.N, env.M); % Cost from start
            fCost = inf(env.N, env.M); % f = g + h
            gCost(start(1), start(2)) = 0;
            fCost(start(1), start(2)) = obj.heuristic(start,goal);

            % Parent map for path reconstruction
            parent = cell(env.N, env.M);

            % Initialize Priority Queue
            openSet = PriorityQueue();
            openSet.insert(start, fCost(start(1), start(2)));

            % Closed set to keep track of visited nodes
            closedSet = false(env.N, env.M);

            visitedCells = []; % For visualization

            % Initialize path and info before plotting
            path = []; % Initialize as empty
            info = struct(); % Initialize as empty struct

            % Initialize visualization
            if obj.visualization
                env.plot(path, info, true); % Initial plot with empty path and info
                drawnow;
            end

            while ~openSet.isEmpty()
                % Extract node with lowest fCost
                [current, ~] = openSet.extractMin();
                
                % Mark as visited
                closedSet(current(1), current(2)) = true;
                visitedCells(end+1, :) = current; %#ok<AGROW>
                
                % Visualization update
                if obj.visualization
                    env.plot(path, info, false, visitedCells);
                    rectangle('Position',[current(2)-0.5, current(1)-0.5,1,1],...
                        'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');
                    drawnow;
                    pause(obj.pauseTime); % Control visualization speed
                end

                % Check if goal is reached
                if isequal(current, goal)
                    path = obj.reconstructPath(parent, start, goal);
                    info.visitedCells = visitedCells;
                    return;
                end

                % Expand neighbors
                neighbors = env.getNeighbors(current);
                for i=1:size(neighbors,1)
                    nb = neighbors(i,:);
                    tentative_g = gCost(current(1), current(2)) + 1; % Cost of 1 per move
                    if tentative_g < gCost(nb(1), nb(2))
                        parent{nb(1), nb(2)} = current;
                        gCost(nb(1), nb(2)) = tentative_g;
                        h = obj.heuristic(nb, goal);
                        f = tentative_g + obj.heuristicWeight * h;
                        fCost(nb(1), nb(2)) = f;
                        if ~closedSet(nb(1), nb(2))
                            openSet.insert(nb, f);
                        end
                    end
                end
            end

            % If no path found
            path = [];
            info.visitedCells = visitedCells;
        end
    end
    
    methods (Access = private)
        function h = heuristic(obj, node, goal)
            % No heuristic for Dijkstra
            h = 0;
        end
        
        function path = reconstructPath(obj, parent, start, goal)
            path = goal;
            current = goal;
            while ~isequal(current, start)
                current = parent{current(1), current(2)};
                path = [current; path]; %#ok<AGROW>
            end
        end
    end
end
