classdef GridEnvironment < handle
    properties
        N
        M
        grid              % NxM matrix: 0 = free, 1 = obstacle
        start             % [row, col]
        goal              % [row, col]
        figureHandle
        axisHandle
        guaranteedPath    % Logical matrix indicating the guaranteed path
    end
    
    methods
        % Constructor
        function obj = GridEnvironment(params)
            obj.N = params.N;
            obj.M = params.M;
            obj.start = params.start;
            obj.goal = params.goal;
            
            % Initialize grid with all free cells
            obj.grid = zeros(obj.N, obj.M);
            
            % Generate a guaranteed path
            obj.guaranteedPath = obj.generatePath(params.pathType);
            
            % Place obstacles randomly, avoiding the guaranteed path
            obj.placeObstacles(params.obstacleDensity);
            
            % Ensure start and goal are free
            obj.grid(obj.start(1), obj.start(2)) = 0;
            obj.grid(obj.goal(1), obj.goal(2)) = 0;
            
            % Initialize figure
            obj.figureHandle = figure;
            hold on;
            obj.axisHandle = gca;
            colormap([1 1 1; 0 0 0]);  % white for free, black for obstacle
            imagesc(obj.grid);
            axis equal tight;
            set(gca,'YDir','normal');
            title('Environment');
            
            % Mark start and goal
            rectangle('Position',[obj.start(2)-0.5,obj.start(1)-0.5,1,1],...
                'FaceColor','g','EdgeColor','k','LineWidth',2);
            rectangle('Position',[obj.goal(2)-0.5,obj.goal(1)-0.5,1,1],...
                'FaceColor','r','EdgeColor','k','LineWidth',2);
            drawnow;
        end
        
        % Get valid neighbors
        function neighbors = getNeighbors(obj, cellCoord)
            % Returns valid neighbors (up, down, left, right)
            % that are within bounds and not obstacles.
            
            r = cellCoord(1); % Current row
            c = cellCoord(2); % Current column
            
            % Define potential moves: [row_offset, col_offset]
            moves = [ -1,  0;  % Up
                       1,  0;  % Down
                       0, -1;  % Left
                       0,  1]; % Right
                   
            % Initialize neighbors list
            neighbors = [];
            
            for i = 1:size(moves, 1)
                nr = r + moves(i, 1); % New row
                nc = c + moves(i, 2); % New column
                
                % Check if the new position is within bounds
                if nr > 0 && nr <= obj.N && nc > 0 && nc <= obj.M
                    % Check if the position is not an obstacle
                    if obj.grid(nr, nc) == 0
                        neighbors = [neighbors; nr, nc]; %#ok<AGROW>
                    end
                end
            end
        end
        
        % Plot the environment
        function plot(obj, path, info, initial, visitedCells)
            if nargin < 5
                visitedCells = [];
            end
            if initial
                return; % Initial plot already done in constructor
            end
            
            % Optionally, plot visited cells
            if ~isempty(visitedCells)
                for k = 1:size(visitedCells,1)
                    r = visitedCells(k,1);
                    c = visitedCells(k,2);
                    if ~isequal([r, c], obj.start) && ~isequal([r, c], obj.goal)
                        rectangle('Position',[c-0.5,r-0.5,1,1],...
                            'FaceColor',[0.8 0.8 0.8],'EdgeColor','none');
                    end
                end
            end
            
            % Optionally, plot the path
            if ~isempty(path)
                for i=1:size(path,1)
                    r = path(i,1);
                    c = path(i,2);
                    rectangle('Position',[c-0.5,r-0.5,1,1],...
                        'FaceColor','b','EdgeColor','k');
                end
            end
        end
        
    end
    
    methods (Access = private)
        function pathLogical = generatePath(obj, pathType)
            % Generates a guaranteed path from start to goal
            % pathType can be 'randomWalk' or 'vectorField'
            
            switch lower(pathType)
                case 'randomwalk'
                    path = obj.randomWalkPath();
                case 'vectorfield'
                    path = obj.vectorFieldPath();
                otherwise
                    error('Unknown pathType. Use ''randomWalk'' or ''vectorField''.');
            end
            
            % Create a logical matrix for guaranteedPath
            pathLogical = false(obj.N, obj.M);
            for i = 1:size(path,1)
                r = path(i,1);
                c = path(i,2);
                pathLogical(r, c) = true; % Mark as part of the path
                obj.grid(r, c) = 0; % Ensure path cells are free
            end
            obj.guaranteedPath = pathLogical;
        end
        
        function path = randomWalkPath(obj)
            % Generates a path using a biased random walk towards the goal
            current = obj.start;
            path = current;
            maxSteps = obj.N * obj.M; % Prevent infinite loops
            
            while ~isequal(current, obj.goal) && size(path,1) < maxSteps
                neighbors = obj.getNeighbors(current);
                if isempty(neighbors)
                    error('Failed to generate a path using random walk.');
                end
                
                % Bias towards the goal by preferring neighbors closer to goal
                distances = sum(abs(neighbors - obj.goal), 2);
                minDist = min(distances);
                bestNeighbors = neighbors(distances == minDist, :);
                
                % Choose randomly among the best neighbors
                idx = randi(size(bestNeighbors,1));
                next = bestNeighbors(idx, :);
                
                % Avoid loops
                if any(all(bsxfun(@eq, path, next), 2))
                    % If looping, choose a different neighbor
                    if size(neighbors,1) > 1
                        otherIdx = setdiff(1:size(neighbors,1), find(all(bsxfun(@eq, neighbors, next),2)));
                        if ~isempty(otherIdx)
                            idx = otherIdx(randi(length(otherIdx)));
                            next = neighbors(idx, :);
                        end
                    end
                end
                
                path = [path; next]; %#ok<AGROW>
                current = next;
            end
            
            if ~isequal(current, obj.goal)
                error('Failed to generate a path to the goal using random walk.');
            end
        end
        
        function path = vectorFieldPath(obj)
            % Generates a path using a vector field approach (greedy towards goal)
            current = obj.start;
            path = current;
            maxSteps = obj.N * obj.M; % Prevent infinite loops
            
            while ~isequal(current, obj.goal) && size(path,1) < maxSteps
                neighbors = obj.getNeighbors(current);
                if isempty(neighbors)
                    error('Failed to generate a path using vector field.');
                end
                
                % Compute direction vectors towards goal
                vectors = obj.goal - neighbors;
                norms = sum(abs(vectors), 2); % Manhattan distance
                [minDist, ~] = min(norms);
                bestNeighbors = neighbors(norms == minDist, :);
                
                % Choose randomly among the best neighbors
                idx = randi(size(bestNeighbors,1));
                next = bestNeighbors(idx, :);
                
                % Avoid loops
                if any(all(bsxfun(@eq, path, next), 2))
                    % If looping, choose a different neighbor
                    if size(neighbors,1) > 1
                        otherIdx = setdiff(1:size(neighbors,1), find(all(bsxfun(@eq, neighbors, next),2)));
                        if ~isempty(otherIdx)
                            idx = otherIdx(randi(length(otherIdx)));
                            next = neighbors(idx, :);
                        end
                    end
                end
                
                path = [path; next]; %#ok<AGROW>
                current = next;
            end
            
            if ~isequal(current, obj.goal)
                error('Failed to generate a path to the goal using vector field.');
            end
        end
        
        function placeObstacles(obj, obstacleDensity)
            % Randomly places obstacles in the grid, avoiding the guaranteed path
            if obstacleDensity < 0 || obstacleDensity > 1
                error('obstacleDensity must be between 0 and 1.');
            end
            
            % Calculate total free cells excluding the path
            totalCells = obj.N * obj.M;
            pathCells = sum(obj.guaranteedPath(:));
            freeCells = totalCells - pathCells;
            
            % Determine number of obstacles to place
            numObstacles = round(obstacleDensity * totalCells);
            
            % Ensure obstacles do not occupy the path
            maxObstacles = freeCells; % All free cells can be obstacles
            if numObstacles > maxObstacles
                warning('Too many obstacles requested. Reducing to maximum possible.');
                numObstacles = maxObstacles;
            end
            
            % Get linear indices of free cells not on the path
            freeIndices = find(obj.grid == 0 & ~obj.guaranteedPath);
            
            % Check if freeIndices is empty
            if isempty(freeIndices) && numObstacles > 0
                error('No available cells to place obstacles.');
            end
            
            % Randomly select obstacle positions
            if numObstacles > 0
                obstacleIndices = randsample(freeIndices, numObstacles);
                % Place obstacles
                obj.grid(obstacleIndices) = 1;
            end
        end
    end
end
