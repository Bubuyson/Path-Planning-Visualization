classdef PriorityQueue < handle
    properties (Access = private)
        heap
        priorities
    end
    
    methods
        function obj = PriorityQueue()
            obj.heap = [];
            obj.priorities = [];
        end
        
        function insert(obj, item, priority)
            % Ensure 'item' is a row vector
            if size(item,1) > 1
                item = item';
            end
            obj.heap(end+1, :) = item; %#ok<AGROW>
            obj.priorities(end+1, 1) = priority; %#ok<AGROW>
            obj.bubbleUp(size(obj.heap,1));
        end
        
        function [item, priority] = extractMin(obj)
            if obj.isEmpty()
                error('Priority Queue is empty');
            end
            item = obj.heap(1, :);
            priority = obj.priorities(1);
            if size(obj.heap,1) == 1
                obj.heap = [];
                obj.priorities = [];
            else
                obj.heap(1, :) = obj.heap(end, :);
                obj.priorities(1) = obj.priorities(end);
                obj.heap(end, :) = [];
                obj.priorities(end) = [];
                obj.bubbleDown(1);
            end
        end
        
        function bool = isEmpty(obj)
            bool = isempty(obj.heap);
        end
        
        function updatePriority(obj, item, newPriority)
            idx = find(all(obj.heap == item, 2));
            if ~isempty(idx)
                if newPriority < obj.priorities(idx)
                    obj.priorities(idx) = newPriority;
                    obj.bubbleUp(idx);
                end
            end
        end
    end
    
    methods (Access = private)
        function bubbleUp(obj, idx)
            while idx > 1
                parent = floor(idx / 2);
                if obj.priorities(parent) <= obj.priorities(idx)
                    break;
                end
                % Swap
                [obj.heap(idx, :), obj.heap(parent, :)] = deal(obj.heap(parent, :), obj.heap(idx, :));
                [obj.priorities(idx), obj.priorities(parent)] = deal(obj.priorities(parent), obj.priorities(idx));
                idx = parent;
            end
        end
        
        function bubbleDown(obj, idx)
            n = size(obj.heap,1);
            while true
                left = 2 * idx;
                right = 2 * idx + 1;
                smallest = idx;
                
                if left <= n && obj.priorities(left) < obj.priorities(smallest)
                    smallest = left;
                end
                if right <= n && obj.priorities(right) < obj.priorities(smallest)
                    smallest = right;
                end
                if smallest == idx
                    break;
                end
                % Swap
                [obj.heap(idx, :), obj.heap(smallest, :)] = deal(obj.heap(smallest, :), obj.heap(idx, :));
                [obj.priorities(idx), obj.priorities(smallest)] = deal(obj.priorities(smallest), obj.priorities(idx));
                idx = smallest;
            end
        end
    end
end
