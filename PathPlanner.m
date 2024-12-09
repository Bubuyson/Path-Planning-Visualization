classdef (Abstract) PathPlanner < handle
    methods (Abstract)
        [path, info] = solve(obj, env);
    end
end
