classdef TrajectoryPWATurtle < TrajectoryPWA
    % Trajectory PWA system for Turtlebot
    
    properties
        position_indices
    end
    
    methods
        function obj = TrajectoryPWATurtle(varargin)
            % Define System
            obj@TrajectoryPWA(varargin{:}, 'dynsys_mode', 'built-in');
            obj.system = obj.define_system();
            obj.position_indices = 1:2;
        end
    end
end

