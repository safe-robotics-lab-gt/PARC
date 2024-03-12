classdef TrajectoryPWASingleInt3D < TrajectoryPWA
    % Trajectory PWA system for Single Integrator 3D
    %
    % Author: Wonsuhk Jung
    % Created: Jan 18th 2024
    
    properties
        position_indices
        avoidset_indices
        non_position_indices
        non_avoidset_indices
    end
    
    methods
        function obj = TrajectoryPWASingleInt3D(varargin)
            % Define System
            obj@TrajectoryPWA(varargin{:}, 'dynsys_mode', 'sym');

            % param name
            obj.param_names = ["vx_des", "vy_des", "vz_des"];

            % Turtlebot-related dimensions
            obj.position_indices = 1:3;

            % Dimension for z-agnostic avoid set using convex hull
            obj.avoidset_indices = 1:6;
            
            % Automated from here
            obj.non_position_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.position_indices);
            obj.non_avoidset_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.avoidset_indices);
            
        end
    end
end
