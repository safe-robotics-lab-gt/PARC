classdef TrajectoryPWAQuad2dBad < TrajectoryPWA
    %TRAJECTORYPWATURTLESYM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position_indices
        avoidset_indices

        non_position_indices
        non_avoidset_indices

        drone
    end
    
    methods
        function obj = TrajectoryPWAQuad2dBad(varargin)
            % Define System
            obj@TrajectoryPWA(varargin{:}, 'dynsys_mode', 'sym');

            % quad2d-related dimensions
            obj.position_indices = 1:2;

            % Dimension for z-agnostic avoid set using convex hull
            % TODO: can we automatically parse this from the traj dyns?
            % all states that follows y_t+1 = (1)y_t + Ci*k_t + Cj*Z
            % k is at the end
            obj.avoidset_indices = [1, 2, 3, 7, 8];

            obj.param_names = ["F1", "F2"];

            % Automated from here
            obj.non_position_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.position_indices);
            obj.non_avoidset_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.avoidset_indices);
            
        end
    end
end

