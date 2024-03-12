classdef TrajectoryPWADubins < TrajectoryPWA
    %TRAJECTORYPWATURTLESYM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position_indices
        avoidset_indices

        non_position_indices
        non_avoidset_indices
    end
    
    methods
        function obj = TrajectoryPWADubins(varargin)
            % Define System
            obj@TrajectoryPWA(varargin{:}, 'dynsys_mode', 'sym');

            % Dubins-related dimensions
            obj.position_indices = 1:2;

            % Dimension for z-agnostic avoid set using convex hull
            % TODO: can we automatically parse this from the traj dyns?
            obj.avoidset_indices = [1, 2, 4, 5];

            obj.param_names = ["w_des", "v_des"];

            % Automated from here
            obj.non_position_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.position_indices);
            obj.non_avoidset_indices = setdiff(1:(obj.n_state+obj.n_param), ...
                                                  obj.avoidset_indices);
            
        end
    end
end

