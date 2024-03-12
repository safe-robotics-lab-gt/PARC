classdef ctrlLazyLQR < low_level_controller
    % LazyLQR implementation
    
    properties
        q_x
        q_y
        q_z
    end
    
    methods
        function obj = ctrlLazyLQR(varargin)
            % FasTrack's 10D-3D performance controller implementation

            kwargs = parse_function_args(varargin{:});

            obj.q_x = 0.5;
            if isfield(kwargs, "q_x")
                obj.q_x = kwargs.q_x;
            end

            obj.q_y = 0.5;
            if isfield(kwargs, "q_y")
                obj.q_y = kwargs.q_y;
            end

            obj.q_z = 0.01;
            if isfield(kwargs, "q_z")
                obj.q_z = kwargs.q_z;
            end
        end
        
        function u = get_control_inputs(LLC,A,t,z,varargin)
            % FasTrack Controller Implementation
            % Argument
            %   t: time
            %   z: \in R^10 (near-hover state: [p, v, th, w]

            T_ref = varargin{1} ;
            Z_ref = varargin{3} ;

            z_d = match_trajectories(t,T_ref,Z_ref) ;

            error_dim = [A.position_indices, A.velocity_indices];
            e = z(error_dim)-z_d;

            u_x = LLC.lqr_q2d(e([1, 4]), LLC.q_x);
            u_y = LLC.lqr_q2d(e([2, 5]), LLC.q_y);
            u_z = LLC.lqr_q2d(e([3, 6]), LLC.q_z);

            u = [u_x; u_y; u_z];
        end

        function u = lqr_q2d(LLC, e, q)
            K = [1/q, sqrt(2/q)];
            u = -K*e;
        end
    end
end

