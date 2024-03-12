classdef TrajectoryPWA
    % Piecewise Affine Trajectory Model
    
    properties
        % Required components
        states_bound % cell{n_state, 1}, cell{i} = [min_state_i, max_state_i]
        params_bound % cell{n_param, 1}, cell{i} = [min_param_i, max_param_i]
        states_params_bound
        affinization_dt
        affinization_N_grid % (n_state + n_param by 1), 0 (affine), n

        % PWA system (MPT object)
        system
        dynsys_mode

        % Misc
        state_indices
        n_state

        param_indices
        param_names
        n_param

        % Tracking
        tracking_error
    end
    
    methods
        function obj = TrajectoryPWA(varargin)
            % Trajectory PWA of Turtlebot
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                'required_key' , ["states_bound", "params_bound", ...
                                  "affinization_dt", "affinization_N_grid"], ...
                'default_key'  , {"dynsys_mode"}, ...
                'default_value', {"sym"});

            obj.states_bound        = kwargs.states_bound;
            obj.params_bound        = kwargs.params_bound;
            obj.affinization_dt     = kwargs.affinization_dt;
            obj.affinization_N_grid = kwargs.affinization_N_grid;
            obj.dynsys_mode         = kwargs.dynsys_mode;

            % Initialize the system
            obj.states_params_bound = {obj.states_bound{:}, obj.params_bound{:}};
            obj.n_state = length(obj.states_bound);
            obj.n_param = length(obj.params_bound);

            obj.state_indices = 1:obj.n_state;
            obj.param_indices = obj.n_state + 1: obj.n_state + obj.n_param;

            switch(obj.dynsys_mode)
                case "sym"
                    % user provide symbolic function and would
                    % automatically get affinized system
                    [x, dx]    = obj.define_system();
                    obj.system = obj.affinize(x, dx);

                case "built-in"
                    % user provide affinized system by hand
                    obj.system = obj.define_system();
                
                otherwise
                    error("Please provide the dynsys_mode between sym and built-in.");
            end
        end

        function bound_polytope = get_bound_polytopes_by_dims(obj, dims)
            bounds = obj.states_params_bound(dims);
            bounds = cat(1, bounds{:});

            n_bounds = size(bounds, 1);
            
            bound_polytope = Polyhedron("A", [eye(n_bounds); -eye(n_bounds)], ...
                                        "b", [bounds(:, 2); -bounds(:, 1)]);
        end
    end
end

