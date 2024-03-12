classdef TrackingErrorData
    % Instance of tracking error function
    % Author: Wonsuhk Jung
    properties
        errors % (num_data, error_dim, length_of_stamps)
        stamps % (1, length_of_stamps)

        error_dim
        num_data % number of tracking error data

        extras
    end
    
    methods
        function obj = TrackingErrorData(varargin)
            %% Abstraction for tracking error data
            %   errors: (num_data, error_dim, length_of_stamps)
            %   stamps: (1, length_of_stamps)
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                'required_key', ["errors", "stamps"]);

            errors = kwargs.errors;
            stamps = kwargs.stamps;

            if size(errors, 2) ~= size(stamps, 2)
                error("The provided [errors] and [stamps] do not have same length.")
            end

            obj.error_dim = size(errors, 1);
            obj.num_data  = 1;
            obj.stamps = stamps;
            obj.errors = reshape(errors, [1, obj.error_dim, length(obj.stamps)]);

            % Extra arguments to reproduce
            obj.extras = struct();
            if isfield(kwargs, "initial_state")
                if size(kwargs.initial_state, 2) == 1
                    kwargs.initial_state = kwargs.initial_state';
                end
                obj.extras.initial_state = kwargs.initial_state;
            end
            if isfield(kwargs, "traj_param")
                if size(kwargs.traj_param, 2) == 1
                    kwargs.initial_state = kwargs.traj_param';
                end
                obj.extras.traj_param = kwargs.traj_param;
            end
        end

        function obj = append(obj, varargin)
            % Append the tracking error data and sort the data according to
            % the augment time order
            %
            % Input
            %   data: TrackingErrorData
            %   dt  : the time difference that we consider the time equal
            %         i.e. if |t1-t2| < dt => t1==t2
            % Author: Wonsuhk Jung
            % Created: Jan 22 2024

            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                'required_key', ["data"], ...
                'default_key',  {"dt"}, ...
                'default_value', {1e-4});

            data2 = kwargs.data;
            tol   = kwargs.dt;

            if obj.error_dim ~= data2.error_dim
                error("You should have same error dimension");
            end

            % sort and merge the error data if |t1-t2| <= tol
            stamps_eval = [obj.stamps, data2.stamps];
            stamps_eval = uniquetol(stamps_eval, tol);

            N_data = obj.num_data + data2.num_data;

            % concat the tracking errors
            errors_eval = zeros(N_data, obj.error_dim, length(stamps_eval));

            for i = 1:obj.error_dim
                for j = 1:obj.num_data
                    errors_eval(j, i, :) = interp1(obj.stamps, ...
                                                   squeeze(obj.errors(j, i, :)), ...
                                                   stamps_eval, 'linear');
                end
                for j = 1:data2.num_data
                    idx = j+obj.num_data;
                    errors_eval(idx, i, :) = interp1(data2.stamps, ...
                                                     squeeze(data2.errors(j, i, :)), ...
                                                     stamps_eval, 'linear');
                end
            end

            % concat the extra fields
            extra_field_names = fieldnames(obj.extras);
            for i = 1:length(extra_field_names)
                name = extra_field_names{i};
                obj.extras.(name) = [obj.extras.(name); data2.extras.(name)];
            end

            % save it to the class variable
            obj.errors   = errors_eval;
            obj.stamps   = stamps_eval;
            obj.num_data = N_data;
        end

        function obj = extend(obj, varargin)
            % extend the tracking error data with cell array of
            % TrackingErrorData
            %
            % Input
            %   datas = cell array of TrackingErrorData
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                "required_key", {"datas"}, ...
                'default_key', {"dt"}, ...
                'default_value', {1e-4});

            datas = kwargs.datas;
            dt    = kwargs.dt;

            for i = 1:length(datas)
                obj = obj.append('data', datas{i}, 'dt', dt);
            end
        end

        function error_fn = error_function(obj)
            % Compute error function based on error data
            % error function is time-wise, dimension-wise maximum
            error_fn = zeros(obj.error_dim, length(obj.stamps));
            for i = 1:obj.error_dim
                errors_i = obj.errors(:, i, :);
                error_fn(i, :) = max(errors_i, [], 1);
            end       
        end

        function h = plot(obj)
            % plot pretty tracking error function
            % Author: Wonsuhk Jung

            error_fn = obj.error_function();

            palette = get_palette_colors();
            
            h = figure; 
            t = tiledlayout(obj.error_dim, 1);

            for i = 1:obj.error_dim
                nexttile(t, i);
                for j = 1:obj.num_data
                    p = plot(obj.stamps, squeeze(obj.errors(j, i, :))'); hold on;
                    p.Color = palette.pale_red;
                end

                p = plot(obj.stamps, error_fn(i, :));
                p.Color = palette.magenta;
                p.LineWidth = 4;
                p.LineStyle = '-.';

                grid on; xlabel("time [sec]");
                legend(p, "tracking error function");
            end
        end
    end
end