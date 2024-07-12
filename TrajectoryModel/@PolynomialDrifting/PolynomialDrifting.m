classdef PolynomialDrifting
    %POLYNOMIALDRIFTING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        final_time
        timestep
        coef
        t_n
        Cs
        ds
    end
    
    methods
        function obj = PolynomialDrifting(varargin)
            %POLYNOMIALDRIFTING Construct an instance of this class
            %   Detailed explanation goes here
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                     'required_key' , ["final_time", "timestep", "coef"]);
            obj.final_time = kwargs.final_time;
            obj.timestep = kwargs.timestep;
            obj.coef = kwargs.coef;
            obj.t_n = size(obj.coef, 1);
            [obj.Cs, obj.ds] = obj.define_dynamic_matrices();
        end
    end
end

