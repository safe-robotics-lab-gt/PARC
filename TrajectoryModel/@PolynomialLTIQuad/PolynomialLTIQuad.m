classdef PolynomialLTIQuad
    %   See Mueller et al. "A computationally efficient motion primitive
    %   for quadrocopter trajectory generation" for details of the 
    %   polynomial planning model
    
    properties
        peak_time
        final_time
        timestep
        domain
        systems
        step_number
    end
    
    methods
        function obj = PolynomialLTIQuad(varargin)
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                     'required_key' , ["peak_time", "final_time", ...
                     "timestep", "domain"]);

            obj.peak_time = kwargs.peak_time;
            obj.final_time = kwargs.final_time;
            obj.timestep = kwargs.timestep;
            obj.domain = kwargs.domain;
            obj.step_number = floor(obj.final_time./obj.timestep);

            obj.systems = obj.define_decoupled_systems();
        end
    end
end

