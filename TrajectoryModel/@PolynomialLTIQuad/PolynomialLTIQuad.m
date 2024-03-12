classdef PolynomialLTIQuad
    %POLYNOMIALLTIQUAD Summary of this class goes here
    %   Detailed explanation goes here
    
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
            %POLYNOMIALLTIQUAD Construct an instance of this class
            %   Detailed explanation goes here
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

