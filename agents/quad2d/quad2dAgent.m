classdef quad2dAgent < RTD_agent_2D
    % 10D near-hover agent implementation
    %
    % Author Wonsuhk Jung, Chuizheng Kong
    %
    % Created Jan 26th 2023

    % [1] On-board Model Predictive Control of a Quadrotor Helicopter

    properties

        state_desired
        input_histories
        input_bounds
        state_bounds
        t_eval

        r   % quadrotor arm radius
        m   % quadrotor mass
        I   % quadrotor inertia
        g   % gravity

        integrator_type = 'ode45' ; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01 ; % for ode4
    end

    methods
        function A = quad2dAgent(varargin)
            % Inherits all the property of Quadrotor Agent but with different
            % dynamics. This agent is subtle - it pretends to be a SE(3) agent
            % by rolling-out dynamics that does not fit with SE(3) agent but
            % updating the unmodeled part by other states according to
            % near-hover assumption.

            A@RTD_agent_2D()
            A.LLC = quad2d_iLQR_LLC;

            A.r = 0.25;
            A.m = 1.0;
            A.I = 0.01;
            A.g = 9.81;
            
            A.footprint = A.r;
            A.make_footprint_plot_data() ;
            A.make_arrow_plot_data() ;
            
            A.n_states = 6 ;
            A.n_inputs = 2 ;
            A.state_desired = [];
            A.input_histories = [];
            A.t_eval = [];
            f_center = A.m*A.g/2;
            A.input_bounds    = [f_center-4, f_center+4;
                f_center-4, f_center+4];
            A.state_bounds = [-2,2; -2,2; -pi,pi; -2,2; -2,2; -2*pi,2*pi];
        end

        function reset(A,state)
            % do the reset
            A.state_desired = [];
            A.input_histories = [];
            A.t_eval = [];
            if nargin < 2
                state = zeros(A.n_states,1) ;
            end
            
            % do the reset
            A.state = zeros(A.n_states,1) ;
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ;
            A.input_time = 0 ;
            
            % reset the state
            switch length(state)
                case A.n_states
                    A.state = state ;
                case 2
                    A.state(A.position_indices) = state ;
                case 3
                    A.state([A.position_indices,A.heading_index]) = state ;
                otherwise
                    error(['The provided state has an incorrect number of elements!',...
                        ' Please provide either a 2-by-1 position, a 3-by-1 position',...
                        ' and heading, or an n_states-by-1 full state vector.'])
            end
            
            A.desired_trajectory = [];
            A.desired_input = [];
            A.desired_time = [];
        end



        function [t_out, s_out] = integrator(A, fun, t_span, s_0)
            switch A.integrator_type
                case 'ode45'
                    [t_out, s_out] = ode45(fun, t_span, s_0) ;
                    if isempty(s_out)
                        error('ODE45 failed! Try Euler integration instead. Ugh.')
                    end
                case 'euler'
                    [t_out, s_out] = ode1(fun, t_span, s_0);
                otherwise
                    error('Please set ode_solver to "ode45" or "euler"')
            end
        end

        function move(A,t_move,T_ref,U_ref,Z_ref)
            % method: move(t_move, T_ref, U_ref, Z_ref)
            %
            % Use quad2d dynamics to roll-out and keep all the format
            % consistent with the SE3_agent

            if nargin < 5
                Z_ref = [];
            end

            [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;



            % 1. iterate the iLQR controller
            [states_des,inputs,k_feedforward,K_feedback, cost_arr] = ...
            A.LLC.iterate(Z_ref(:,1), T_ref, U_ref, Z_ref, 2000);

            % 2. run near-hover quadrotor dynamics
            t_span = 0:A.integrator_time_discretization:t_move;
            [t_out, s_out] = A.integrator(@(t,s) A.dynamics(t,s,T_ref,U_ref,Z_ref),...
                t_span, Z_ref(:,1));

            A.commit_move_data(t_out', s_out', A.t_eval, A.input_histories, A.state_desired);
        end
        
        function commit_move_data(A, T_act, Z_act, T_in, U_in, Z_in)
            % everything in wide format
            [u_t,uidx] = unique(T_in);
            u_inputs = U_in(:,uidx);
            u_states_des = Z_in(:,uidx);

            interp_mode = 'previous';
            [U_in, Z_in] = match_trajectories(T_act,u_t,u_inputs, ...
                u_t,u_states_des,interp_mode);
            commit_move_data@RTD_agent_2D(A,T_act,Z_act,T_in,U_in,Z_in) ;
        end

        function s_dot = dynamics(A,t,s,T_ref,U_ref,Z_ref)
            % s_dot = dynamics(t,s,T,U,params)
            %
            %
            % Authors: Shreyas Kousik, Wonsuhk Jung, Chuizheng Kong
            % Created: 14 Jan 2024
            % Updated: 26 Jan 2024

            % parse property of quad2d agent

            % x           = bound_values(s(1),A.state_bounds(1,2));
            % z           = bound_values(s(2),A.state_bounds(2,2));
            % theta       = bound_values(s(3),A.state_bounds(3,2));
            % vx          = bound_values(s(4),A.state_bounds(4,2)) ;
            % vz          = bound_values(s(5),A.state_bounds(5,2)) ;
            % theta_dot   = bound_values(s(6),A.state_bounds(6,2));
            
            theta       = s(3);
            vx          = s(4);
            vz          = s(5);
            theta_dot   = s(6);


            % compute control inputs (angular accelerations)
            [u,z_des] = A.LLC.get_control_inputs(A,t,s,T_ref,U_ref,Z_ref) ;
            u = bound_values(u, A.input_bounds(:, 1), A.input_bounds(:, 2));

            F1 = u(1) ; % rear propeller force
            F2 = u(2) ; % front force
            

            % set disturbances
            d_x = 0 ;
            d_z = 0 ;

            % compute dynamics
            s_dot = [vx+d_x;
                vz+d_z;
                theta_dot;
                1/A.m*sin(theta)*(F1+F2);
                1/A.m*cos(theta)*(F1+F2)-A.g;
                A.r/A.I*(F1-F2)];

            A.input_histories = [A.input_histories, u];
            A.state_desired = [A.state_desired, z_des];
            A.t_eval = [A.t_eval, t];
        end

        function plot_at_time(A,t)
            % compute footprint for plot
            % had to switch the heading since it's in x-z frame, R_y + is
            % into the page
            z_t = match_trajectories(t,A.time,A.state) ;
            p_t = z_t(A.position_indices) ;
            h_t = z_t(A.heading_index) ;
            R_t = rotation_matrix_2D(h_t*-1) ; %%%%%here
            fp_t = A.footprint_vertices(:,1:end-1) ;
            N_fp = size(fp_t,2) ;
            V_fp = R_t*fp_t + repmat(p_t,1,N_fp) ;
            
            % make arrow for plot
            A.arrow_vertices = [-0.1, -0.09, -0.09, 0.09, 0.09, 0.1, 0.1, -0.1; 0.05, 0.05, 0.01, 0.01, 0.05, 0.05, -0.01, -0.01];
            V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,8) ;
            
            % plot
            hold_check = hold_switch() ;
            
            if check_if_plot_is_available(A,'arrow')
                % A.plot_data.footprint.Vertices = V_fp' ;
                A.plot_data.arrow.Vertices = V_arrow' ;
            else
                % plot footprint
                % fp_data = patch(V_fp(1,:),V_fp(2,:),A.plot_footprint_color,...
                %     'EdgeColor',A.plot_footprint_edge_color,...
                %     'FaceAlpha',A.plot_footprint_opacity,...
                %     'EdgeAlpha',A.plot_footprint_edge_opacity) ;
                
                % plot arrow on footprint
                arrow_data = patch(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
                    'EdgeColor',A.plot_arrow_color,...
                    'FaceAlpha',A.plot_arrow_opacity,...
                    'EdgeAlpha',A.plot_arrow_opacity) ;
                
                % save plot data
                % A.plot_data.footprint = fp_data ;
                A.plot_data.arrow = arrow_data ;
            end
            
            if A.plot_trajectory_at_time_flag
                % get the executed path up to the current time
                X = A.state(A.position_indices,:) ;
                T_log = A.time <= t ;
                X = X(:,T_log) ;
                
                % plot it
                if check_if_plot_is_available(A,'trajectory')
                    A.plot_data.trajectory.XData = X(1,:) ;
                    A.plot_data.trajectory.YData = X(2,:) ;
                end
                    traj_data = plot_path(X,'b-') ;
                    A.plot_data.trajectory = traj_data ;
            end
            
            hold_switch(hold_check) ;
        end
    end
end

