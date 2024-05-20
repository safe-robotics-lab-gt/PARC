classdef bicycle_agent_predrifting < handle
    % drift parking vehicle simulator
    % Author: Chuizheng Kong
    % Created: 01/16/2024

    properties

        veh
        tire_f
        tire_r
        dT
        t_s = []

        integrator_type
        integrator_time_discretization
        visual = true;

        % input states
        states = []
        states_dot = []
        inputs = []
        delta_max
        Fxf_max
        Fxr_max

        % fiala tire slipping model states
        fiala_tire_vars = []

        Q_k
        R_k
        Q_T
        k_vd
        t_plan
        t_horizon
        t_sync
        n_iterations
        controller
        mpc_states
        mpc_inputs
        idx_curr
        states_plan
        states_pred
        inputs_pred
        V_dot_

    end

    methods
        %% constructor
        function A = bicycle_agent_predrifting(veh, tire_f, tire_r, input_bounds, dT)
            A.veh = veh;
            A.tire_f = tire_f;
            A.tire_r = tire_r;
            A.dT = dT;

            A.integrator_type = 'ode4';
            A.integrator_time_discretization = 0.01;

            A.Fxr_max = input_bounds(1);
            A.delta_max = input_bounds(2); % [rad]
            A.Fxf_max = input_bounds(3);

        end

        function zd = dynamics(A,t,z,T,U,Z)
            x       = z(1);
            y       = z(2);
            yaw     = z(3);
            r       = z(4);
            V       = z(5);
            beta    = z(6);

            ux      = z(7);
            uy      = z(8);

            u = A.get_control_inputs(t,z,T,U,Z);
            u(isnan(u)) = 0; %safety check
            Fxr = u(1);
            delta = u(2);
            Fxf = u(3);
            if ~isempty(A.inputs)
                delta_prev = A.inputs(end,2);
                if abs(delta - delta_prev) > deg2rad(1) % input rate of change limit
                    delta = delta_prev + sign(delta-delta_prev)*deg2rad(1);
                end
            end
            delta = bound_values(delta, -A.delta_max, A.delta_max);
            Fxr = bound_values(Fxr, 0, A.Fxr_max);
            Fxf = bound_values(Fxf, -A.Fxf_max/1.5, 100);%1.5


            [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot] = ...
                nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, A.veh, A.tire_f, A.tire_r, x, y, yaw);
            [alpha_slip_f, alpha_slip_r] = slip_angle_bound(A.tire_f, Fxf,A.tire_r, Fxr);

            if ~isnan(V_dot)
                A.V_dot_ = V_dot;
            end

            zd = [x_dot;y_dot;yaw_dot;r_dot;V_dot;beta_dot;ux_dot;uy_dot];

            A.t_s = [A.t_s;t];
            A.t_sync = [A.t_sync;A.idx_curr];

            vars = [alpha_f,alpha_r,alpha_slip_f,alpha_slip_r];
            A.fiala_tire_vars = [A.fiala_tire_vars;vars];

            input = [Fxr,delta,Fxf];
            A.inputs = [A.inputs;input];

            A.states_dot = [A.states_dot;zd'];
        end

        function u = get_control_inputs(A,t_curr,z_curr,T_des,U_des,Z_des)
            x = z_curr(1);
            y = z_curr(2);
            yaw = z_curr(3);
            V = z_curr(5);
            current_state = [x;y;yaw;V];

            [~,idx] = min(abs(T_des-t_curr));
            if idx ~= A.idx_curr
                [states_solve,inputs_solve,~,~,~] = A.controller.solve_ilqr(idx,current_state);
                A.mpc_states(idx,:,:) = states_solve;
                A.mpc_inputs(idx,:,:) = inputs_solve;
                A.states_pred(idx,:) = states_solve(1,:);
                A.inputs_pred(idx,:) = inputs_solve(1,:);
                A.idx_curr = idx;
                % disp('current time step: '+string(idx)+'/'+string(length(T_des)));
            end

            V_dot_pred = A.inputs_pred(A.idx_curr,1);
            Fxr = A.k_vd*(V_dot_pred - A.V_dot_)*A.veh.m;
            delta = A.inputs_pred(A.idx_curr,2);
            if ~isempty(A.fiala_tire_vars)
                fiala_drift_margin_front_rear = A.fiala_tire_vars(end,3:4) - A.fiala_tire_vars(end,1:2);
                front_slip_bool = fiala_drift_margin_front_rear(1,2) < 0;
                if front_slip_bool
                    delta = A.inputs(end,2);
                end
            end
            Fxf = 0;

            u = [Fxr;delta;Fxf];
        end

        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end

        function simulate(A, t_p, init_states,input_guess, target_states, mpc_params)
            A.t_s = [];
            A.states = [];
            A.states_dot = [];
            A.inputs = [];
            A.fiala_tire_vars = [];

            A.t_plan = t_p;
            A.t_sync = [];
            N = length(t_p);
            n_states = 4;
            n_inputs = 2;
            A.Q_k = mpc_params.Q_k;
            A.R_k = mpc_params.R_k;
            A.Q_T = mpc_params.Q_T;
            param = mpc_params.parameters;
            A.t_horizon = mpc_params.t_horizon;
            A.n_iterations = mpc_params.n_iterations;
            horizon = round(A.t_horizon / A.dT);
            A.idx_curr = 0;
            A.mpc_states = zeros(N,horizon+1,n_states);
            A.mpc_inputs = zeros(N,horizon,n_inputs);
            A.k_vd = 2;
            A.V_dot_ = 0;

            A.states_plan = target_states;
            A.states_pred = zeros(N,n_states);
            A.inputs_pred = zeros(N,n_inputs);

            padded_states = [target_states;repmat(target_states(end,:),horizon,1)];
            padded_inputs = [input_guess;repmat(input_guess(end,:),horizon,1)];

            A.controller = ilqr_mpc(padded_states,padded_inputs,A.dT,horizon, ...
                @bike_f_disc,@bike_A_disc,@bike_B_disc, ...
                A.Q_k,A.R_k,A.Q_T,param,A.n_iterations);

            z_curr = [init_states(1:3);0;init_states(4);0;0;0];

            U_ref = input_guess;
            Z_ref = target_states;

            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,t_p,U_ref,Z_ref),...
                [0 t_p(end)], z_curr);

            [u_t,uidx] = unique(A.t_s);
            u_inputs = A.inputs(uidx,:);
            u_fiala_tire_vars = A.fiala_tire_vars(uidx,:);
            u_states_dot = A.states_dot(uidx,:);
            u_t_sync = A.t_sync(uidx);

            interp_mode = 'previous';
            [A.inputs, A.fiala_tire_vars, A.states_dot, A.t_sync] = match_trajectories(tout,u_t',u_inputs', ...
                u_t',u_fiala_tire_vars', ...
                u_t',u_states_dot', ...
                u_t',u_t_sync',interp_mode);
            A.inputs = A.inputs';
            A.fiala_tire_vars = A.fiala_tire_vars';
            A.states_dot = A.states_dot';
            A.t_s = tout';
            A.states = zout';
            A.t_sync = A.t_sync';
            if A.visual
                A.visualize();
            end
        end

        function visualize(A)
            x_m               = A.states(:,1);
            y_m               = A.states(:,2);
            yaw_rad           = A.states(:,3);
            r_radps           = A.states(:,4);
            V_mps             = A.states(:,5);
            beta_rad          = A.states(:,6);
            ux_mps            = A.states(:,7);
            uy_mps            = A.states(:,8);

            alphaF_rad        = A.fiala_tire_vars(:,1);
            alphaR_rad        = A.fiala_tire_vars(:,2);
            alphaF_slip_rad   = A.fiala_tire_vars(:,3);
            alphaR_slip_rad   = A.fiala_tire_vars(:,4);

            Fxr_N             = A.inputs(:,1);
            delta_rad         = A.inputs(:,2);
            Fxf_N             = A.inputs(:,3);

            r_dot_radpss      = A.states_dot(:,4);
            V_dot_mpss        = A.states_dot(:,5);
            beta_dot_radps    = A.states_dot(:,6);

            slip_f_bool = abs(alphaF_rad) > alphaF_slip_rad;
            slip_r_bool = abs(alphaR_rad) > alphaR_slip_rad;

            x_m_plan            = A.states_plan(:,1);
            y_m_plan            = A.states_plan(:,2);
            V_mps_plan          = A.states_plan(:,4);
            x_m_pred            = A.states_pred(:,1);
            y_m_pred            = A.states_pred(:,2);
            V_mps_pred          = A.states_pred(:,4);
            V_dot_pred          = A.inputs_pred(:,1);

            f = animateDrift_MPC(ux_mps, uy_mps, r_radps, x_m, y_m, yaw_rad, ...
                slip_f_bool, slip_r_bool, delta_rad, A.t_sync,A.mpc_states, A.veh, A.integrator_time_discretization);
            plot(x_m_plan,y_m_plan,'b--')
            plot(x_m_pred,y_m_pred,'k--')
            hold off

            figure(1);
            grid on; box on;
            plot(A.t_s, V_mps, 'r', 'LineWidth', 2);
            hold on;
            plot(A.t_plan, V_mps_plan, 'b--', 'LineWidth', 3);
            plot(A.t_plan, V_mps_pred, 'k--', 'LineWidth', 3);
            title('Vehicle velocity over Time')
            xlabel('Time, s');
            legend('V, m/s', 'v_{planed}, m/s', 'v_{pred}, m/s', 'Location', 'best');
            hold off;

            figure(2)
            subplot(3,1,1)
            plot(A.t_s, rad2deg(delta_rad))
            grid on
            ylabel('delta [deg]')
            title('Actuator Commands over Time')

            subplot(3,1,2)
            plot(A.t_s, V_dot_mpss)
            hold on
            plot(A.t_plan, V_dot_pred,'b--')
            hold off
            grid on
            ylabel('V_dot [m/s^2]')
            xlabel('Time [s]')

            subplot(3,1,3)
            plot(A.t_s, Fxr_N)
            grid on
            ylabel('Fxr [N]')
            xlabel('Time [s]')

            figure(3)
            subplot(2,1,1)
            title('Vehicle States over Time')
            plot(A.t_s, abs(alphaF_rad),'LineWidth',1.5)
            hold on
            plot(A.t_s, alphaF_slip_rad,'LineWidth',1.5)
            hold off
            legend('\alpha_f (rad)', '\alpha_f_{slip}')

            subplot(2,1,2)
            plot(A.t_s, abs(alphaR_rad),'LineWidth',1.5)
            hold on
            plot(A.t_s, alphaR_slip_rad,'LineWidth',1.5)
            hold off
            legend('\alpha_r (rad)', '\alpha_r_{slip}')

            figure(5)
            subplot(3,1,1)
            plot(A.t_s, rad2deg(beta_rad), 'r', 'LineWidth', 2);
            grid on
            ylabel('beta, deg');
            ylim([-100,100])
            subplot(3,1,2)
            plot(A.t_s, V_mps, 'b', 'LineWidth', 2);
            grid on
            ylabel('V, m/s');
            % ylim([0,17])
            subplot(3,1,3)
            plot(A.t_s, r_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
            grid on
            xlabel('Time, s');
            ylabel('r, rad/s');

            figure(6); hold on;
            plot(A.t_s, r_dot_radpss, 'r', 'LineWidth', 2);
            plot(A.t_s, V_dot_mpss, 'b', 'LineWidth', 2);
            plot(A.t_s, beta_dot_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
            grid on; box on;
            ylim([-10,10])
            title('Vehicle States over Time')
            xlabel('Time, s');
            ylabel('Vehicle states');
            legend('r_dot, m/s^2', 'V_dot, m/s^2', 'beta_dot, rad/s', 'Location', 'best');
            hold off;

        end
    end
end