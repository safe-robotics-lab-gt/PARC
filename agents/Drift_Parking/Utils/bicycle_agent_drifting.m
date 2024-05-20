classdef bicycle_agent_drifting < handle
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
        ani_only = true;

        % input states
        states = []
        states_dot = []
        inputs = []
        delta_max
        Fxf_max
        Fxr_max
        init_state = []

        % fiala tire slipping model states
        fiala_tire_vars = []
        Fxf_act

        Kx = 2000;
        Kx_break = 2000;
        Kr = 0.4;
        Ky = -0.5;
        Kbeta = 1;
    end

    methods
        %% constructor
        function A = bicycle_agent_drifting(veh, tire_f, tire_r, input_bounds, dT)
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
            Fxf = bound_values(Fxf, -A.Fxf_max, A.Fxf_max);%1.5


            [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot,A.Fxf_act] = ...
                nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, A.veh, A.tire_f, A.tire_r, x, y, yaw);
            [alpha_slip_f, alpha_slip_r] = slip_angle_bound(A.tire_f, Fxf,A.tire_r, Fxr);

            zd = [x_dot;y_dot;yaw_dot;r_dot;V_dot;beta_dot;ux_dot;uy_dot];

            A.t_s = [A.t_s;t];

            vars = [alpha_f,alpha_r,alpha_slip_f,alpha_slip_r];
            A.fiala_tire_vars = [A.fiala_tire_vars;vars];

            input = [Fxr,delta,A.Fxf_act];
            A.inputs = [A.inputs;input];

            A.states_dot = [A.states_dot;zd'];
        end

        function u = get_control_inputs(A,t_curr,z_curr,T_des,U_des,Z_des)
            r = z_curr(4);
            ux = z_curr(7);
            uy = z_curr(8);
            beta = z_curr(6);
            % yaw = z_curr(3);

            r_des     = Z_des(1);
            V_des     = Z_des(2);
            beta_des  = Z_des(3);

            ux_des    = V_des*cos(beta_des);
            uy_des    = V_des*sin(beta_des);

            Fxr_ff    = U_des(1);
            delta_ff  = U_des(2);

            Fxr = A.Kx*(ux_des-ux)+Fxr_ff;
            % delta = A.Kr*(r_des-r)+A.Ky*(uy_des-uy)+delta_ff;
            %             delta = 0;
            % Fxf = 0;

            delta = beta;
            Fxr_prev = A.inputs(end,1);
            Fxf = -A.Fxf_max;

            if t_curr >= 0
                beta_init = A.init_state(6);
                beta_err_max = pi/2 - abs(beta_init);
                beta_err = pi/2 - abs(beta);
                beta_ratio = beta_err/beta_err_max;


                %
                % beta_dot_init = A.states_dot(1,6);
                % beta_dot_curr = A.state_dot(end,6);
                % beta_dot_ratio = beta_dot_curr
                % Fxr = A.Fxr_max;

                % Fxf = 0;
                Fxr = 0;
                %                 if t_curr > 1
                %                     Fxf = 0;
                %                 end
            end

            if Fxr_prev <=100 || ux <=1
                Fxr = 0;
            end

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

        function simulate(A, t_sim, init_states,input_guess, des_rVb_states, prev_inputs)
            A.t_s = [];
            A.states = [];
            A.states_dot = [];
            A.inputs = prev_inputs;
            A.fiala_tire_vars = [];

            t_span = 0:A.dT:t_sim;
            z_curr = init_states;
            A.init_state = init_states;

            U_ref = input_guess;
            Z_ref = des_rVb_states;

            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,t_span,U_ref,Z_ref),...
                [0 t_span(end)], z_curr);

            [u_t,uidx] = unique(A.t_s);
            u_inputs = A.inputs(uidx,:);
            u_fiala_tire_vars = A.fiala_tire_vars(uidx,:);
            u_states_dot = A.states_dot(uidx,:);

            interp_mode = 'previous';
            [A.inputs, A.fiala_tire_vars, A.states_dot] = match_trajectories(tout,u_t',u_inputs', ...
                u_t',u_fiala_tire_vars', ...
                u_t',u_states_dot',interp_mode);
            A.inputs = A.inputs';
            A.fiala_tire_vars = A.fiala_tire_vars';
            A.states_dot = A.states_dot';
            A.t_s = tout';
            A.states = zout';
            if A.visual
                A.visualize();
            end
        end

        function ax_space = plot_drift_sequence(A)
            % create a new axis for the graph
            % snap_ratio = 50; % less the more cars
            % alpha = 0.8; %0.6
            % alpha_change = false;

            snap_ratio = 30; % less the more cars
            alpha = 0.6; %0.6
            alpha_change = true;
            T_s               = A.t_s;
            x_m               = A.states(:,1);
            y_m               = A.states(:,2);
            yaw_rad           = A.states(:,3);
            V_mps             = A.states(:,5);
            delta_rad         = A.inputs(:,2);


            figure;
            ax_space = subplot(1,1,1);
            hold on;
            xlabel('$p_x$ (m)', 'Interpreter', 'latex');
            ylabel('$p_y$ (m)', 'Interpreter', 'latex')
            axis equal; grid on; box on;
            set(gca,'YTick',-1000:5:1000);
            set(gca,'XTick',-1000:5:1000);

            factor = 2.;
            % left = min(x_m)-factor*5;
            % right = max(x_m)+factor*5;
            % up = max(y_m)+factor*4/2;
            % down = min(y_m)-factor*4/2;
            left = -5;
            right = 40;
            down = -20;
            up = 5;
            xlim(ax_space, [ left, right]);
            ylim(ax_space, [ down, up]);
            % plot_world_road(ax_space, [ left, right], [down, up]);

            V_max = max(V_mps);
            plot_car(ax_space, [x_m(1);y_m(1)], yaw_rad(1), delta_rad(1),A.veh,0.5)% 0.8
            for idx = 1:snap_ratio:length(T_s)
                V_curr = abs(V_mps(idx));
                if alpha_change
                    alpha_rel = alpha*V_curr/V_max;
                else
                    alpha_rel = alpha;
                end
                plot_car(ax_space, [x_m(idx);y_m(idx)], yaw_rad(idx), delta_rad(idx),A.veh,alpha_rel)
            end
            plot_car(ax_space, [x_m(end);y_m(end)], yaw_rad(end), delta_rad(end),A.veh,1)

        end


        function ax_space = visualize(A)
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

            ax_space = animateDrift(ux_mps, uy_mps, r_radps, x_m, y_m, yaw_rad, slip_f_bool, ...
                slip_r_bool, delta_rad, A.veh, A.integrator_time_discretization);
            if ~A.ani_only
                figure(2); hold on;
                plot(A.t_s, ux_mps, 'r', 'LineWidth', 2);
                plot(A.t_s, uy_mps, 'b', 'LineWidth', 2);
                plot(A.t_s, r_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
                ylim([-8 15]);
                grid on; box on;
                title('Vehicle States over Time')
                xlabel('Time, s');
                ylabel('Vehicle states');
                legend('Ux, m/s', 'Uy, m/s', 'r, rad/s', 'Location', 'SouthEast');
                hold off;

                figure(3)
                subplot(3,1,1)
                plot(A.t_s, rad2deg(delta_rad))
                grid on
                ylabel('delta [deg]')
                title('Actuator Commands over Time')

                subplot(3,1,2)
                plot(A.t_s, Fxr_N)
                grid on
                ylabel('Fxr [N]')
                xlabel('Time [s]')

                subplot(3,1,3)
                plot(A.t_s, Fxf_N)
                grid on
                ylabel('Fxf [N]')
                xlabel('Time [s]')

                figure(4)
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
                % ylim([0,20])
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
                title('Vehicle States over Time')
                xlabel('Time, s');
                ylabel('Vehicle states');
                legend('r_dot, m/s^2', 'V_dot, m/s^2', 'beta_dot, rad/s', 'Location', 'best');
                hold off;

                figure(7); hold on;
                subplot(3, 1, 1);
                hold on;
                plot(A.t_s, A.states(:,1));
                ylabel('x (m)')

                subplot(3, 1, 2);
                hold on;
                plot(A.t_s, A.states(:,2));
                ylabel('y (m)')

                subplot(3,1,3);
                hold on;
                plot(A.t_s, A.states(:,3)*180/pi);
                ylabel('yaw (deg)')
            end
        end
    end
end