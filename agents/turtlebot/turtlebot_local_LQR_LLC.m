classdef turtlebot_local_LQR_LLC < turtlebot_LLC
    methods
        %% constructor
        function LLC = turtlebot_local_LQR_LLC(varargin)
            % set default lookahead properties
            lookahead_distance = 0.01 ; % m
            lookahead_time = 0.01 ; % s
            lookahead_type = 'time' ;
            
            % set default gains for LQR
            DG.position = 200 ;
            DG.speed = 100 ;
            DG.yaw = 10 ;
            
            % create low-level controller
            LLC@turtlebot_LLC('lookahead_distance',lookahead_distance,...
                'lookahead_time',lookahead_time,...
                'lookahead_type',lookahead_type,...
                'gains',DG,'default_gains',DG,'stop_gains',DG,...
                varargin{:}) ;            
        end
        
        %% get control inputs
        function U = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            p_cur = z_cur(A.position_indices) ;
            h_cur = z_cur(A.heading_index) ;
            v_cur = z_cur(A.speed_index) ;
            
            % get time along traj to use for feedback
            t_lkhd = LLC.lookahead_time ;
            t_fdbk = min(t_cur + t_lkhd, T_des(end)) ;
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_fdbk,T_des,U_des,LLC.interp_mode) ;
                p_des = z_cur(A.position_indices) ;
                v_des = 0 ;
                h_des = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_des,z_des] = match_trajectories(t_fdbk,T_des,U_des,T_des,Z_des,LLC.interp_mode) ;
                p_des = z_des(A.position_indices) ;
                v_des = z_des(A.speed_index) ;
                h_des = z_des(A.heading_index) ;
            end
            
            %solve iterative LQR problem
            
            % obtaining the lqr gain based on states
            q_pos = LLC.default_gains.position;
            q_h = LLC.default_gains.yaw;
            q_v = LLC.default_gains.speed;
            
            % linearized using taylor expansion
            A = [0, 0, -v_cur*sin(h_cur), cos(h_cur);
                 0, 0, v_cur*cos(h_cur), sin(h_cur);
                 0, 0, 0, 0;
                 0, 0, 0, 0];

            B = [0, 0;
                 0, 0;
                 1, 0;
                 0, 1];
             
             Q = diag([q_pos,q_pos,q_h,q_v]);
             R = diag([1,1]);

             
             [K, ~,~] = lqr(A,B,Q,R);

            % get desired feedforward inputs
            w_des = u_des(1) ; %ignored
            a_des = u_des(2) ; % ignored
            
            % get gains
            control_input = -K*([p_cur - p_des; h_cur - h_des; v_cur - v_des]);
            
            % compute unsaturated inputs (they get saturated by the agent)
            w_out = control_input(1);
            a_out = control_input(2);
            
            % create output
            U = [w_out ; a_out] ;
        end
    end
end