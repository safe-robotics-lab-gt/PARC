classdef turtlebot_iLQR_LLC < turtlebot_LLC
    % This function is adapted from from paper,
    % iLQR for Piecewise-Smooth Hybrid Dynamical Systems
    % https://arxiv.org/pdf/2103.14584.pdf
    %
    % code implementation can be found here:
    % https://github.com/Kongx231/ilqr
    % modified by Chuizheng Kong
    % on date: 11/26/2023
    
    % This class sets up the optimization problem with the constructor and
    % solve will return the optimal set of inputs and gains to track the desired trajectory.
    % Note that the discrete dynamics of the system and it's Jacobians are
    % required to be functions ready to be called with
    % inputs(state,input,dt,parameters)
     properties
        init_state_
        states_
        inputs_
        target_states_
        target_inputs_
        dt_
        start_time_
        end_time_
        time_span_
        
        A_
        B_
        Q_k_
        R_k_
        Q_T_
        n_states_
        n_inputs_
        f_
        parameters_
        
        K_feedback_
        k_feedforward_
        
        n_timesteps_
        n_iterations_
        
        % Expected cost reductions
        expected_cost_redu_
        expected_cost_redu_grad_
        expected_cost_redu_hess_

     end
    
    methods
        %% constructor
        function LLC = turtlebot_iLQR_LLC(varargin)
            
            LLC@turtlebot_LLC(varargin{:});
            turtlebot_discrete_dynamics_sym();
            
            % Dynamics
            LLC.n_states_ = size(states, 1);
            LLC.n_inputs_ = size(inputs, 1);
            LLC.dt_ = 0.01; % same as integrator_time_discretization
            LLC.f_ = @calc_f_disc;
            LLC.A_ = @calc_A_disc;
            LLC.B_ = @calc_B_disc;
            % Weighting
            LLC.Q_k_ = 0.5*eye(LLC.n_states_); % 10
            %LLC.Q_k_(1,1) = 5; LLC.Q_k_(2,2) = 5;LLC.Q_k_(3,3) = 1;
            LLC.R_k_ = 0.1*eye(LLC.n_inputs_);
            LLC.Q_T_ = 100*eye(LLC.n_states_); % 10
            %LLC.Q_T_(1,1) = 10; LLC.Q_T_(2,2) = 10;LLC.Q_T_(3,3) = 10;
            LLC.parameters_ = [];
             
        end

        function flag = sym_file_exists(obj)
            
            
            flag = true;

            for i = 1:numel(sym_files)
                flag_i = (exist(sym_files(i)) == 2);
                flag = flag && flag_i;
            end
        end
        
        %% solving optimal control iteratively
        function [states,inputs,k_feedforward,K_feedback, cost_arr] = iterate(LLC, z_init, T, U, Z, n_iterations)
            LLC.start_time_ = T(1);
            LLC.end_time_ = T(end);
            LLC.time_span_ = LLC.start_time_:LLC.dt_:LLC.end_time_;
            LLC.n_timesteps_ = size(LLC.time_span_,2);
            
            LLC.init_state_ = z_init;
            LLC.target_states_ = match_trajectories(LLC.time_span_, T,Z)';
            initial_guess = [U(1,1)*ones(LLC.n_timesteps_,1),... 
                            U(2,1)*ones(LLC.n_timesteps_, 1)];
            LLC.target_inputs_ = initial_guess;
            
            LLC.inputs_ = LLC.target_inputs_;
            LLC.states_ = LLC.target_states_;

            
            % Max iterations
            LLC.n_iterations_ = n_iterations;
            
            % Compute the rollout to get the initial trajectory with the
            % initial guess
            %[new_states,new_inputs] = LLC.rollout();
            [k_feedforward,K_feedback,expected_reduction] = LLC.backwards_pass();
            [new_states,new_inputs]=forwards_pass(LLC,0);
            LLC.states_ = new_states;
            LLC.inputs_ = new_inputs;


            % Compute the current cost of the initial trajectory
            current_cost = LLC.compute_cost(new_states,new_inputs);
            cost_arr = [];
            cost_arr = [cost_arr; current_cost];
            
            learning_speed = 0.95; % This can be modified, 0.95 is very slow
            low_learning_rate = 0.05; % if learning rate drops to this value stop the optimization
            low_expected_reduction = 1e-3; % Determines optimality
            armijo_threshold = 0.1; % Determines if current line search solve is good (this is typically labeled as "c")
            for ii = 1:LLC.n_iterations_
%                disp(['Starting iteration: ',num2str(ii)]);
                % Compute the backwards pass
                [k_feedforward,K_feedback,expected_reduction] = LLC.backwards_pass();
                
                if(abs(expected_reduction)<low_expected_reduction)
                    % If the expected reduction is low, then end the
                    % optimization
%                    disp("Stopping optimization, optimal trajectory");
                    break;
                end
                learning_rate = 1;
                armijo_flag = 0;
                % Execute linesearch until the armijo condition is met (for
                % now just check if the cost decreased) TODO add real
                % armijo condition
                while(learning_rate > 0.05 && armijo_flag == 0)
                    % Compute forward pass
                    [new_states,new_inputs]=forwards_pass(LLC,learning_rate);
                    new_cost = LLC.compute_cost(new_states,new_inputs);

                    % Calculate armijo condition
                    cost_difference = (current_cost - new_cost);
                    expected_cost_redu = learning_rate*LLC.expected_cost_redu_grad_ + learning_rate^2*LLC.expected_cost_redu_hess_;
                    armijo_flag = cost_difference/expected_cost_redu > armijo_threshold;
                    if(armijo_flag == 1)
                        % Accept the new trajectory if armijo condition is
                        % met
                        current_cost = new_cost;
                        cost_arr = [cost_arr; current_cost];
                        LLC.states_ = new_states;
                        LLC.inputs_ = new_inputs;
                    else
                        % If no improvement, decrease the learning rate
                        learning_rate = learning_speed*learning_rate;
                        % disp(['Reducing learning rate to: ',num2str(learning_rate)]);
                    end
                end
                if(learning_rate<low_learning_rate)
                    % If learning rate is low, then stop optimization
%                    disp("Stopping optimization, low learning rate");
                    break;
                end
            end
            % Return the current trajectory
            states = LLC.states_;
            inputs = LLC.inputs_;
        end

        %% define traj tracking cost function
        function total_cost = compute_cost(LLC,states,inputs)
            % Initialize cost
            total_cost = 0.0;
            for ii = 1:LLC.n_timesteps_
                current_x = states(ii,:)'; % Not being used currently
                current_u = inputs(ii,:)';

                % Get the current target input and state
                current_x_des = LLC.target_states_(ii,:)';
                current_u_des = LLC.target_inputs_(ii,:)';
                
                % Compute differences from the target
                x_difference = current_x_des - current_x;
                u_difference = current_u_des - current_u;
                current_cost = u_difference'*LLC.R_k_*u_difference + x_difference'*LLC.Q_k_*x_difference;
                total_cost = total_cost+current_cost;
            end
            % Compute terminal cost
            terminal_x_diff = LLC.target_states_(end,:)'-states(end,:)';
            terminal_cost = terminal_x_diff'*LLC.Q_T_*terminal_x_diff;
            total_cost = total_cost+terminal_cost;
        end
        
        %% simulating dynamics
        function [states,inputs] = rollout(LLC)
            states = zeros(LLC.n_timesteps_+1,LLC.n_states_);
            inputs = zeros(LLC.n_timesteps_,LLC.n_inputs_);
            current_state = LLC.init_state_;
            states(1,:) = current_state;
            for ii=1:LLC.n_timesteps_
                current_input = LLC.inputs_(ii,:)';
                next_state = LLC.f_(current_state,current_input,LLC.dt_,LLC.parameters_);
                % Store states and inputs
                states(ii+1,:) = next_state';
                inputs(ii,:) = current_input'; % in case we have a control law, we store the input used
                % Update the current state
                current_state = next_state;
            end
            % Store the trajectory (states,inputs)
            LLC.states_ = states;
            LLC.inputs_= inputs;
        end
        
        %% dynamic programing backward pass
        function [k_trj,K_trj,expected_cost_redu] = backwards_pass(LLC)
            % Initialize feedforward gains
            k_trj = zeros(size(LLC.inputs_));
            K_trj = zeros(size(LLC.inputs_,1),size(LLC.inputs_,2),size(LLC.states_,2));
            % Initialize expected cost reduction
            expected_cost_redu = 0;
            expected_cost_redu_grad = 0;
            expected_cost_redu_hess = 0;
            
            % Iitialize gradient and hessian of the value function
            V_x = LLC.Q_T_*(LLC.states_(end,:)'-LLC.target_states_(end,:)');
            V_xx = LLC.Q_T_;
            
            for ii = flip(1:LLC.n_timesteps_) % Go backwards in time
                % Get the current state and input
                current_x = LLC.states_(ii,:)';
                current_u = LLC.inputs_(ii,:)';
                
                % Get the gradient and hessian of the current cost
                l_x = zeros(size(current_x)); % Defined as zero right now because there is no desired trajectory
                l_xx = LLC.Q_k_; % Q_k should also be zero here
                l_u = LLC.R_k_*current_u;
                l_uu = LLC.R_k_;
                
                % Get the jacobian of the discretized dynamics
                A_k = LLC.A_(current_x,current_u,LLC.dt_,LLC.parameters_);
                B_k =LLC.B_(current_x,current_u,LLC.dt_,LLC.parameters_);
                
                % Compute the coefficient expansion terms
                Q_x = l_x+A_k'*V_x;
                Q_u = l_u+B_k'*V_x;
                Q_xx = l_xx+A_k'*V_xx*A_k;
                Q_ux = B_k'*(V_xx)*A_k;
                Q_uu = l_uu+B_k'*(V_xx)*B_k;
                
                % Compute the gains
                k = -(Q_uu)\Q_u;
                K = -(Q_uu)\Q_ux;
                
                % Update the gradient and hessian of the value function
                V_x = Q_x +K'*Q_uu*k +K'*Q_u + Q_ux'*k;
                V_xx = (Q_xx+Q_ux'*K+K'*Q_ux+K'*Q_uu*K);
                
                % Store the gains
                k_trj(ii,:) = k';
                K_trj(ii,:,:) = K;
                
                % Get the current expected cost reduction from each source
                current_cost_reduction_grad = -Q_u'*k;
                current_cost_reduction_hess = 0.5 * k'*(Q_uu)*k;
                current_cost_reduction = current_cost_reduction_grad + current_cost_reduction_hess;
                
                % Store each component separately for computing armijo
                % condition
                expected_cost_redu_grad = expected_cost_redu_grad + current_cost_reduction_grad;
                expected_cost_redu_hess = expected_cost_redu_hess + current_cost_reduction_hess;
                expected_cost_redu = expected_cost_redu+current_cost_reduction;
                
            end
            % Store expected cost reductions
            LLC.expected_cost_redu_grad_ = expected_cost_redu_grad;
            LLC.expected_cost_redu_hess_ = expected_cost_redu_hess;
            LLC.expected_cost_redu_ = expected_cost_redu;
            
            % Store gain schedule
            LLC.k_feedforward_= k_trj;
            LLC.K_feedback_ = K_trj;
        end
        
        %% forward iteration pass
        function [states,inputs]=forwards_pass(LLC,learning_rate)
            states = zeros(LLC.n_timesteps_+1,LLC.n_states_);
            inputs = zeros(LLC.n_timesteps_,LLC.n_inputs_);
            current_state = LLC.init_state_;
            
            % set the first state to be the initial
            states(1,:) = current_state';
            for ii=1:LLC.n_timesteps_
                % Get the current gains and compute the feedforward and
                % feedback terms
                current_feedforward = learning_rate*LLC.k_feedforward_(ii,:)';
                current_feedback = reshape(LLC.K_feedback_(ii,:,:),LLC.n_inputs_,LLC.n_states_)*(current_state-LLC.states_(ii,:)');
                current_input = LLC.inputs_(ii,:)' + current_feedback + current_feedforward;
                
                % simualte forward
                next_state = LLC.f_(current_state,current_input,LLC.dt_,LLC.parameters_);
                % Store states and inputs
                states(ii+1,:) = next_state';
                inputs(ii,:) = current_input';
                % Update the current state
                current_state = next_state;
            end
        end
        
        %% inferencing final optimal control input
        function U = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            p_cur = z_cur(A.position_indices) ;
            h_cur = z_cur(A.heading_index) ;
            v_cur = z_cur(A.speed_index) ;

            Z_opt = LLC.states_';
            U_opt = LLC.inputs_';
            T_opt = LLC.time_span_;
            
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_opt = zeros(2,1);
                p_opt = z_cur(A.position_indices) ;
                v_opt = 0 ;
                h_opt = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_opt,z_opt] = match_trajectories(t_cur,T_opt,U_opt,[T_opt,T_opt(end)+LLC.dt_],Z_opt,LLC.interp_mode) ;
                p_opt = z_opt(A.position_indices) ;
                v_opt = z_opt(A.speed_index) ;
                h_opt = z_opt(A.heading_index) ;
            end
            
            % create output
            U = u_opt;
        end
        
        
        
    end
end