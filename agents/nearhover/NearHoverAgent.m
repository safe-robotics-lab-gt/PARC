classdef NearHoverAgent < quadrotor_agent
% 10D near-hover agent implementation
%
% Author Wonsuhk Jung
%
% Created Jan 16th 2023

% [1] On-board Model Predictive Control of a Quadrotor Helicopter
    
properties
    n_0 % roll/pitch control gain
    k_T % thrust coefficient
    d_0 % attitude coefficient (pitch)
    d_1 % attitude coefficient (roll)

    input_histories
    input_bounds
    footprint_polytope
    animation_video_filename
end

methods
    function A = NearHoverAgent(varargin)
        % Inherits all the property of Quadrotor Agent but with different
        % dynamics. This agent is subtle - it pretends to be a SE(3) agent
        % by rolling-out dynamics that does not fit with SE(3) agent but
        % updating the unmodeled part by other states according to
        % near-hover assumption.
        A@quadrotor_agent()
        
        % Fastrack Setting
        A.n_0 = 10;
        A.k_T = 0.91;
        A.d_0 = 10;
        A.d_1 = 8;

        A.integrator_method = "ode45";
        A.integrator_time_discretization = 0.01;

        A.footprint_polytope = make_set_cuboid3D('center', [0; 0; 0], ...
            'thickness', 0.54, 'width', 0.54, 'height', 0.05);

        A.n_inputs        = 3 ;
        A.input_histories = [];
        A.input_bounds    = [-pi/9, pi/9; 
                             -pi/9, pi/9;
                                 0, 1.5*A.gravity_acceleration];
        A.animation_video_filename = "agent_animation";

        
    end

    function [s, R] = dynamics_state_to_agent_state(A, s_nh)
        % Convert the near-hover quadrotor's state to agent state which is
        % (px, py, pz, vx, vy, vz, wx, wy, wz), SO(3)
        % Argument
        %   s_nh : (s_nh_dim, num_states)
        %
        % Output
        %   s    : (sdim, num_states)
        %   R    : (3, 3, num_states)
        %
        % Author: Wonsuhk Jung
        % Created Jan 16 2023

        if size(s_nh, 2) == 10, s_nh = s_nh'; end

        p  = s_nh(A.position_indices, :);
        v  = s_nh(A.velocity_indices, :);
        th = s_nh(7:8, :);  % pitch-roll
        w  = s_nh(9:10, :); % pitch-roll

        % Yaw and yaw rate is assumed to be zero here.
        zero_vec = zeros(1, size(p, 2));
        s = [p; v; w; zero_vec];
        R = eul2rotm([th(2, :)', th(1, :)', zero_vec'], 'ZYX');
    end

    function s_nh = agent_state_to_dynamics_state(A, s, R)
        % Convert the agent state s = (p, v, w), R into near-hover state
        % (px, py, pz, vx, vy, vz, thx, thy, wx, wy) \in R^10.
        %
        % Argument
        %   s    : (sdim, num_states)
        %   R    : (3, 3, num_states)
        % Output
        %   s_nh : (10, num_states)
        %
        % Author: Wonsuhk Jung
        % Created Jan 16 2023

        if size(s, 2) == A.n_states, s = s'; end

        p  = s(A.position_indices, :);
        v  = s(A.velocity_indices, :);
        w  = s(A.angular_velocity_indices, :);
        
        % The rotation matrix follow yaw-pitch-roll convention ([1] pg.12)
        th   = rotm2eul(R, 'ZYX');
        th_x = th(:, 2)'; % pitch - why is this th_x, confirm this with Shreyas
        th_y = th(:, 1)'; % roll  - why is this th_y, confirm this with Shreyas

        assert(norm(th(:, 3)) == 0, "The yaw angle should be always zero.");

        s_nh = [p; v; th_x; th_y; w(1:2)];
    end


    function [t_out, s_out] = integrator(A, fun, t_span, s_0)
        switch A.integrator_method
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
        % Use near-hover dynamics to roll-out and keep all the format
        % consistent with the SE3_agent

        if nargin < 5
            Z_ref = [];
        end

        [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;

        % 1. convert [p,v,w,R] state into the near-hover quadrotor's state
        s_0 = A.state(:, end);
        R_0 = A.attitude(:, :, end);

        % 1. convert agent state configuration into near-hover state
        s_nh_0 = A.agent_state_to_dynamics_state(s_0, R_0);

        % 2. run near-hover quadrotor dynamics
        t_span = 0:A.integrator_time_discretization:t_move;
        [t_out, s_nh_out] = A.integrator(@(t,s) A.dynamics(t,s,T_ref,U_ref,Z_ref),...
            t_span, s_nh_0);

        % fix row-column
        t_out    = t_out';
        s_nh_out = s_nh_out';

        % 3. convert near-hover quadrotor's state into [p,v,w,R] state
        % column-stacking
        [s_out, R_out] = A.dynamics_state_to_agent_state(s_nh_out);

        A.commit_move_data(t_out, s_out, R_out, T_used, U_used, Z_used);
    end

    function s_dot = dynamics(A,t,s,T_ref,U_ref,Z_ref)
        % s_dot = dynamics(t,s,T,U,params)
        %
        % ODE dynamic model of quadrotor with 10 states from the FasTrack paper
        % s = [x, y, z, vx, vy, vz, thx, thy, wx, wy] follows Haimin's TubeMPC
        %
        % Authors: Shreyas Kousik, Wonsuhk Jung
        % Created: 14 Jan 2024
        % Updated: 15 Jan 2024
        
        % parse property of near-hover agent
        g   = A.gravity_acceleration;
        
        % velocities
        v_x = s(4) ;
        v_y = s(5) ;
        v_z = s(6) ;
        
        % roll and pitch angles ("th" for "theta")
        th_x = s(7) ;
        th_y = s(8) ;
        
        % angular velocities
        w_x = s(9) ;
        w_y = s(10) ;
        
        % compute control inputs (angular accelerations)
        u = A.LLC.get_control_inputs(A,t,s,T_ref,U_ref,Z_ref) ;
        u = u + [0; 0; g/A.k_T]; % additional term cancels gravity
        u = clip(u, A.input_bounds(:, 1), A.input_bounds(:, 2));
        
        a_x = u(1) ; % pitch acceleration
        a_y = u(2) ; % roll acceleration
        a_z = u(3) ; % vertical thrust 

        % set disturbances
        d_x = 0 ;
        d_y = 0 ;
        d_z = 0 ;
        
        % compute dynamics
        s_dot = [v_x + d_x ;
                 v_y + d_y ;
                 v_z + d_z ;
                 g*tan(th_x) ;
                 g*tan(th_y) ;
                 A.k_T*a_z - g ;
                 -A.d_1*th_x + w_x ;
                 -A.d_1*th_y + w_y ;
                 -A.d_0*th_x + A.n_0*a_x ;
                 -A.d_0*th_y + A.n_0*a_y ];
        
        A.input_histories = [A.input_histories, u];
    end

    function [filename, encoder] = video_setup(A)
        % Check os
        os = get_current_os();
        switch os
            case "linux"
                video_ext = ".avi";
                video_enc = "Motion JPEG AVI";
            case "mac"
                video_ext = ".mp4";
                video_enc = "MPEG-4";
        end

        % No duplication
        filename = A.animation_video_filename;
        dir_content = dir(pwd) ;
        filenames   = {dir_content.name} ;
        file_check  = any(cellfun(@(x) strcmp(strcat(filename, video_ext),x),filenames)) ;
        filename_new = filename ;
        cur_int = 1 ;

        while file_check
            filename_new = strcat(filename,'_',num2str(cur_int)) ;
            file_check  = any(cellfun(@(x) strcmp(strcat(filename_new, video_ext),x),filenames)) ;
            cur_int = cur_int + 1 ;
        end

        filename = filename_new;

        % Check if it is actually compatible
        video_title_with_ext = strcat(filename, video_ext);
        try
            v = VideoWriter(video_title_with_ext, video_enc);
            open(v);
        catch
            error("Incompatible video encoder");
        end
        close(v);

        filename = video_title_with_ext;
        encoder  = video_enc;
    end

    function videoObject = animate(A,varargin)
        % method: animate(save_gif)
        %
        % Given the agent's executed trajectory, animate it for the
        % duration given by A.time. The time between animated frames is
        % given by A.animation_time_discretization.
            kwargs = parse_function_args(varargin{:});
            kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
                'default_key', {'save_gif', 'save_video', 'time_interval'}, ...
                'default_value', {false, false, [A.time(1), A.time(end)]});

            save_gif   = kwargs.save_gif; start_gif = save_gif;
            save_video = kwargs.save_video; start_video = save_video;
            time_interval = kwargs.time_interval;

            if save_video
                [video_filename, video_enc] = A.video_setup();
            end

            if save_gif
                gif_filename = A.gif_setup();
            end

            % get timing info
            t_vec = time_interval(1):A.animation_time_discretization:time_interval(end) ;
            frame_rate = A.animation_time_discretization / A.animation_playback_rate ;

            % get axis limits
            lims = A.get_axis_lims() ;

            for t_idx = t_vec
                % create plot
                A.plot_at_time(t_idx)

                if A.animation_set_axes_flag
                    axis equal
                    axis(lims)
                end
                
                if A.animation_set_view_flag
                    view(A.animation_view)
                end

                % create gif
                if save_gif
                    % get current figure
                    fh = get(groot,'CurrentFigure') ;
                    frame = getframe(fh) ;
                    im = frame2im(frame);
                    [imind,cm] = rgb2ind(im,256);

                    if start_gif
                        imwrite(imind,cm,gif_filename,'gif', 'Loopcount',inf,...
                                'DelayTime',frame_rate) ;
                        start_gif = false ;
                    else 
                        imwrite(imind,cm,gif_filename,'gif','WriteMode','append',...
                                'DelayTime',frame_rate) ;
                    end
                else
                    pause(frame_rate)
                end
                
                % create video
                if save_video
                    fh = get(groot, 'CurrentFigure');
                    frame = getframe(fh);
                    im = frame2im(frame); % Convert frame to image
                    resizedIm = imresize(im, [420 560]); % Resize image to 420x560 pixels
                    
                    % Convert resized image back to a frame if necessary
                    frame = im2frame(resizedIm);
                    if start_video
                        videoObject = VideoWriter(video_filename, video_enc);
                        videoObject.FrameRate = 1/frame_rate;
                        open(videoObject);
                        
                        writeVideo(videoObject, frame);
                        
                        start_video = false;
                    else
                        writeVideo(videoObject, frame);
                    end
                else
                    pause(frame_rate);
                end
            end
        end
end
end

