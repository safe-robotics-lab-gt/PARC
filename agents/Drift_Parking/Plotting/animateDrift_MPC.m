function f = animateDrift_MPC(Ux, Uy, r,x,y,yaw,slip_f, slip_r, delta_rad, idx_sync,mpc_states, veh, dT )
% ANIMATEDRIFT animates drift vehicle simulation data
%
% the visualization can be moved forward or backward in time with the arrow keys or a scrollwheel
% animations can be started and paused by pressing 'p'
% videos can be recorded by pressing 'm' to start and stop

% Matt Brown, Vincent Laurense, John Subosits

animation_fps = 50;%50;                     % approximate fps of animation
animation_filename = 'myMovie.avi';     % movie filename
delta_mag_factor = 1;                   % show animated steer angle as larger than modeled

arrow_step = 1;                         % stepsize when arrow keys are pressed
scroll_step = 5;                       % stepsize when mousewheel is scrolled
timer_step = round(1/dT/animation_fps);                         % stepsize when animation is played

body_color = [.8 0 .2];                 % color of body
tire_color = [0 0 0];                   % color of tires

% integrate velocities to E-N
%[posE_m, posN_m, psi_rad] = getGlobalCoordinates(Uy, r, Ux, dT);
posE_m = x;
posN_m = y;
psi_rad = yaw;

f = figure('Name', 'Drifting Animation (Press P to Play)', 'IntegerHandle', 'off', 'WindowScrollWheelFcn', @mousescroll_callback, 'WindowKeyPressFcn', @keypress_callback);
animate_timer = timer('ExecutionMode', 'fixedRate', 'Period', 1/animation_fps);
set(animate_timer, 'TimerFcn', @timer_callback)
recording_video = false;
video_obj = [];

ax_space = subplot(1,1,1);
hold on;

sim.tire_f = cell(2);
sim.tire_r = cell(2);
sim.body                = fill(0,0,veh.body_color);
sim.tire_f{1}              = fill(0,0,veh.tire_color);
sim.tire_f{2}              = fill(0,0,veh.tire_color);
sim.tire_r{1}              = fill(0,0,veh.tire_color);
sim.tire_r{2}              = fill(0,0,veh.tire_color);
sim.ref                 = plot(0,0, 'Color', 'k',        'LineWidth', 2);
sim.traj                = plot(0,0, '--k',      'LineWidth', 2,'DisplayName','actual');
sim.drift_traj_f        = plot(0,0, ':b', 'LineWidth', 4);
sim.drift_traj_r        = plot(0,0, ':b', 'Linewidth', 4);
sim.mpc_traj            = plot(0,0, '--g', 'Linewidth', 3);

curI = 1;
maxI = length(Ux);

xlabel('E [m]')
ylabel('N [m]')
axis equal;
grid on;
box on;
%xticks, yticks was introduced in 2016b! use set(gca,'XTick') for backwards compatibility.
set(gca,'YTick',-1000:5:1000);
set(gca,'XTick',-1000:5:1000);


update_axes();

    function keypress_callback(~, event)
        if strcmp(event.Key, 'rightarrow')
            increment_I(arrow_step);
        end
        if strcmp(event.Key, 'leftarrow')
            decrement_I(arrow_step);
        end
        if strcmp(event.Key, 'p')
            if strcmp(animate_timer.Running, 'on')
                stop(animate_timer);
            else
                start(animate_timer);
            end
        end
        if strcmp(event.Key, 'm')
            if recording_video                          % stop recording
                recording_video = false;
                close(video_obj);
                stop(animate_timer);
            elseif strcmp(animate_timer.Running, 'off') % start recording
                recording_video = true;
                video_obj = VideoWriter(animation_filename);
                video_obj.FrameRate = 30;
                video_obj.Quality = 100;
                open(video_obj);
                start(animate_timer);
            end
        end
        update_axes();
    end

    function mousescroll_callback(~,event)
        if event.VerticalScrollCount > 0
            decrement_I(scroll_step);
        else
            increment_I(scroll_step);
        end
        update_axes();
    end

    function timer_callback(~,~)
        if curI == maxI
            stop(animate_timer);
            if recording_video
                recording_video = false;
                close(video_obj);
                return;
            end
        end
        increment_I(timer_step);
        update_axes()
        if recording_video
            cur_frame = getframe(f);
            try % side effect of global recording_video is extra call to timer_callback before recording_video is updated
                writeVideo(video_obj, cur_frame);
            end
        end
    end

    function increment_I(step)
        curI = curI + step;
        if curI > maxI
            curI = maxI;
        end
    end

    function decrement_I(step)
        curI = curI - step;
        if curI < 1
            curI = 1;
        end
    end


    function update_axes()

        % Plot the path driven so far
        %         plot(posE_m(1:curI), posN_m(1:curI), '--k');


        % Plot the vehicle
        update_vehicle(psi_rad(curI), posE_m(curI), posN_m(curI), ...
            delta_mag_factor*delta_rad(curI), slip_f(curI), slip_r(curI), mpc_states(idx_sync(curI),:,:));

        factor = 2;
        xlim(ax_space, [ posE_m(curI)-factor*5 posE_m(curI)+factor*5 ]);
        ylim(ax_space, [ posN_m(curI)-factor*4 posN_m(curI)+factor*4 ]);
        set(f, 'Name', sprintf('Drifting Animation (Press P to Play), (%d/%d)', curI, maxI));
        drawnow;
    end

    function update_vehicle(psi, posE, posN, delta, slip_f_curr, slip_r_curr, mpc_st)
        a = veh.a;
        b = veh.b;
        rW = .34;       % tire radius

        % body
        R_bd = rotation_matrix_2D(psi);
        V_bd = R_bd*veh.body_vertices + repmat([posE;posN],1,size(veh.body_vertices,2));
        set(sim.body, 'XData', V_bd(1,:), 'YData', V_bd(2,:));
        pos_vert= repmat([posE;posN],1,size(veh.tire_f_vertices,2));
        V_rt = R_bd*veh.tire_r_vertices + pos_vert;
        V_ft = R_bd*veh.tire_f_vertices + pos_vert;

        % tires
        R_t = rotation_matrix_2D(delta);
        for i = 1:2
            f_vert = V_ft(:,6*i-5:6*i-1);
            f_center = repmat( 0.5*(max(f_vert,[],2)+min(f_vert,[],2)),[1,5]);
            origion_vert = f_vert - f_center;
            V_ft(:,6*i-5:6*i-1) = R_t * origion_vert + f_center;
            set(sim.tire_f{i}, 'XData', V_ft(1,6*i-5:6*i-1), 'YData', V_ft(2,6*i-5:6*i-1));
            set(sim.tire_r{i}, 'XData', V_rt(1,6*i-5:6*i-1), 'YData', V_rt(2,6*i-5:6*i-1));
            if slip_f_curr
                set(sim.tire_f{i}, 'FaceColor',tire_color);
            else
                set(sim.tire_f{i}, 'FaceCOlor',veh.tire_color);
            end
            if slip_r_curr
                set(sim.tire_r{i}, 'FaceColor',tire_color);
            else
                set(sim.tire_r{i}, 'FaceCOlor',veh.tire_color);
            end
        end

        % ref point
        ref_rad = .1;
        ang = linspace(0,2*pi,20);
        set(sim.ref, 'Xdata', posE+ref_rad*cos(ang), 'YData', posN+ref_rad*sin(ang));
        set(sim.traj, 'XData', posE_m(1:curI), 'YData', posN_m(1:curI));
        set(sim.mpc_traj, 'XData', mpc_st(1,:,1), 'YData', mpc_st(1,:,2));

    end

    function [posE, posN, psi] = getGlobalCoordinates(Uy, r, Ux, tstep)

        N = numel(r);
        posE = zeros(N,1);
        posN = zeros(N,1);
        psi = zeros(N,1);

        for i = 1:N-1

            dEdt = -Uy(i)*cos(psi(i))-Ux(i)*sin(psi(i));
            dNdt = Ux(i)*cos(psi(i)) - Uy(i)*sin(psi(i));
            dPsidt = r(i);

            posE(i+1) = posE(i) + tstep*dEdt;
            posN(i+1) = posN(i) + tstep*dNdt;
            psi(i+1)  = psi(i) + tstep*dPsidt;

        end

    end

end
