function [point_found, t_span, states, states_dot, inputs, fiala_tire_vars] = find_switching_point(A,beta_switch)
% return trimmed trajectories before desired drifting 
point_found = false;
t_span = [];
states = [];
states_dot = [];
inputs = [];
fiala_tire_vars = [];

% fiala_drift_margin_front_rear = A.fiala_tire_vars(:,3:4) - A.fiala_tire_vars(:,1:2);
% rear_slip_bool = fiala_drift_margin_front_rear(:,2) < 0;
% filter_length = 10;
% for idx = 1:length(rear_slip_bool)-filter_length+1
%     if all(rear_slip_bool(idx:idx+filter_length-1))
%         switch_idx = idx+filter_length-1;
%         point_found = true;
%         switch_idx = switch_idx + 1;
%         t_span = A.t_s(1:switch_idx);
%         states = A.states(1:switch_idx,:);
%         states_dot = A.states_dot(1:switch_idx,:);
%         inputs = A.inputs(1:switch_idx,:);
%         fiala_tire_vars = A.fiala_tire_vars(1:switch_idx,:);
%         break
%     end
% end

beta_rad = A.states(:,6);
filter_length = 10;
beta_switch_bool = beta_rad > beta_switch;
for idx = 1:length(beta_switch_bool)-filter_length+1
    if all(beta_switch_bool(idx:idx+filter_length-1))
        switch_idx = idx;
        point_found = true;
        switch_idx = switch_idx + 1;
        t_span = A.t_s(1:switch_idx);
        states = A.states(1:switch_idx,:);
        states_dot = A.states_dot(1:switch_idx,:);
        inputs = A.inputs(1:switch_idx,:);
        fiala_tire_vars = A.fiala_tire_vars(1:switch_idx,:);
        break
    end
end

end