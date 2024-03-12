function lti_systems = define_systems(obj)
% DEPRECIATED

% Parse settings
t_pk = obj.peak_time;
t_f = obj.final_time;
dt = obj.timestep;
R = obj.domain;
t_n = obj.step_number;

lti_systems(1, t_n) = LTISystem;
i = 0;

for t = 0:dt:(t_f - dt)
    i = i + 1;
    if t < t_pk
        a_k_v = (1 - (3.*(t.^2))./(t_pk.^2) + (2.*(t.^3))./(t_pk.^3)).*dt;
        a_k_a = (t - (2.*(t.^2))./(t_pk) + (t.^3)./(t_pk.^2)).*dt;
        a_k_pk = ((3.*(t.^2))./(t_pk.^2) - (2.*(t.^3))./(t_pk.^3)).*dt;
        A = eye(12);
        A(1:3, 4:6) = eye(3).*a_k_v;
        A(1:3, 7:9) = eye(3).*a_k_a;
        A(1:3, 10:12) = eye(3).*a_k_pk;
        
        lti_system = LTISystem('A', A, 'Ts', dt);
        lti_system.setDomain('x', R);

        lti_systems(i) = lti_system;
    else
        a_k_pk = ((2.*(t.^3))./((t_f - t_pk).^3) - (3.*(t.^2))./((t_f - t_pk).^2)).*dt;
        A = eye(12);
        A(1:3, 4:6) = eye(3).*dt;
        A(1:3, 7:9) = eye(3).*t.*dt;
        A(1:3, 10:12) = eye(3).*a_k_pk;

        lti_system = LTISystem('A', A, 'Ts', dt);
        lti_system.setDomain('x', R);

        lti_systems(i) = lti_system;
    end
end

