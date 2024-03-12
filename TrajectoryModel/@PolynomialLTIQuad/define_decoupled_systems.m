function lti_systems = define_decoupled_systems(obj)
% Parse settings
t_pk = obj.peak_time;
t_f = obj.final_time;
dt = obj.timestep;
R = obj.domain;
t_n = obj.step_number;

lti_systems(1, t_n) = LTISystem;
for i = 1:t_n
    t = -dt + i.*dt;
    A = eye(4);
    if t < t_pk
        alpha_2 = ((t + dt).^2) - t.^2;
        alpha_3 = ((t + dt).^3) - t.^3;
        alpha_4 = ((t + dt).^4) - t.^4;
        A(1, 2) = dt - (alpha_3)./(t_pk.^2) + (alpha_4)./(2.*(t_pk.^3));
        A(1, 3) = (alpha_2./2) - (2.*alpha_3)./(3.*t_pk) + (alpha_4)./(4.*(t_pk.^2));
        A(1, 4) = (alpha_3)./(t_pk.^2) - (alpha_4)./(2.*(t_pk.^3));
    else
        beta_3 = ((t - t_pk + dt).^3) - (t - t_pk).^3;
        beta_4 = ((t - t_pk + dt).^4) - (t - t_pk).^4;
        A(1, 4) = dt + (beta_4./(2.*((t_f - t_pk).^3))) - (beta_3./((t_f - t_pk).^2));
    end
    lti_system = LTISystem('A', A, 'Ts', dt);
    lti_system.setDomain('x', R);

    lti_systems(i) = lti_system;
end

end


