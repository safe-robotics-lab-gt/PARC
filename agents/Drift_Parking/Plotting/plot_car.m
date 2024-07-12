function plot_car(ax, pos, theta, delta, veh, alpha)
%% plot car
hold(ax, "on");
% body
R_bd = rotation_matrix_2D(theta);
posE = pos(1);
posN = pos(2);
V_bd = R_bd*veh.body_vertices + repmat([posE;posN],1,size(veh.body_vertices,2));
bd = fill(ax,V_bd(1,:),V_bd(2,:),veh.body_color,'LineStyle','none');
bd.FaceAlpha = alpha;
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
    ft = fill(ax,V_ft(1,6*i-5:6*i-1), V_ft(2,6*i-5:6*i-1),veh.tire_color, 'LineStyle','none');
    rt = fill(ax,V_rt(1,6*i-5:6*i-1), V_rt(2,6*i-5:6*i-1),veh.tire_color, 'LineStyle','none');
    ft.FaceAlpha = alpha;
    rt.FaceAlpha = alpha;
    plot(ax,posE,posN,'.','MarkerSize',10,'Color','k')
end
% hold(ax,"off")