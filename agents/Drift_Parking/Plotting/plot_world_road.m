function plot_world_road(ax_space, xlim, ylim)
left = xlim(1); right = xlim(2);
up = ylim(2); down = ylim(1);
lw = 2;
% draw road boundary
fill(ax_space,[left,right,right,left,left],...
    [down,down,up,up,down],...
    [207, 207, 207]/255);

drive_up = up-2; drive_down = down+2;
mid_up = drive_down + 14+0.6; mid_down = drive_down + 14;

% road look 1

fill(ax_space,[left,right,right,left,left],...
    [down,down,drive_down,drive_down,down],...
    [150, 150, 150]/255);

% plot([left,right],[drive_up, drive_up],'LineWidth',lw*2,'Color','w')
% plot([left,right],[mid_up+7, mid_up+7],'LineWidth',lw,'Color','w')
plot([left,right],[mid_up+5, mid_up+5],'--','LineWidth',lw*2,'Color','w')

plot([left,right],[mid_up, mid_up],'--','LineWidth',lw,'Color','y')
% plot([left,right],[mid_down, mid_down],'LineWidth',lw,'Color','w')
plot([left,right],[mid_down, mid_down],'LineWidth',lw,'Color','y')

plot([left,right],[drive_down+9, drive_down+9],'--','LineWidth',lw*2,'Color','w')
plot([left,right],[drive_down+4, drive_down+4],'LineWidth',lw,'Color','w')
plot([left,right],[drive_down, drive_down],'LineWidth',lw*2,'Color','w')

% road look 2

% plot([left,right],[drive_up, drive_up],'LineWidth',lw*2,'Color','w')
% plot([left,right],[(mid_up+drive_up)/2, (mid_up+drive_up)/2],'--','LineWidth',lw*2,'Color','w')
% 
% plot([left,right],[mid_up, mid_up],'--','LineWidth',lw,'Color','y')
% % plot([left,right],[mid_down, mid_down],'LineWidth',lw,'Color','w')
% plot([left,right],[mid_down, mid_down],'LineWidth',lw,'Color','y')
% 
% plot([left,right],[(mid_down+drive_down)/2, (mid_down+drive_down)/2],'--','LineWidth',lw*2,'Color','w')
% plot([left,right],[drive_down, drive_down],'LineWidth',lw*2,'Color','w')
end