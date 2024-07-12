Fx = 516.25; % N
alp = -pi/10:pi/1000:pi/10;
Fy = zeros(1,length(alp));
Fy_tanh = zeros(1,length(alp));
for idx = 1:length(alp)
    Fy(idx) = fiala_model(alp(idx), tire_r, Fx);
    Fy_tanh(idx) = fiala_tanh(alp(idx), tire_r, Fx);
end
plot(alp, Fy,'r--', 'linewidth', 2)
hold on 
plot(alp, Fy_tanh, 'linewidth', 1.5)
legend('original fiala', 'tanh fiala')
hold off