%% x_dot

figure('name','Velocity');
% title('$\dot{x}$','Interpreter','latex','FontSize',20)
axis([0 inf 0 7])
hold on
grid on
ylabel('$\dot{x}$ [m/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, q_out_dot_ct.data(:,1), 'Color', '#0E7AC6', 'LineWidth', 2);
plot(q_out_ekf.time, q_out_dot_ekf.data(:,1), 'Color', '#ff33cc', 'LineWidth', 2);
plot(q_out_ls.time, q_out_dot_ls.data(:,1), 'Color', '#C6C00E', 'LineWidth', 2);
plot(ref_v_ct.time, ref_v_ct.data(:,1), 'Color', 'k', 'LineWidth', 2,'LineStyle','--');

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')

% Error

figure('name','Velocity error');
% title('Error $\dot{x}$','Interpreter','latex','FontSize',20)
axis([0 inf -0.5 6])
hold on
grid on
ylabel('$e_{\dot{x}}$ [m/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, ref_v_ct.data(:,1) - q_out_dot_ct.data(:,1), 'Color','#248f8f' , 'LineWidth', 2);
plot(q_out_ekf.time, ref_v_ekf.data(:,1) - q_out_dot_ekf.data(:,1), 'Color', '#FF2700', 'LineWidth', 2);
plot(q_out_ls.time, ref_v_ls.data(:,1) - q_out_dot_ls.data(:,1), 'Color', '#999900', 'LineWidth', 2);

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')


% theta

figure('name','Theta');
% title('${\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf 0.2 1.2]);
elseif selector == 2
    axis([0 inf 0 1.2]);
else
    axis([0 inf 0 0.8]);
end
hold on
grid on
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, q_out_ct.data(:,2), 'Color', '#0E7AC6', 'LineWidth', 2);
plot(q_out_ekf.time, q_out_ekf.data(:,2), 'Color', '#ff33cc', 'LineWidth', 2);
plot(q_out_ls.time, q_out_ls.data(:,2), 'Color', '#C6C00E', 'LineWidth', 2);
plot(ref_t_ct.time, ref_t_ct.data(:,1), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')

% Error

figure('name','Theta error');
% title('Error ${\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.4 0.6]);
elseif selector == 2
    axis([0 inf -0.3 0.7]);
else
    axis([0 inf -0.1 0.4]);
end
hold on
grid on
ylabel('$e_{\theta}$ [rad]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, ref_t_ct.data(:,1) - q_out_ct.data(:,2), 'Color', '#248f8f', 'LineWidth', 2);
plot(q_out_ekf.time, ref_t_ekf.data(:,1) - q_out_ekf.data(:,2), 'Color', '#FF2700', 'LineWidth', 2);
plot(q_out_ls.time, ref_t_ls.data(:,1) - q_out_ls.data(:,2), 'Color','#999900' , 'LineWidth', 2);

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')


% theta_dot


figure('name','Theta_dot');
% title('$\dot{\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.4 0.8]);
elseif selector == 2
    axis([0 inf -0.5 0.6]);
else
    axis([0 inf -0.6 0.4]);
end
hold on
grid on
ylabel('$\dot{\theta}$ [rad/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, q_out_dot_ct.data(:,2), 'Color', '#0E7AC6', 'LineWidth', 2);
plot(q_out_ekf.time, q_out_dot_ekf.data(:,2), 'Color', '#ff33cc', 'LineWidth', 2);
plot(q_out_ls.time, q_out_dot_ls.data(:,2), 'Color', '#C6C00E', 'LineWidth', 2);
plot(ref_td_ct.time, ref_td_ct.data(:,1), 'Color', 'k', 'LineWidth', 2,'LineStyle','--');

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')


% Error

figure('name','Theta_dot error');
% title('Error $\dot{\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.8 0.4]);
elseif selector == 2
    axis([0 inf -0.5 0.6]);
else
    axis([0 inf -0.3 0.6]);
end
hold on
grid on
ylabel('$e_{\dot{\theta}}$ [rad/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out_ct.time, ref_td_ct.data(:,1) - q_out_dot_ct.data(:,2), 'Color', '#248f8f', 'LineWidth', 2);
plot(q_out_ekf.time, ref_td_ekf.data(:,1) - q_out_dot_ekf.data(:,2), 'Color', '#FF2700', 'LineWidth', 2);
plot(q_out_ls.time, ref_td_ls.data(:,1) - q_out_dot_ls.data(:,2), 'Color','#999900', 'LineWidth', 2);

legend('Computed torque','CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex')