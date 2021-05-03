%% m

figure('name','Mass');
%     title('Mass','Interpreter','latex','FontSize',20)
axis([0 inf 0 161]);
hold on
grid on
ylabel('$m$ [kg]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(parameters_ekf.time, parameters_ekf.data(:,1), 'Color', '#1980e6', 'LineWidth', 2);
plot(parameters_ls.time, parameters_ls.data(:,1), 'Color', '#80f20d', 'LineWidth', 2);
plot(parameters_ekf.time, m*ones(1, length(parameters_ekf.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex','Location','southeast')


%% I

figure('name','Inertia');
%     title('Inertia','Interpreter','latex','FontSize',20)
axis([0 inf -7 250]);
hold on
grid on
ylabel('$I$ [kg$\cdot$m$^{2}$]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(parameters_ekf.time, parameters_ekf.data(:,2), 'Color', '#1980e6', 'LineWidth', 2);
plot(parameters_ls.time, parameters_ls.data(:,2), 'Color', '#80f20d', 'LineWidth', 2);
plot(parameters_ekf.time, I*ones(1, length(parameters_ekf.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex','Location','southeast')


%% b

figure('name','b');
%     title('${b}$','Interpreter','latex','FontSize',20)
axis([0 inf 0 25]);
hold on
grid on
ylabel('$b$ [N$\cdot$m$\cdot$s/rad]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(parameters_ekf.time, parameters_ekf.data(:,3), 'Color', '#1980e6', 'LineWidth', 2);
plot(parameters_ls.time, parameters_ls.data(:,3), 'Color', '#80f20d', 'LineWidth', 2);
plot(parameters_ekf.time, b*ones(1, length(parameters_ekf.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex','Location','northeast')


%% K

figure('name','K');
%     title('${K}$','Interpreter','latex','FontSize',20)
axis([0 inf -10 45]);
hold on
grid on
ylabel('$K$ [kg/m]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(parameters_ekf.time, parameters_ekf.data(:,4), 'Color', '#1980e6', 'LineWidth', 2);
plot(parameters_ls.time, parameters_ls.data(:,4), 'Color', '#80f20d', 'LineWidth', 2);
plot(parameters_ekf.time, K*ones(1, length(parameters_ekf.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('CT + EKF','Li-Slotine', 'color', 'w','Interpreter','latex','Location','southeast')