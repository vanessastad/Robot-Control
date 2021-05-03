%% Graph

% In this script we graph the results obtained with the simulink model

%% x_dot

figure('name','Velocity');
% title('$\dot{x}$','Interpreter','latex','FontSize',20)
axis([0 inf 0 6])
hold on
grid on
ylabel('$\dot{x}$ [m/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, q_out_dot.data(:,1), 'Color', '#0E7AC6', 'LineWidth', 2);
plot(ref_v.time, ref_v.data(:,1), 'Color', 'k', 'LineWidth', 2,'LineStyle','--');

legend('$\dot{x}$','$\dot{x}_{ref}$', 'color', 'w','Interpreter','latex')

% Error

figure('name','Velocity error');
% title('Error $\dot{x}$','Interpreter','latex','FontSize',20)
axis([0 inf -0.5 6])
hold on
grid on
ylabel('$e_{\dot{x}}$ [m/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, ref_v.data(:,1) - q_out_dot.data(:,1), 'Color', '#FF2700', 'LineWidth', 2);

% legend('Error $\dot{x}$','color','w','Interpreter','latex')


%% theta

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

plot(q_out.time, q_out.data(:,2), 'Color', '#19CE36', 'LineWidth', 2);
plot(ref_t.time, ref_t.data(:,1), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

legend('${\theta}$','${\theta}_{ref}$', 'color', 'w','Interpreter','latex')

% Error

figure('name','Theta error');
% title('Error ${\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.4 0.6]);
elseif selector == 2
    axis([0 inf -0.3 0.7]);
else
    axis([0 inf -0.1 0.8]);
end
hold on
grid on
ylabel('$e_{\theta}$ [rad]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, ref_t.data(:,1) - q_out.data(:,2), 'Color', '#FF2700', 'LineWidth', 2);

% legend('Error ${\theta}$','color','w','Interpreter','latex')


%% theta_dot


figure('name','Theta_dot');
% title('$\dot{\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.4 0.8]);
elseif selector == 2
    axis([0 inf -0.5 0.6]);
else
    axis([0 inf -1 0.6]);
end
hold on
grid on
ylabel('$\dot{\theta}$ [rad/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, q_out_dot.data(:,2), 'Color', '#C6C00E', 'LineWidth', 2);
plot(ref_td.time, ref_td.data(:,1), 'Color', 'k', 'LineWidth', 2,'LineStyle','--');

legend('$\dot{\theta}$','$\dot{\theta}_{ref}$', 'color', 'w','Interpreter','latex')


% Error

figure('name','Theta_dot error');
% title('Error $\dot{\theta}$','Interpreter','latex','FontSize',20)
if selector == 1
    axis([0 inf -0.8 0.4]);
elseif selector == 2
    axis([0 inf -0.5 0.6]);
else
    axis([0 inf -0.6 1]);
end
hold on
grid on
ylabel('$e_{\dot{\theta}}$ [rad/s]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, ref_td.data(:,1) - q_out_dot.data(:,2), 'Color', '#FF2700', 'LineWidth', 2);

% legend('Error $\dot{\theta}$','color','w','Interpreter','latex')
%
if model ~= 1
    %% m

    figure('name','Mass');
%     title('Mass','Interpreter','latex','FontSize',20)
    if model == 2
        axis([0 inf 157 161]);
    else
        axis([0 inf 0 160]);
    end
    hold on
    grid on
    ylabel('$m$ [kg]','Interpreter','latex','FontSize',18)
    xlabel('time [s]','Interpreter','latex','FontSize',18)

    plot(parameters.time, parameters.data(:,1), 'Color', '#ff33cc', 'LineWidth', 2);
    plot(parameters.time, m*ones(1, length(parameters.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

    legend('$m$','$m_{ref}$', 'color', 'w','Interpreter','latex')


    %% I

    figure('name','Inertia');
%     title('Inertia','Interpreter','latex','FontSize',20)
    if model == 2
        axis([0 inf 234 239]);
    else
        axis([0 inf -10 250]);
    end
    hold on
    grid on
    ylabel('$I$ [kg$\cdot$m$^{2}$]','Interpreter','latex','FontSize',18)
    xlabel('time [s]','Interpreter','latex','FontSize',18)

    plot(parameters.time, parameters.data(:,2), 'Color', '#999900', 'LineWidth', 2);
    plot(parameters.time, I*ones(1, length(parameters.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');
    
    legend('$I$','$I_{ref}$', 'color', 'w','Interpreter','latex')


    %% b

    figure('name','b');
%     title('${b}$','Interpreter','latex','FontSize',20)
    if model == 2
        axis([0 inf 0.14 0.21]);
    else
        axis([0 inf 0 25]);
    end
    hold on
    grid on
    ylabel('$b$ [N$\cdot$m$\cdot$s/rad]','Interpreter','latex','FontSize',18)
    xlabel('time [s]','Interpreter','latex','FontSize',18)

    plot(parameters.time, parameters.data(:,3), 'Color', '#b30000', 'LineWidth', 2);
    plot(parameters.time, b*ones(1, length(parameters.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

    legend('${b}$','$b_{ref}$', 'color', 'w','Interpreter','latex')


    %% K

    figure('name','K');
%     title('${K}$','Interpreter','latex','FontSize',20)
    if model == 2
        axis([0 inf 35 42]);
    else
        axis([0 inf -10 49]);
    end
    hold on
    grid on
    ylabel('$K$ [kg/m]','Interpreter','latex','FontSize',18)
    xlabel('time [s]','Interpreter','latex','FontSize',18)

    plot(parameters.time, parameters.data(:,4), 'Color', '#248f8f', 'LineWidth', 2);
    plot(parameters.time, K*ones(1, length(parameters.data(:,1))), 'Color', 'k', 'LineWidth',2,'LineStyle','--');

    legend('${K}$','$K_{ref}$', 'color', 'w','Interpreter','latex')
end