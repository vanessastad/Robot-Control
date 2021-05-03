%% Graph

% In this script we graph the results obtained with the simulink model

%% x

% figure('name','x');
% % title('$x$','Interpreter','latex','FontSize',20)
% if selector == 3
%     axis([0 inf -50 50]) 
% else
%     axis([0 inf -25 25])
% end
% hold on
% grid on
% ylabel('$x$ [m]','Interpreter','latex','FontSize',18)
% xlabel('time [s]','Interpreter','latex','FontSize',18)
% 
% plot(q_out.time, q_out.data(:,1), 'Color', '#0E7AC6', 'LineWidth', 2);
% plot(ref.time, ref.data(:,1), 'Color', 'k', 'LineWidth', 2,'LineStyle','--');
% 
% legend('$x$','${x}_{ref}$', 'color', 'w','Interpreter','latex')

% Error

figure('name','x error');
% title('Error $x$','Interpreter','latex','FontSize',20)
if selector == 3
    axis([0 inf -10 50]) 
else
    axis([0 inf -5 25])
end
hold on
grid on
ylabel('$e_{x}$ [m]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, ref.data(:,1) - q_out.data(:,1), 'Color', '#FF2700', 'LineWidth', 2);

% legend('Error $x$','color','w','Interpreter','latex')


%% z

% figure('name','z');
% % title('$z$','Interpreter','latex','FontSize',20)
% if selector == 3
%     axis([0 inf -50 50]) 
% else
%     axis([0 inf -25 25])
% end
% hold on
% grid on
% ylabel('$z$ [m]','Interpreter','latex','FontSize',18)
% xlabel('time [s]','Interpreter','latex','FontSize',18)
% 
% plot(q_out.time, q_out.data(:,2), 'Color', '#19CE36', 'LineWidth', 2);
% plot(ref.time, ref.data(:,6), 'Color', 'k', 'LineWidth',2,'LineStyle','--');
% 
% legend('$z$','${z}_{ref}$', 'color', 'w','Interpreter','latex')

% Error

figure('name','z error');
% title('Error $z$','Interpreter','latex','FontSize',20)
if selector == 3
    axis([0 inf -10 50]) 
else
    axis([0 inf -5 25])
end
hold on
grid on
ylabel('$e_{z}$ [m]','Interpreter','latex','FontSize',18)
xlabel('time [s]','Interpreter','latex','FontSize',18)

plot(q_out.time, ref.data(:,6) - q_out.data(:,2), 'Color', '#FF2700', 'LineWidth', 2);

% legend('Error $z$','color','w','Interpreter','latex')


