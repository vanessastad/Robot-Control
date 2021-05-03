%% Tavola di osservabilità

% The goal of this project is to analyze the observability of the following not linear
% system that is a rearing up motorcycle:

% m*x'' = tau/R -[*k(1+sin(theta))*x'^2]/2
% I*theta'' + b*theta' +m*g*L*cos(theta)/2 = tau + [k*(1+sin(theta)*(x'^2)*L*sin(theta)]/4

% Considering the action of rearing up a motorcycle, we will assume:
% x_dot > 0
% 0 < theta < pi/2

clear all
close all
clc

%% Symbolic variables definition

% state 
x_dot = sym('x_dot','real'); 
theta = sym('theta','real'); % angle between the horizontal axis and the motorcycle's axis
theta_dot = sym('theta_dot','real');

% Parameter
m = sym('m','real'); % [kg]
R = sym('R','real'); % [m]
K = sym('K','real'); % [Kg/m]
I = sym('I','real'); % [Kg*m^2]
b = sym('b','real'); % [N*m*s/rad]
gravity = sym('gravity','real'); % [m/s^2]
L = sym('L','real'); % [m]

% Input
tau = sym('tau','real'); % [N*m^2]

% State vector (3x1):
q = [x_dot, theta, theta_dot];

% Output
h = L*sin(theta);

% State space representation
q1_dot = (1/m)*(tau/R - (K/2)*(1 + sin(q(2)))*q(1)^2);
q2_dot = q(3);
q3_dot = (1/I)*(tau + (L/4)*(q(1)^2)*K*(1 + sin(q(2))*sin(q(2)) - b*q(3) - m*gravity*(L/2)*cos(q(2))));

% Drift field (3x1)
f = [-(1/m)*(K/2)*(1 + sin(q(2)))*q(1)^2; q(3); (1/I)*(((L/4)*(q(1)^2)*K*(1 + sin(q(2)))*sin(q(2))) - b*q(3) - m*gravity*(L/2)*cos(q(2)))];

% Control field (3x1)
g = [1/(m*R); 0; 1/I];

%% Observability analysis (analytic aproach) 

% Derivative output vector (3*1): 
dh = simplify(jacobian(h, q));
dLfh = dir_der(f, dh, q);
dLgh = dir_der(g, dh, q);
dLgLfh = dir_der(g, dLfh, q);
dLfh2 = dir_der(f, dLfh, q);
dLfh3 = dir_der(f, dLfh2, q);

% Observability codistribution(6x3)
O = [dh; dLfh; dLgh; dLgLfh; dLfh2; dLfh3];
rankO = rank(O);


%% Observability analysis (geometric aproach) 
dim = 3; 
Omega0 = [0; 1; 0]; %span(dh)
[n, k, Omega_end] = checkdim( Omega0, O, dim);

%% Study of observability

% Observability codistribution(3x3)
O_det = [O(1:2,:)' O(5,:)']';
d = det(O_det);
solx_dot = subs(d, x_dot, 0); % This case will not occure beause we have assumed x_dot > 0
soltheta =  subs(d, theta, pi/2);  % This case will not occure beause we have assumed theta > 0
soltheta_dot = simplify(subs(d, theta_dot, (1/b)*[2 - cos(theta)*(cos(theta)+L*gravity*m/2)])); % Under this condition the system lose observability
%% Plot
% 
% %   Assuming: m = 160 kg
% %             I = 235.2 Kg*m^2
% %             R = 0.6 m
% %             L = 2.1 m
% %             b = 0.2 N*m*s/rad
% %             K = 40.248 Kg/m
% %             gravity = 9.81 m/s^2
% 
% d_num = subs(d, {m I R L b K gravity}, {160 235.2 0.6 2.1 0.2 40.248 9.81});
% 
% sample = 30;
% 
% % We take 30 samples in the interval
% x = linspace(0.1, 6, sample); % x_dot
% y = linspace(0.1, pi/2, sample); % theta
% 
% % Built matrix
% [X, Y] = meshgrid(x, y);
% 
% % initialization of the unknown variable Z (theta_dot) knowing x_dot e theta
% Z = sym('Z', [sample, sample]);
% 
% % Determinant of Observability Covariance
% fz = -simplify((6656013.*X.*cos(Y).^3.*((2.*Z)/5 + (82404.*cos(Y))/25 + 2.*cos(Y).^2 - 4))/8000000);
% 
% % Solution of fz = 0 to find Z
% for i = 1 : sample
%     for j = 1 : sample
%         fzij = fz(i, j) == 0;
%         Z(i, j) = solve(fzij);
%     end
% end
% 
% % Necessary to plot the graph
% Z = double(Z);
% 
% figure 
% surf(X, Y, Z)
% xlabel('$\dot{x}$','Interpreter','latex')
% ylabel('$\theta$','Interpreter','latex')
% zlabel('$\dot{\theta}$','Interpreter','latex')
% colormap(autumn)
% shading interp 
% % colorbar;
%% Parametric observability

% Assuming R, m, L and g known, we want now to estimate the parameters b, k and I that for the physic of the system have to be positive. 
% It's necessary to introduce some modifications:

% New vector state (7x1)
q_new = [q m I b K];

% New Drift field (7x1)
f_new = [-(1/m)*(K/2)*(1 + sin(q_new(2)))*q_new(1)^2; q_new(3); (1/I)*(((L/4)*(q_new(1)^2)*K*(1 + sin(q_new(2)))*sin(q_new(2))) - b*q_new(3) - m*gravity*(L/2)*cos(q_new(2))); 0; 0; 0; 0];
% New Control field (7x1)
g_new = [1/(m*R); 0; 1/q_new(6); 0; 0; 0; 0];



%% Observability analysis (analytic aproach) 

% Derivative output vector (7x1): 
dh_new = simplify(jacobian(h, q_new));

dLfh_new = dir_der(f_new, dh_new, q_new);
dLgh_new = dir_der(g_new, dh_new, q_new);

dLgLfh_new = dir_der(g_new, dLfh_new, q_new);
dLfh2_new = dir_der(f_new, dLfh_new, q_new);

dLfh3_new = dir_der(f_new, dLfh2_new, q_new);
dLgLfh2_new = dir_der(g_new, dLfh2_new, q_new);
dLfLgfh_new = dir_der(f_new, dLgLfh_new, q_new); % nulla
dLgLgfh_new = dir_der(g_new, dLgLfh_new, q_new);

dLfh4_new = dir_der(f_new, dLfh3_new, q_new);
dLgLfh3_new = dir_der(g_new, dLfh3_new, q_new);
dLfLgfh2_new = dir_der(f_new, dLgLfh2_new, q_new);
dLgLgfh2_new = dir_der(g_new, dLgLfh2_new, q_new);



% Observability codistribution(8x7)
O_new = [dh_new; dLfh_new; dLgLfh_new; dLfh2_new; dLfh3_new; dLgLfh2_new; dLfh4_new; dLgLfh3_new];
rankO = rank(O_new);

%% Observability analysis (geometric aproach) 

Omega0_new = [0; 1; 0; 0; 0; 0; 0]; %span(dh)
[n, k, Omega_end] = checkdim(Omega0_new, O_new, 7);

%% Study of observability

% Observability codistribution(7x7)
O_det_new = [O_new(1:7,:)]'; % O_new(4:7,:)']';
d_new = det(O_det_new); % If the parameter I'd be equal to 0 the equation'd be not defined but this case is excluded for the system physic.

% Parameter Analysis
solx_dot_new = subs(d_new, x_dot, 0);  % This case will not occure beause we have assumed x_dot > 0
soltheta_new =  subs(d_new, theta, pi/2);  % This case will not occure beause we have assumed theta > pi/2
soltheta_dot_new = simplify(subs(d_new, theta_dot, (1/b)*[2 - cos(theta)*(cos(theta)+L*gravity*m/2)])); % Under this condition the system lose observability
solK = subs(d_new, K, 0);  % This case will not occure beause we have assumed K > 0
solb = simplify(subs(d_new, b, -(2*cos(theta)^2 + L*gravity*m*cos(theta) - 4)/(2*theta_dot))); % Under this condition the system lose observability


%% We want now to analyze the controllability of the system (Chow theorem)

delta0 = g_new;
dLfg1 = lie_bracket(f_new, g_new, q_new);
delta1 = [delta0 dLfg1];

dLfg2 = lie_bracket(f_new, dLfg1, q_new);
delta2 = [delta1 dLfg2];

dLfg3 = lie_bracket(f_new, dLfg2, q_new);
delta3 = [delta2 dLfg3];

rank(delta3); % the filtration is stopped because rank(delta3) = rank(delta2); b, K and I are not controllable
%%



