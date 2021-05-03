%% Feedback linearization
% We consider the model of an UAV with an arm

clear all
close all
clc
%% Symbolic variables definition

% State variables
pc1x = sym('pc1x','real'); 
pc1z = sym('pc1z','real'); 
pc1x_dot = sym('pc1x_dot','real');
pc1z_dot = sym('pc1z_dot','real');
theta1_dot = sym('theta1_dot','real');
theta1 = sym('theta1','real');
theta2 = sym('theta2','real');
thetam = sym('thetam','real');
theta12_dot = sym('theta12_dot','real');


% Parameter
m1 = sym('m1','real'); % [kg]
m2 = sym('m2','real'); % [kg]
mM = sym('mM','real'); % [kg]
J1 = sym('J1', 'real'); % [Kg*m^2]
J2 = sym('J2', 'real'); % [Kg*m^2]
Jm = sym('Jm', 'real'); % [Kg*m^2]
d2x = sym('d2x', 'real'); % [m]
d2z = sym('d2z', 'real'); % [m]
dgx = sym('dgx', 'real'); % [m]
dgz = sym('dgz', 'real'); % [m]
gravity = sym('gravity','real'); % [m/s^2]
tau = sym('tau','real'); % [N*m^2]
ut = sym('ut','real'); % [N]
ur = sym('ur','real'); % [N*m^2]

%theta12 = theta1 + theta2;
theta12 = sym('theta12', 'real');

%thetae = theta2 - thetam;
thetae = sym('thetae', 'real');

% Rotation matrix
R12 = [cos(theta12) -sin(theta12); sin(theta12) cos(theta12)];
R12_dot = diff(R12, theta12);

d2 = [d2x; d2z];
mb = J2 + Jm + m2*norm(d2)^2;
ms = m1 + m2 + mM;
beta1 = -m2*(d2x*cos(theta12) + d2z*sin(theta12));
beta2 = -m2*(d2z*cos(theta12) - d2x*sin(theta12));
beta = m2 * R12_dot * d2;
g4 = m2*gravity*(d2x*cos(theta12) + d2z*sin(theta12));

% Lagrangian matrices
g = [0; -ms*gravity; 0; g4];
C = [beta1*theta12_dot^2; beta2*theta12_dot^2; 0; 0];
M = [ms 0 0 beta(1); 0 ms 0 beta(2); 0 0 J1 0; beta(1) beta(2) 0 mb];
G = [-sin(theta1) 0 0; -cos(theta1) 0 0; dgx 1 -1; 0 0 1];

% Input (3x1)
u = [ut ur tau]'; % [N*m^2]

% State vector (8x1):
x = [pc1x, pc1z, pc1x_dot, pc1z_dot, theta1, theta1_dot, theta12, theta12_dot]';
assumeAlso(x, 'real');

% Output (3x1)
y = [x(1); x(2); x(7)];       % y = [pc1x, pc1z, theta12]'

% State space representation
x1_dot = x(3);
x2_dot = x(4);
x5_dot = x(6);
x7_dot = x(8);

q_ddot = simplify((inv(M))*(-C - g)); % It allows us to find state variables easily
x3_dot = q_ddot(1);
x4_dot = q_ddot(2);
x6_dot = q_ddot(3);
x8_dot = q_ddot(4);

% Drift field (8x1)
f_x = [x1_dot; x2_dot; x3_dot; x4_dot; x5_dot; x6_dot; x7_dot; x8_dot]; % It contains everything is not an input

% Control field (8x3)
invMG = simplify((inv(M))*G);
g_x = [0 0 0; 0 0 0; invMG(1,:); invMG(2,:); 0 0 0; invMG(3,:); 0 0 0; invMG(4,:)];

% Affine-form system
x_dot = f_x + g_x*u;

%% Equilibrium points

theta1_eq = 0;
theta1_dot_eq = 0;
theta12_eq = 0;
theta12_dot_eq = 0;
pc1x_dot_eq = 0;
pc1z_dot_eq = 0;

% Every pc1x, pc1z is an equilibrium point and this is coherent with the
% nature of UAV. 

ut_eq = (m1 + m2 + mM)*gravity; % This equilibrium doesn't depend on ut, tau and ur so, 
%it will be necessary a traslation of our system in order to compensate this contribute.
tau_eq = simplify((m2*(d2z*ut_eq*sin(theta1_eq + theta12_eq) - d2x*ut_eq*cos(theta1_eq + theta12_eq) + 2*d2x*gravity*m1*cos(theta12_eq) + 2*d2x*gravity*m2*cos(theta12_eq) + 2*d2x*gravity*mM*cos(theta12_eq)))/(m1 + m2 + mM));
ur_eq = simplify(tau_eq - dgx*ut_eq);

u_eq = [ut_eq; ur_eq; tau_eq];

%% Numerical substitutions

m1_num = 1.309; % [kg]
m2_num = 0.098; % [kg]
mM_num = 0.06; % [kg]
J1_num = 0.0154; % [Kg*m^2]
J2_num = 0.0011; % [Kg*m^2]
Jm_num = 0.4101; % [Kg*m^2]
d2x_num = 0; % [m]
d2z_num = 0.0979; % [m]
dgx_num = 0; % [m]
dgz_num = 0.0081; % [m]
gravity_num = 9.81; % [m/s^2]

x_dot_num = (subs(x_dot, {m1 m2 mM J1 J2 Jm d2x d2z dgx dgz gravity}, {m1_num m2_num mM_num J1_num J2_num Jm_num d2x_num d2z_num dgx_num dgz_num gravity_num}));
u_eq_num = subs(u_eq, {gravity m1 m2 mM dgx d2x}, {gravity_num m1_num m2_num mM_num dgx_num d2x_num});

%% Linear approximation

A_in = jacobian(x_dot_num, x);
A = double(subs(A_in, {theta1 theta1_dot theta12 theta12_dot pc1x_dot pc1z_dot ut ur tau}, {theta1_eq theta1_dot_eq theta12_eq theta12_dot_eq pc1x_dot_eq pc1z_dot_eq u_eq_num(1) u_eq_num(2) u_eq_num(3)}));

B_in = jacobian(x_dot_num, u);
B = double(subs(B_in, {theta1 theta1_dot theta12 theta12_dot pc1x_dot pc1z_dot ut ur tau}, {theta1_eq theta1_dot_eq theta12_eq theta12_dot_eq pc1x_dot_eq pc1z_dot_eq u_eq_num(1) u_eq_num(2) u_eq_num(3)}));

% The new system is x_tilde = (x - x_eq) where x_eq = 0 and with input u_tilde
x_tilde = x; 
u_tilde = u - u_eq_num;

% Dynamic of the linearized system
x_dot_lin_num = A*x_tilde + B*u_tilde;

%% We want now to find a controller for our system

poles = [-2 -2 -1 -1 -4 -3 -4 -3];
K = place(A, B, poles);

App = A - B*K; % The minus is due to matlab notation of place command
upp = -K*x + u_eq_num;  %  <-- u = upp, u_tilde = -K*x <-- u = u_tilde + u_eq_num <-- u_tilde = u - u_eq_num;

%% We want now to study the RAS of our linearization in order to understand if it is a good approximation
% In order to do this we sobtitute the found input (upp) into the original system (x_dot_num)
% into the linearization (x_dot_lin_num_tilde) and then we calculate the residual error between those two (f_tilde)

x_dot_num_tilde = simplify(subs(x_dot_num, {ut ur tau}, {upp(1) upp(2) upp(3)}));
x_dot_lin_num_tilde = simplify(subs(x_dot_lin_num, {ut ur tau}, {upp(1) upp(2) upp(3)}));

f_tilde = x_dot_num_tilde - x_dot_lin_num_tilde;

% In order to find a Lyapunov candidate:
Q = eye(8);
P = lyap(App', Q);
% In order to prevent numerical errors
P = round(P,3);

% This function estimate the RAS
evalvdot(P, Q, f_tilde);

