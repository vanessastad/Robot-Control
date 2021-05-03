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
mM = sym('m','real'); % [kg]
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
theta12 = sym('theta12', 'real');
thetae = sym('thetae', 'real');

% Rotation matrix
R12 = [cos(theta12) -sin(theta12); sin(theta12) cos(theta12)];
R12_dot = diff(R12, theta12);

d2 = [d2x; d2z];
%beta = m2 * R12_dot * d2;
beta1 = -m2*(d2x*cos(theta12) + d2z*sin(theta12));
beta2 = -m2*(d2z*cos(theta12) - d2x*sin(theta12));
% g4 = sym('g4', 'real');
g4 = m2*gravity*(d2x*cos(theta12) + d2z*sin(theta12));
%mb = J2 + Jm + m2*norm(d2)^2;
mb = sym('mb', 'real');
%ms = m1 + m2 + mM;
ms = sym('ms', 'real');
% beta1 = sym('beta1', 'real');
% beta2 = sym('beta2', 'real');



g = [0; -ms*gravity; 0; g4];
C = [beta1*theta12_dot^2; beta2*theta12_dot^2; 0; 0];
M = [ms 0 0 beta1; 0 ms 0 beta2; 0 0 J1 0; beta1 beta2 0 mb];
G = [-sin(theta1) 0 0; -cos(theta1) 0 0; dgx 1 -1; 0 0 1];


% Input (3x1)
u = [ut ur tau]'; % [N N*m^2 N*m^2]

% State vector (8x1):
x = [pc1x, pc1z, pc1x_dot, pc1z_dot, theta1, theta1_dot, theta12, theta12_dot]';

% Output (3x1)
% y = [pc1x, pc1z, theta12]'
y = [x(1), x(2), 0, 0, 0, 0, x(7), 0]; 

% State space representation
x1_dot = x(3);
x2_dot = x(4);
x5_dot = x(6);
x7_dot = x(8);

q_ddot = simplify(M\(-C - g));

x3_dot = q_ddot(1);
x4_dot = q_ddot(2);
x6_dot = q_ddot(3);
x8_dot = q_ddot(4);


% Drift field (8x1)
f_x = [x1_dot; x2_dot; x3_dot; x4_dot; x5_dot; x6_dot; x7_dot; x8_dot];

% Control field (8x3)
invMG = simplify(M\G);
g_x = [0 0 0; 0 0 0; invMG(1,:); invMG(2,:); 0 0 0; invMG(3,:); 0 0 0; invMG(4,:)];

% Affine-form system
x_dot = f_x + g_x*u;

%% The system is stricly proper. We derivate y to find u

y1_dot = simplify(jacobian(y(1), x))* x_dot;
y1_ddot = simplify(jacobian(y1_dot, x))* x_dot;

y2_dot = simplify(jacobian(y(2), x))* x_dot;
y2_ddot = simplify(jacobian(y2_dot, x))* x_dot;

y3_dot = simplify(jacobian(y(7), x))* x_dot;
y3_ddot = simplify(jacobian(y3_dot, x))* x_dot;


% tau and ut have to be deleyed
% 
% gammaTmp = - simplify(inv(M)*[C + g]);
% eTmp = simplify(inv(M)*G);
% 
% gamma = [gammaTmp(1:2, :); gammaTmp(4, :)];
% E = [eTmp(1:2, :); eTmp(4, :)];
% detE = det(E)
% E is not invertible

%% It is necessary to insert a delay in tau and ut

% New vector state
z = [x; ut; tau];

% New state space representation
z1_dot = x(3);
z2_dot = x(4);
z5_dot = x(6);
z7_dot = x(8);

% We modify q_ddot because ut and tau are new states
q_new_ddot = simplify(M\(G(:,1)*z(9) + G(:,3)*z(10) - C - g));

w1 = sym('w1', 'real');
w2 = sym('w2', 'real');
w = [w1; w2];


z3_dot = q_new_ddot(1);
z4_dot = q_new_ddot(2);
z6_dot = q_new_ddot(3);
z8_dot = q_new_ddot(4);

% Drift field (10x1)
f_z = [z1_dot; z2_dot; z3_dot; z4_dot; z5_dot; z6_dot; z7_dot; z8_dot; 0; 0];

% Control field (10x1)
g_z = [g_x(:,2); 0; 0];
g_w = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0 ; 1 0; 0 1];


% New Affine-form system
z_dot = f_z + g_z*u(2) + g_w*w;

y1_new = z(1);
y2_new = z(2);
y3_new = z(7);

% We derivate the output looking for input
y1_new_dot = simplify(jacobian(y1_new, z))* z_dot;
y1_new_ddot = simplify(jacobian(y1_new_dot, z))* z_dot;
y1_new_dddot = simplify(jacobian(y1_new_ddot, z))* z_dot;

y2_new_dot = simplify(jacobian(y2_new, z))* z_dot;
y2_new_ddot = simplify(jacobian(y2_new_dot, z))* z_dot;
y2_new_dddot = simplify(jacobian(y2_new_ddot, z))* z_dot;

y3_new_dot = simplify(jacobian(y3_new, z))* z_dot;
y3_new_ddot = simplify(jacobian(y3_new_dot, z))* z_dot;
y3_new_dddot = simplify(jacobian(y3_new_ddot, z))* z_dot;

% We calculate gamma and E
Lfh1 = simplify(jacobian(y1_new, z))*f_z;
L2fh1 = simplify(jacobian(Lfh1, z))*f_z;
L3fh1 = simplify(jacobian(L2fh1, z))*f_z;

Lfh2 = simplify(jacobian(y3_new, z))*f_z;
L2fh2 = simplify(jacobian(Lfh2, z))*f_z;
L3fh2 = simplify(jacobian(L2fh2, z))*f_z;

Lfh3 = simplify(jacobian(y3_new, z))*f_z;
L2fh3 = simplify(jacobian(Lfh3, z))*f_z;
L3fh3 = simplify(jacobian(L2fh3, z))*f_z;

gamma_new = [L3fh1; L3fh2; L3fh3];

E_new = [simplify(jacobian(L2fh1, z))*g_w(:,1) simplify(jacobian(L2fh1, z))*g_z simplify(jacobian(L2fh1, z))*g_w(:,2);
        simplify(jacobian(L2fh2, z))*g_w(:,1) simplify(jacobian(L2fh2, z))*g_z simplify(jacobian(L2fh2, z))*g_w(:,2);
        simplify(jacobian(L2fh3, z))*g_w(:,1) simplify(jacobian(L2fh3, z))*g_z simplify(jacobian(L2fh3, z))*g_w(:,2)];

Det_E_new = det(E_new);  % E_new is not invertible

%% It is necessary to insert a delay in tau and ut

% New vector state
zn = [z; w1; w2];

% New state space representation
z1n_dot = zn(3);
z2n_dot = zn(4);
z5n_dot = zn(6);
z7n_dot = zn(8);

% We modify q_ddot because ut and tau are new states
q_new_ddot = simplify(M\(G(:,1)*zn(9) + G(:,3)*zn(10) -C - g));

w1n = sym('w1n', 'real');
w2n = sym('w2n', 'real');
wn = [w1n; w2n];


z3n_dot = q_new_ddot(1);
z4n_dot = q_new_ddot(2);
z6n_dot = q_new_ddot(3);
z8n_dot = q_new_ddot(4);
z9n_dot = w1;
z10n_dot = w2;

% Drift field (12x1)
fn_z = [z1n_dot; z2n_dot; z3n_dot; z4n_dot; z5n_dot; z6n_dot; z7n_dot; z8n_dot; z9n_dot; z10n_dot; 0; 0];

% Control field (12x1)
gn_z = [g_z; 0; 0];
gn_w = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0 ; 0 0; 0 0; 1 0; 0 1];


% New Affine-form system
zn_dot = fn_z + gn_z*u(2) + gn_w*wn;

% New output
y1_nnew = zn(1);
y2_nnew = zn(2);
y3_nnew = zn(7);

% We derivate the output looking for input
y1_nnew_dot = simplify(jacobian(y1_nnew, zn))* zn_dot;
y1_nnew_ddot = simplify(jacobian(y1_nnew_dot, zn))* zn_dot;
y1_nnew_dddot = simplify(jacobian(y1_nnew_ddot, zn))* zn_dot;
y1_nnew_ddddot = simplify(jacobian(y1_nnew_dddot, zn))* zn_dot;

y2_nnew_dot = simplify(jacobian(y2_nnew, zn))* zn_dot;
y2_nnew_ddot = simplify(jacobian(y2_nnew_dot, zn))* zn_dot;
y2_nnew_dddot = simplify(jacobian(y2_nnew_ddot, zn))* zn_dot;
y2_nnew_ddddot = simplify(jacobian(y2_nnew_dddot, zn))* zn_dot;

y3_nnew_dot = simplify(jacobian(y3_nnew, zn))* zn_dot;
y3_nnew_ddot = simplify(jacobian(y3_nnew_dot, zn))* zn_dot;
y3_nnew_dddot = simplify(jacobian(y3_nnew_ddot, zn))* zn_dot;
y3_nnew_ddddot = simplify(jacobian(y3_nnew_dddot, zn))* zn_dot;

% We calculate gamma and E
Lfh1_new = simplify(jacobian(y1_nnew, zn))*fn_z;
L2fh1_new = simplify(jacobian(Lfh1_new, zn))*fn_z;
L3fh1_new = simplify(jacobian(L2fh1_new, zn))*fn_z;
L4fh1_new = simplify(jacobian(L3fh1_new, zn))*fn_z;

Lfh2_new = simplify(jacobian(y2_nnew, zn))*fn_z;
L2fh2_new = simplify(jacobian(Lfh2_new, zn))*fn_z;
L3fh2_new = simplify(jacobian(L2fh2_new, zn))*fn_z;
L4fh2_new = simplify(jacobian(L3fh2_new, zn))*fn_z;

Lfh3_new = simplify(jacobian(y3_nnew, zn))*fn_z;
L2fh3_new = simplify(jacobian(Lfh3_new, zn))*fn_z;
L3fh3_new = simplify(jacobian(L2fh3_new, zn))*fn_z;
L4fh3_new = simplify(jacobian(L3fh3_new, zn))*fn_z;

gamma_nnew = [L4fh1_new; L4fh2_new; L4fh3_new];

E_nnew = [simplify(simplify(jacobian(L3fh1_new, zn))*gn_w(:,1)) simplify(simplify(jacobian(L3fh1_new, zn))*gn_z) simplify(simplify(jacobian(L3fh1_new, zn))*gn_w(:,2));
          simplify(simplify(jacobian(L3fh2_new, zn))*gn_w(:,1)) simplify(simplify(jacobian(L3fh2_new, zn))*gn_z) simplify(simplify(jacobian(L3fh2_new, zn))*gn_w(:,2));
          simplify(simplify(jacobian(L3fh3_new, zn))*gn_w(:,1)) simplify(simplify(jacobian(L3fh3_new, zn))*gn_z) simplify(simplify(jacobian(L3fh3_new, zn))*gn_w(:,2))];


Det_E_nnew = simplify(det(E_nnew)); % E is invertible if ut!=0 and the system can be completely and exacly dinamically linearized 
% relative degree r = [4, 4, 4]' and total degree r = 12
%%
% It is possible to feedback linearized the system assuming 
v_aux1 = sym('v_aux1','real');
v_aux2 = sym('v_aux2','real');
v_aux3 = sym('v_aux3','real');
v_aux = [v_aux1; v_aux2; v_aux3];

v = simplify(- E_nnew\gamma_nnew + E_nnew\v_aux); % w1n = v(1), ur = v(2), w2n = v(3)

%% We define now the new system

zeta = [y1_nnew; y1_nnew_dot; y1_nnew_ddot; y1_nnew_dddot; y2_nnew; y2_nnew_dot; y2_nnew_ddot; y2_nnew_dddot; y3_nnew; y3_nnew_dot; y3_nnew_ddot; y3_nnew_dddot];
zeta_dot = simplify(jacobian(zeta, zn)*zn_dot);

%% The linearazing input is applied to the new system

lin = simplify(subs(zeta_dot, {w1n ur w2n}, {v(1) v(2) v(3)})); 

% In order to show the correctness of the results we propose the following
% check:

y_dyn = [y1_nnew_dot; y1_nnew_ddot; y1_nnew_dddot; v_aux1; y2_nnew_dot; y2_nnew_ddot; y2_nnew_dddot; v_aux2; y3_nnew_dot; y3_nnew_ddot; y3_nnew_dddot; v_aux3];
simplify(lin - y_dyn);
