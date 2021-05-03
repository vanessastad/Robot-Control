%% Main

clear all
close all
clc

%% Data

m = 160; % [kg]
I = 235.2; % [kg*m^2]
R = 0.6; % [m]
L = 2.1; % [m]
b = 0.2; % [N*m*s/rad]
K = 40.248; % [kg/m]
gravity = 9.81; % [m/s^2]

lambda = 5; % Li-Slotine gain
Ath = 0.5; % Amplitude [rad]
wth = 0.2; % Pulsation [rad/s]

selector = select(1);

% model = select(2);

%% Li-Slotine control

% Steady conditions
theta = pi/4; % [rad]
x_ddot_r = 0; % [m/s^2]
theta_ddot_r = 0; % [rad/s^2]
x_dot_r = 5; % [m/s]
x_dot = 5; % [m/s]
theta_dot_r = 0; % [rad/s]

% Assuming to introduce D:
% M(q)*ddq_r + C(q, dq)*dq_r + G(q) = D(q, dq, dq_r, ddq_r, pk, pd)
%
% It's possible to organize the dynamic parameters as:
% D(q, dq, dq_r, ddq_r, pk, pd) = Y(q, dq, dq_r, ddq_r, pk)*pi_d
% where pk are the kinematic parameters and pd the dynamic ones. 

% Dynamic regressor
Y = [R*x_ddot_r, 0, 0, R*(x_dot*x_dot_r*(1+sin(theta)))/2; gravity*(L/2)*cos(theta), theta_ddot_r, theta_dot_r, -(L/4)*(x_dot*x_dot_r*sin(theta)*(1+sin(theta)))];

% Introducing an error pi_tilde_d = pi_d - pi_hat_d, where 
% pi_d = [m; I; b; K] and pi_hat_d is its estimation, at steady is valid
% the equation Y(q, dq, dq_r, ddq_r, pk)*pi_tilde_d = 0. 
% In our case, at steady state for constant reference, theta_ddot_r and 
% theta_dot_r are equal to zero, so that b and I belong to the kernel of Y 
% and the estimation error of these parameters is constant.

% To analyze the convergence of the estimation error, we study the rank
% of sigma = integral(Y'*Y) between T and T + deltaT
% dim = rank(Y'*Y);

% Sigma is not enough exciting because it's singular. This suggest that
% some parameters are not estimated. In particular, the kernel is composed
% of [0; 1; 0; 0] and [0; 0; 1; 0] so b and I are not identifiable.

%% 
if selector == 1
    time = 60;
elseif selector == 2
    time = 100;
else
    time = 200;
end

% if model == 1
%     sim('computed_torque');
% elseif model == 2
    sim('EKF_onlyparameters');
% else
    sim('li_slotine');
% end

% run animation.m;

% run graphics.m;
