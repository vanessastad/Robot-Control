%% Main

clear all
close all
clc

%% Data 

m1 = 1.309; % [kg]
m2 = 0.098; % [kg]
mM = 0.06; % [kg]
J1 = 0.0154; % [kg*m^2]
J2 = 0.0011; % [kg*m^2]
Jm = 0.4101; % [kg*m^2]
d2x = 0; % [m]
d2z = 0.0979; % [m]
dgx = 0; % [m]
dgz = 0.0081; % [m]
gravity = 9.81; % [m/s^2]

% Controller gain
lambda = [100 300 300 100 100 300 300 100 100 300 300 100]; 

wx = 0.16;  % Pulsation for references [rad/s] 
wy = 0.16;  % Pulsation for references [rad/s]

selector = select(1);

if selector == 3
    Ax = 10;    % Amplitude for references [rad]    
    Ay = 10;    % Amplitude for references [rad] 
else
    Ax = 25;    % Amplitude [rad]    
    Ay = 25;    % Amplitude [rad] 
end

sim('feedbacklin');

run animation.m;

run graphics.m;