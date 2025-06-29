%% ##### SIPC ######

%% Constants 
q_i = [0 10*pi/180 0 0];  % Starting state fof system

M = 0.2;     % [kg] - Cart Mass
m = 0.02;   % [kg] - Pendulum End Mass
m_l = 0.01; % [kg] - Pendulum Mass

L = 0.28;   % [m]  - Pendulum Length

k = 0.1;  % [ ]   - Wheel/Ground friction coefficient
b = 0.05;  % [ ]   - Pendulum/Rotator angular friction coefficient

g = 9.81; % [m/s^2] - Gravitational Acceleration

%% System Analysis
% Intermediary Matricies
M0 = [M+m+m_l        (m+0.5*m_l)*L
    (m+0.5*m_l)*L   (m+0.25*m_l)*L^2]; % Mass matrix, linearized
F0 = [-k 0
      0 -b];    % Friction matrix, linearized
G0 = [0       0
     0      -(m + 0.5*m_l)*g*L];     % Gravity matrix, linearized
H = [1
     0];     
invM0 = inv(M0);

% System Matricies
A = [zeros(2,2) eye(2,2)
     invM0*G0   invM0*F0];  % System dynamics matrix
B = [ zeros(2,2)
     invM0];              % Input matrix

C = eye(4,4);    % Output
D =  0;

S = [0 1 0 0];  % Sensor Matrix;

% Plant
plant = ss(A, B, C, D);
plant.Name = "SIPC";
plant.InputName = "u";
plant.StateName = ["x", "theta", "x_dot", "theta_dot"];
plant.OutputName = ["x", "theta", "x_dot", "theta_dot"];

plant_tf = tf(plant);

W_c = ctrb(plant);
W_o = obsv(plant);

rO = rank(W_o);
rC = rank(W_c);

% BLDC Model
R = 1.0;    % [Ohm] - DC Motor R_in
L = 0.5;    % [H]   - DC Motor L_in

Kt = 0.001; % [Nm/A] - Torque Constant  
Kb = 0.00158; % [V/(rad/s)] - EMF Constant
J = 0.001;    % [kg.m^2] - Moment of Inertia at shaft
b_m = 0.001; % [ ] - Angular friction

r_wheel = 0.01; % [m] - Radius of wheel