%% ##### SIPC ######

%% Constants 
q_i = [0 10*pi/180 0 0];  % Starting state fof system

M = 0.5;     % [kg] - Cart Mass
m = 0.1;   % [kg] - Pendulum End Mass
m_l = 0.1; % [kg] - Pendulum Mass

L = 0.28;   % [m]  - Pendulum Length

k = 0;  % [ ]   - Wheel/Ground friction coefficient
b = 0;  % [ ]   - Pendulum/Rotator angular friction coefficient

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

C = [0 1 0 0];    % Measurement Matrix
D =  0;

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