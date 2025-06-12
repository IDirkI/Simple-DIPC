%% ##### SIPC ######

%% Constants 
q_i = [0 0.1 0 0];  % Starting state fof system

M = 1;     % [kg] - Cart Mass
m = 0.5;   % [kg] - Pendulum End Mass
m_l = 0.1; % [kg] - Pendulum Mass

L = 1;   % [m]  - Pendulum Length
l = L/2; % [m]  - Pendulum Half Length

k = 0.3;  % [ ]   - Wheel/Ground friction coefficient
b = 0.1;  % [ ]   - Pendulum/Rotator angular friction coeffieint

g = 9.81; % [m/s^2] - Gravitational Acceleration

%% System Analysis
% Intermediary Matricies
M0 = [M+m+m_l        (m+0.5*m_l)*L
    (m+0.5*m_l)*L   (m+0.25*m_l)*L^2]; % Mass matrix, linearized
F0 = [-k 0
      0 -b];    % Friction matrix, linearized
G0 = [0       0
     0      (m + 0.5*m_l)*g*L];     % Gravity matrix, linearized
H = [1
     0];     
invM0 = inv(M0);

% System Matricies
A = [zeros(2,2) eye(2,2)
     invM0*G0   invM0*F0];  % System dynamics matrix
B = [ zeros(2,1)
     invM0*H];              % Input matrix
W = [eye(2,2)
     invM0];                % Disturbance matrix

C = [0 1 0 0];    % Measurement Matrix
D =  0 ;

Q = [1  0   0  0
     0  100 0  0
     0  0   10 0
     0  0   0  10]; % State Cost Matrix

sys = ss(A, B, C, D);
pole(sys)