%% Constants
g = 9.81;   % [kg.m/s^2] Gravitational Constant
k = 0.1;    % [kg/s] Dissipation coefficient between cart & track
d1 = 0.1;   % [kg/s] Dissipation coefficient between cart & pendulum 1
d2 = 0.1;   % [kg/s] Dissipation coefficient between pendulum 1 & 2

%% DPIC Parameters
Mc = 1.5;   % [kg] Mass of cart
m1 = 0.5;   % [kg] Mass of pendulum 1
m2 = 0.5;   % [kg] Mass of pendulum 2
L1 = 0.5;   % [m] Length of pendulum 1
L2 = 0.5;   % [m] Length of pendulum 2

%% Shorthand


%% Linearization Matricies
M0 = [ Mc+m1+m2          (1/2*m1 + m2)*L1      1/2*m2*L2      
          (1/2*m1 + m2)*L1  (1/3*m1 + m2)*(L1)^2  1/2*m2*L1*L2  
          1/2*m2*L2         1/2*m2*L1*L2          1/3*m2*(L2)^2 ]; % M matrix for table state q0
iM0 = inv(M0);  % inv M0 for stable state q0
C0 = [ -k  0  0
        0 -d1 0
        0  0 -d1 ];
dG0 = [ 0          0              0
        0 (1/2*m1 + m2)*g*L1      0
        0          0         1/2*m2*g*L2 ]; % dG/dy evaluated at stable state q0
%% Control Matricies
A_r = iM0 * dG0;
D   = iM0 * C0;
B_r = iM0 * [1;0;0];
Wd_r = iM0;


%   q̇ = A.q + B.u + Wd.wd
%   y = C.q + D.u + Wn.wn
A = [  zeros(3, 3) eye(3, 3)   
           A_r        D      ]; % A matrix infront of the q term
B = [ zeros(3,1)
         B_r     ];  % B matrix infront of the u term
Wd = [ zeros(3, 1)
        Wd_r(:, 1)     ];  % Wd matrix infront of the w_d disturbance term
C = [ eye(2,2) zeros(2,4) ]; % Reduced observability matrix 
D = [ 0 ];

%% System
Q = [ 10  0   0   0   0   0
       0 100  0   0   0   0
       0  0  100  0   0   0
       0  0   0  10   0   0
       0  0   0   0  10   0
       0  0   0   0   0  10 ]; % State Cost
R = 10000;          % Actuator Cost

Rn = [ 1  0
       0 1.3];      %  Measurement Noise Covariance
Qn = 1;             %  Process Disturbance Covariance

% Controller
K = lqr(A, B, Q, R);

% Plant & Open Loop System
plant = ss(A, B, C, D);
plant.InputName = 'u_d';
plant.OutputName = 'yt';
plant.StateName = {'x1', 'x2', 'x3', 'x4', 'x5', 'x6'};

sum_disturb = sumblk('u_d = u + w');
sum_noise = sumblk('y = yt + v', 2);

sys_dist = connect(plant, sum_disturb, {'u', 'w'}, 'yt');
sys_o = connect(sys_dist, sum_noise, {'u', 'w', 'v'}, 'y');

% K-Filter & LQG regulator
[Kf, L, P] = kalman(sys_o, Qn, Rn);

reg = lqgreg(Kf, K);

% Feedback System
sys = connect(sys_o, reg, {'u', 'w', 'v'}, 'y');
sys_tf = tf(sys);

% Controllability and Observability
G_c = ctrb(sys);
G_o = obsv(sys);
Nc = rank(G_c);
No = rank(G_o);

%% Analysis
% x-Impulse
impulse(sys)



