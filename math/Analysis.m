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
Wd = [ zeros(3,3)
        Wd_r     ];  % Wd matrix infront of the w_d disturbance term
C = [ eye(2,2) zeros(2,4) ]; % Reduced observability matrix 
D = [ 0 ];

%% System
Q = [ 10  0  0  0  0  0
       0 100 0  0  0  0
       0  0  1  0  0  0
       0  0  0  1  0  0
       0  0  0  0  1  0
       0  0  0  0  0  1 ]; % State Cost
R = 1;          % Actuator Cost

Rn = [ 1   0  
       0  1.3 ];    %  Noise Covariance
Qn = 1;             %  Disturbance Covariance

% Plant & System
Plant = ss(A, B, C, D, -1);
Plant.InputName = 'u_d';
Plant.OutputName = 'yt';

Sum = sumblk('u_d = u + w');
sys = connect(Plant, Sum, {'u', 'w'}, 'yt');

% LQR Gain
K = lqr(sys, Q, R);
K = K(1, :);

% Kalman Filter
[Kf, L, P] = kalman(sys, Qn, Rn);

% LQG Regulator
regulator = lqgreg(Kf, K);





