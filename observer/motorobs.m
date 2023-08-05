% observer/motorobs.m
% State observer design for motion control in MATLAB
%% Continuous-time system model
% motor parameters
L = 1e-3; R = 1; J = 5e-5; B = 1e-4; K = 0.1;

% state-space model
A = [-R/L, 0, -K/L; 0, 0, 1; K/J, 0, -B/J];
B = [1/L; 0; 0];
C = [0, 1, 0];
D = [0];

% check original eigenvalues
eig(A)
%% Observer design
% check observability
O = obsv(A,C);
rank(O)

% desired poles for the observer
pole_des = [-500+250j, -500-250j, -1000];

% design observer by placing poles of A-LC
Lt = place(A.',C.',pole_des);
L  = Lt.'

% check poles of estimator-error dynamics
est_poles = eig(A - L*C)

%% Simulation
% define augmented system to run the simulation
Aaug = [A, zeros(3,3); L*C, A-L*C];
Baug = [B;B];
Caug = [C, zeros(1,3)];
Daug = 0;
sys = ss(Aaug,Baug,Caug,Daug);

% define intial conditions
x0 = [10, 2, 10]'; xhat0 = [0, 0, 0]'; X0 = [x0; xhat0];

% define simulink parameters
Tend = 0.03; % simulation end time
amplitude = 10; % sin wave input amplitude
initpha = 0; % initial phase
freq = 600; % sin wave freq (rad/s)
t = 0:1e-4:Tend;

u = amplitude*sin(freq*t+initpha);

[Y,T,X] = lsim(sys,u,t,X0);

%% Performance verification
figure;
subplot(3,1,1); plot(t, X(:,1), t, X(:,4), '--','linewidth',1.5);
xlabel('time (sec)'); legend('$x_1 = i_a$', '$\hat x_1$','Interpreter','latex'); grid;
ylabel('$x_1$','Interpreter','latex')
title('States and their estimates');
subplot(3,1,2); plot(t, X(:,2), t, X(:,5), '--','linewidth',1.5);
xlabel('time (sec)'); legend('$x_2 = \theta$', '$\hat x_2$','Interpreter','latex'); grid;
ylabel('$x_2$','Interpreter','latex')
subplot(3,1,3); plot(t, X(:,3), t, X(:,6), '--','linewidth',1.5);
xlabel('time (sec)'); legend('$x_3 = \dot{\theta}$', '$\hat x_3$','Interpreter','latex'); grid;
ylabel('$x_3$','Interpreter','latex')
