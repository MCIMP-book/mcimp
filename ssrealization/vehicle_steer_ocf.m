% ssrealization/vehicle_steer_ocf.m
a = 1.799/2; % Length from rear wheels to center of mass
b = 1.799; % Length of the car
v_0 = 10; % Initial velocity in meters per second

%% Linearized System
% Define the state-space model of the vehicle
A = [0 v_0; 0 0];
B = [a*v_0/b v_0/b]';
C = [1 0; 0 1];
D = [0; 0];

sys = ss(A,B,C,D);

% Choosing y as the single output
sys_y = sys(1);

% Transfer function by analysis
Gy = v_0/b*tf([0 a v_0],[1 0 0])

% Computed transfer function
[num_y, den_y] = tfdata(sys_y);
G_y = tf(num_y, den_y)
