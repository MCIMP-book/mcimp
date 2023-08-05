% ssdescription/vehicle_steer_linearization.m
% Vehicle steering: bicycle model
% Accuracy of linearization
a = 1.799/2; % Length from rear wheels to center of mass
b = 1.799; % Length of the car
v_0 = 10; % Initial velocity in meters per second

%% nonlinear system
% Define the nonlinear system
f = @(x,u) [v_0*sin(atan(a*tan(u)/b)+x(2)); v_0*sin(atan(a*tan(u)/b))/a];
h = @(x,u) x;

% Set simulation time and initial conditions
tspan = [0 10];
x0 = [0 0];

%% Linearized System
% Define the state-space model of the vehicle
A = [0 v_0; 0 0];
B = [a*v_0/b v_0/b]';
C = [1 0; 0 1];
D = [0; 0];

sys = ss(A,B,C,D);

sys_y = sys(1);
[num_y, den_y] = tfdata(sys_y);
G_y = tf(num_y, den_y);

%% Effect of input scaling
Ugain = [0.05,0.1,0.5,1];
t = linspace(tspan(1),tspan(2),1000);
Y = zeros(length(t),length(Ugain));
Ylinear = Y;
for ii = 1:length(Ugain)
    % Define input signal
    u = @(t) Ugain(ii)*sin(2*pi*1*t); % Sinusoidal input signal with a frequency of 1 Hz and an amplitude of 0.1 radians

    % Simulate the nonlinear system using ode45 solver
    [t,x] = ode45(@(t,x) f(x,u(t)), t, x0);

    Y(:,ii) = x(:,1);

    % Simulate the linearized system
    Ylinear(:,ii) = lsim(sys_y,u(t),t,x0);
end

% Plot the results
figure;
subplot(411)
plot(t, Y(:,1), t, Ylinear(:,1), 'r--');
title('Response to input at 1 Hz and magnitude 0.05 radians');
ylabel('Position y (m)');
legend('nonlinear model','linearized model','Location','northwest')
subplot(412)
plot(t, Y(:,2), t, Ylinear(:,2), 'r--');
title('Response to input at 1 Hz and magnitude 0.1 radians');
ylabel('Position y (m)');
subplot(413)
plot(t, Y(:,3), t, Ylinear(:,3), 'r--');
title('Response to input at 1 Hz and magnitude 0.5 radians');
ylabel('Position y (m)');
subplot(414)
plot(t, Y(:,4), t, Ylinear(:,4), 'r--');
xlabel('Time (s)');
ylabel('Position y (m)');
title('Response to input at 1 Hz and magnitude 1 radians');

%% Effect of input frequency variation
FreqVect = [1,5,10,30];
t = linspace(tspan(1),tspan(2),1000);
Y = zeros(length(t),length(FreqVect));
Ylinear = Y;
for ii = 1:length(FreqVect)
    % Define input signal
    u = @(t) 0.05*sin(2*pi*FreqVect(ii)*t);

    % Simulate the nonlinear system using ode45 solver
    [t,x] = ode45(@(t,x) f(x,u(t)), t, x0);

    Y(:,ii) = x(:,1);

    % Simulate the linearized system
    Ylinear(:,ii) = lsim(sys_y,u(t),t,x0);
end

% Plot the results
figure;
subplot(411)
plot(t, Y(:,1), t, Ylinear(:,1), 'r--');
title(['Response to input at ',num2str(FreqVect(1)), ' Hz and magnitude 0.05 radians']);
ylabel('Position y (m)');
legend('nonlinear model','linearized model','Location','northwest')
subplot(412)
plot(t, Y(:,2), t, Ylinear(:,2), 'r--');
title(['Response to input at ',num2str(FreqVect(2)), ' Hz and magnitude 0.05 radians']);
ylabel('Position y (m)');
subplot(413)
plot(t, Y(:,3), t, Ylinear(:,3), 'r--');
title(['Response to input at ',num2str(FreqVect(3)), ' Hz and magnitude 0.05 radians']);
ylabel('Position y (m)');
subplot(414)
plot(t, Y(:,4), t, Ylinear(:,4), 'r--');
xlabel('Time (s)');
ylabel('Position y (m)');
title(['Response to input at ',num2str(FreqVect(4)), ' Hz and magnitude 0.05 radians']);
