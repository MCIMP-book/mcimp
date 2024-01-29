% laplaceZtransforms/transfer_fun_dt.m
num = [0.09952, -0.08144];
den = [1, -1.792, 0.8187];
Ts = 0.1; % sampling time
sys_tf = tf(num,den,Ts)
poles = pole(sys_tf);
zeros = zero(sys_tf);
disp(['System Poles = ',num2str(poles')])
disp(['System Zeros = ',num2str(zeros')])

[yout, T] = step(sys_tf);
figure, stairs(T, yout)
figure, impulse(sys_tf)

u1 = 2*ones(length(T),1);
u2 = sin(T);
figure, lsim(sys_tf,u1,T)
figure, lsim(sys_tf,u2,T)
