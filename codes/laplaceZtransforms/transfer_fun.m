% laplaceZtransforms/transfer_fun.m
num = [1 2];
den = [1 2 3];
sys_tf = tf(num,den)
poles = pole(sys_tf);
zeros = zero(sys_tf);
disp(['System Poles = ',num2str(poles')])
disp(['System Zeros = ',num2str(zeros')])

[yout, T] = step(sys_tf);
figure, plot(T, yout)
figure, impulse(sys_tf)

u1 = 2*ones(length(T),1);
u2 = sin(T);
figure, lsim(sys_tf,u1,T)
figure, lsim(sys_tf,u2,T)
