% kf/second_order_kf.m
A=[0 1;0 0.7114];Bw=[0.0384;0.0722];C=[1 0];D=0;
W=1; % input noise variance
r=[0.05 2];

format long;
% solving for Xss
Xss=dlyap(A,Bw*Bw'); % XSS=A*Xss*A'+Bw*Bw'
X11=Xss(1,1);
V1=r(1)^2*X11;
V2=r(2)^2*X11;

% steady-state KF gains and covariances
%    [X,L,G] = DARE(A,B,Q,R,S,E) computes the unique stabilizing
%    G:gain matrix  X:M
%                                       -1
%     E'XE = A'XA - (A'XB + S)(B'XB + R)  (A'XB + S)' + Q
M1=dare(A',C',Bw*W*Bw',V1);
F1=M1*C'/(C*M1*C'+V1);
M2=dare(A',C',Bw*W*Bw',V2);
F2=M2*C'/(C*M2*C'+V2);
Z1=M1-M1*C'/(C*M1*C'+V1)*C*M1;
Z2=M2-M2*C'/(C*M2*C'+V2)*C*M2;

% finite settling time observer not based on least square optimal stochastic estimation
F3=[1;0.7114];
N=200;
w=randn(N,1);
v1=randn(N,1)*(r(1)*sqrt(X11));
v2=randn(N,1)*(r(2)*sqrt(X11));
Ts=1;
t=0:199;

% actual system response
[y0,t0,x]=lsim(ss(A,Bw,C,D,Ts),w,t); % assume E[x]=0
y1=y0+v1;
y2=y0+v2;

% Kalman Filter simulation
Akf1=[0 1-F1(1); 0 0.7114-F1(2)];
Bkf1=F1;
[y_kf1,t1,x_kf1]=lsim(ss(Akf1,Bkf1,C,D,Ts),y1,t);
Akf2=[0 1-F2(1); 0 0.7114-F2(2)];
Bkf2=F2;
[y_kf2,t2,x_kf2]=lsim(ss(Akf2,Bkf2,C,D,Ts),y2,t);

% finite settling time observer simulation
Afsto=[0 1-F3(1); 0 0.7114-F3(2)];
Bfsto=F3;
[y_fsto1,t3,x_fsto1]=lsim(ss(Afsto,Bfsto,C,D,Ts),y1,t);
[y_fsto2,t4,x_fsto2]=lsim(ss(Afsto,Bfsto,C,D,Ts),y2,t);

% estimation error covariance matrix from the time average
Zss1_t=zeros(2,2);
Zss1_t(1,1)=((x(1:end-1,1)-x_kf1(2:end,1))'*(x(1:end-1,1)-x_kf1(2:end,1)))/(N-1);
Zss1_t(1,2)=((x(1:end-1,1)-x_kf1(2:end,1))'*(x(1:end-1,2)-x_kf1(2:end,2)))/(N-1);
Zss1_t(2,1)=Zss1_t(1,2);
Zss1_t(2,2)=((x(1:end-1,2)-x_kf1(2:end,2))'*(x(1:end-1,2)-x_kf1(2:end,2)))/(N-1);

Zss2_t=zeros(2,2);
Zss2_t(1,1)=((x(1:end-1,1)-x_kf2(2:end,1))'*(x(1:end-1,1)-x_kf2(2:end,1)))/(N-1);
Zss2_t(1,2)=((x(1:end-1,1)-x_kf2(2:end,1))'*(x(1:end-1,2)-x_kf2(2:end,2)))/(N-1);
Zss2_t(2,1)=Zss2_t(1,2);
Zss2_t(2,2)=((x(1:end-1,2)-x_kf2(2:end,2))'*(x(1:end-1,2)-x_kf2(2:end,2)))/(N-1);

figure;m=8;
subplot(m,1,1);plot(t0,x(:,1));title('x_1');
subplot(m,1,2);plot(t0,v1);title('v');
subplot(m,1,3);plot(t0,y1);title('y');
subplot(m,1,4);plot(t1,x_kf1(:,1));title('x_1_e_s_t(k|k)');
subplot(m,1,5);plot(t0,x(:,2));title('x_2');
subplot(m,1,6);plot(t1,x_kf1(:,2));title('x_2_e_s_t(k|k)');
subplot(m,1,7);plot(t1,x_fsto1(:,1));title('x_1_e_s_t(k) FSTO');
subplot(m,1,8);plot(t1,x_fsto1(:,2));title('x_2_e_s_t(k) FSTO');
xlabel 'time step: k'

figure;
subplot(m,1,1);plot(t0,x(:,1));title('x_1');
subplot(m,1,2);plot(t0,v2);title('v');
subplot(m,1,3);plot(t0,y2);title('y');
subplot(m,1,4);plot(t2,x_kf2(:,1));title('x_1_e_s_t(k|k)');
subplot(m,1,5);plot(t0,x(:,2));title('x_2');
subplot(m,1,6);plot(t2,x_kf2(:,2));title('x_2_e_s_t(k|k)');
subplot(m,1,7);plot(t1,x_fsto2(:,1));title('x_1_e_s_t(k) FSTO');
subplot(m,1,8);plot(t1,x_fsto2(:,2));title('x_2_e_s_t(k) FSTO');
xlabel 'time step: k'

% comparison of the direct output measurement with the KF
figure
plot(t0,x(:,1)-y1,'r--',t1(1:end-1),x(1:end-1,1)-x_kf1(2:(end),1),'k');% note the index of x(k|k); the KF is one-step delayed
legend('x_1-y','x_1-x_1_e_s_t(k|k)');title(['direct measurement error vs estimation error r=',num2str(r(1))]);
xlabel 'k'; ylabel 'x_1'
figure
plot(t0,x(:,1)-y2,'r--',t2(1:end-1),x(1:end-1,1)-x_kf2(2:end,1),'k');
legend('x_1-y','x1-x_1_e_s_t(k|k)');title(['direct measurement error vs estimation error r=',num2str(r(2))]);
xlabel 'k'; ylabel 'x_1'

% comparison of the finite settleing time observer with the KF
figure
plot(t0(1:end-1),x(1:end-1,2)-x_fsto1(2:end,2),'r--',t1(1:end-1),x(1:end-1,2)-x_kf1(2:(end),2),'k');
legend('FSTO','KF');title(['estimation error of the second state; r=',num2str(r(1))]);
xlabel 'k'; ylabel 'x_2'
figure
plot(t0(1:end-1),x(1:end-1,2)-x_fsto2(2:end,2),'r--',t2(1:end-1),x(1:end-1,2)-x_kf2(2:end,2),'k');
legend('FSTO','KF');title(['estimation error of the second state; r=',num2str(r(2))]);
xlabel 'k'; ylabel 'x_2'
