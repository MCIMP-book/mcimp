% modeling/hdddsa.m
% Dual-stage HDD model
num_sector=420;                   % Number of sector
num_rpm=7200;                     % Number of RPM
Ts = 1/(num_rpm/60*num_sector);   % Sampling time

% VCM
Kp_vcm=3.7976e+07;
omega_vcm=[0, 5300 ,6100 ,6500 ,8050 ,9600 ,14800 ,17400 ,21000 ,26000 ,26600 ,29000 ,32200 ,38300 ,43300 ,44800]*2*pi;
kappa_vcm=[1, -1.0 ,+0.1 ,-0.1 ,0.04 ,-0.7 ,-0.2  ,-1.0  ,+3.0  ,-3.2  ,2.1   ,-1.5  ,+2.0  ,-0.2  ,+0.3  ,-0.5 ];
zeta_vcm =[0, 0.02 ,0.04 ,0.02 ,0.01 ,0.03 ,0.01  ,0.02  ,0.02  ,0.012 ,0.007 ,0.01  ,0.03  ,0.01  ,0.01  ,0.01 ];

% PZT
omega_pzt=[14800 ,21500 ,28000 ,40200 ,42050,44400,46500 ,100000]*2*pi;
kappa_pzt=[-0.005,-0.01 ,-0.1  ,+0.8  ,0.3  ,-0.25  ,0.3  ,10.0 ];
zeta_pzt =[0.025 ,0.03  ,0.05  ,0.008  ,0.008 ,0.01 ,0.02  ,0.3 ];

%% LT(Low temp.) model：VCM: +4 % resonance shift from nominal model, PZT actuator: +6 % resonance shift from nominal model
% VCM
Sys_Pc_vcm_c1=0;
for i=1:length(omega_vcm)
	Sys_Pc_vcm_c1=Sys_Pc_vcm_c1+ss(tf([0,0,kappa_vcm(i)]*Kp_vcm,[1, 2*zeta_vcm(i)*0.8*omega_vcm(i)*1.04, (omega_vcm(i)*1.04)^2]));
	Sys_Pc_vcm_c1=ssbal(Sys_Pc_vcm_c1);
end

% PZT
Sys_Pc_pzt_c1=0;
for i=1:length(omega_pzt)
	Sys_Pc_pzt_c1=Sys_Pc_pzt_c1+ss(tf([0,0,kappa_pzt(i)],[1, 2*zeta_pzt(i)*0.8*omega_pzt(i)*1.06, (omega_pzt(i)*1.06)^2]));
	Sys_Pc_pzt_c1=ssbal(Sys_Pc_pzt_c1);
end
Sys_Pc_pzt_c1=Sys_Pc_pzt_c1/abs(freqresp(Sys_Pc_pzt_c1,0));

%% RT(Room temp.) model：Same as nominal models
% VCM
Sys_Pc_vcm_c2=0;
for i=1:length(omega_vcm)
	Sys_Pc_vcm_c2=Sys_Pc_vcm_c2+ss(tf([0,0,kappa_vcm(i)]*Kp_vcm,[1, 2*zeta_vcm(i)*omega_vcm(i), omega_vcm(i)^2]));
	Sys_Pc_vcm_c2=ssbal(Sys_Pc_vcm_c2);
end

% PZT
Sys_Pc_pzt_c2=0;
for i=1:length(omega_pzt)
	Sys_Pc_pzt_c2=Sys_Pc_pzt_c2+ss(tf([0,0,kappa_pzt(i)],[1, 2*zeta_pzt(i)*omega_pzt(i), omega_pzt(i)^2]));
	Sys_Pc_pzt_c2=ssbal(Sys_Pc_pzt_c2);
end
Sys_Pc_pzt_c2=Sys_Pc_pzt_c2/abs(freqresp(Sys_Pc_pzt_c2,0));

%% HT(High temp.) model：VCM: −4 % resonance shift from nominal model, PZT actuator: −6 % resonance shift from nominal model
% VCM
Sys_Pc_vcm_c3=0;
for i=1:length(omega_vcm)
	Sys_Pc_vcm_c3=Sys_Pc_vcm_c3+ss(tf([0,0,kappa_vcm(i)]*Kp_vcm,[1, 2*zeta_vcm(i)*1.2*omega_vcm(i)*0.96, (omega_vcm(i)*0.96)^2]));
	Sys_Pc_vcm_c3=ssbal(Sys_Pc_vcm_c3);
end

% PZT
Sys_Pc_pzt_c3=0;
for i=1:length(omega_pzt)
	Sys_Pc_pzt_c3=Sys_Pc_pzt_c3+ss(tf([0,0,kappa_pzt(i)],[1, 2*zeta_pzt(i)*1.2*omega_pzt(i)*0.94, (omega_pzt(i)*0.94)^2]));
	Sys_Pc_pzt_c3=ssbal(Sys_Pc_pzt_c3);
end
Sys_Pc_pzt_c3=Sys_Pc_pzt_c3/abs(freqresp(Sys_Pc_pzt_c3,0));

%% LT / PZT gain +5% (Case 4)
Sys_Pc_vcm_c4=Sys_Pc_vcm_c1;
Sys_Pc_pzt_c4=Sys_Pc_pzt_c1*1.05;

%% RT / PZT gain +5% (Case 5)
Sys_Pc_vcm_c5=Sys_Pc_vcm_c2;
Sys_Pc_pzt_c5=Sys_Pc_pzt_c2*1.05;

%% HT / PZT gain +5% (Case 6)
Sys_Pc_vcm_c6=Sys_Pc_vcm_c3;
Sys_Pc_pzt_c6=Sys_Pc_pzt_c3*1.05;

%% LT / PZT gain -5% (Case 7)
Sys_Pc_vcm_c7=Sys_Pc_vcm_c1;
Sys_Pc_pzt_c7=Sys_Pc_pzt_c1*0.95;

%% RT / PZT gain -5% (Case 8)
Sys_Pc_vcm_c8=Sys_Pc_vcm_c2;
Sys_Pc_pzt_c8=Sys_Pc_pzt_c2*0.95;

%% HT / PZT gain -5% (Case 9)
Sys_Pc_vcm_c9=Sys_Pc_vcm_c3;
Sys_Pc_pzt_c9=Sys_Pc_pzt_c3*0.95;

%% All plant
Sys_Pc_vcm_all=[Sys_Pc_vcm_c1;Sys_Pc_vcm_c2;Sys_Pc_vcm_c3;Sys_Pc_vcm_c4;Sys_Pc_vcm_c5;Sys_Pc_vcm_c6;Sys_Pc_vcm_c7;Sys_Pc_vcm_c8;Sys_Pc_vcm_c9];
Sys_Pc_pzt_all=[Sys_Pc_pzt_c1;Sys_Pc_pzt_c2;Sys_Pc_pzt_c3;Sys_Pc_pzt_c4;Sys_Pc_pzt_c5;Sys_Pc_pzt_c6;Sys_Pc_pzt_c7;Sys_Pc_pzt_c8;Sys_Pc_pzt_c9];

%% Cotrolled object (Discrete-time system)
% Case 1
Sys_Pd_vcm_c1=c2d(Sys_Pc_vcm_c1,Ts,'ZOH');
Sys_Pd_pzt_c1=c2d(Sys_Pc_pzt_c1,Ts,'ZOH');

% Case 2
Sys_Pd_vcm_c2=c2d(Sys_Pc_vcm_c2,Ts,'ZOH');
Sys_Pd_pzt_c2=c2d(Sys_Pc_pzt_c2,Ts,'ZOH');

% Case 3
Sys_Pd_vcm_c3=c2d(Sys_Pc_vcm_c3,Ts,'ZOH');
Sys_Pd_pzt_c3=c2d(Sys_Pc_pzt_c3,Ts,'ZOH');

% Case4
Sys_Pd_vcm_c4=c2d(Sys_Pc_vcm_c4,Ts,'ZOH');
Sys_Pd_pzt_c4=c2d(Sys_Pc_pzt_c4,Ts,'ZOH');

% Case 5
Sys_Pd_vcm_c5=c2d(Sys_Pc_vcm_c5,Ts,'ZOH');
Sys_Pd_pzt_c5=c2d(Sys_Pc_pzt_c5,Ts,'ZOH');

% Case 6
Sys_Pd_vcm_c6=c2d(Sys_Pc_vcm_c6,Ts,'ZOH');
Sys_Pd_pzt_c6=c2d(Sys_Pc_pzt_c6,Ts,'ZOH');

% Case 7
Sys_Pd_vcm_c7=c2d(Sys_Pc_vcm_c7,Ts,'ZOH');
Sys_Pd_pzt_c7=c2d(Sys_Pc_pzt_c7,Ts,'ZOH');

% Case 8
Sys_Pd_vcm_c8=c2d(Sys_Pc_vcm_c8,Ts,'ZOH');
Sys_Pd_pzt_c8=c2d(Sys_Pc_pzt_c8,Ts,'ZOH');

% Case 9
Sys_Pd_vcm_c9=c2d(Sys_Pc_vcm_c9,Ts,'ZOH');
Sys_Pd_pzt_c9=c2d(Sys_Pc_pzt_c9,Ts,'ZOH');

% All
Sys_Pd_vcm_all=[Sys_Pd_vcm_c1;Sys_Pd_vcm_c2;Sys_Pd_vcm_c3;Sys_Pd_vcm_c4;Sys_Pd_vcm_c5;Sys_Pd_vcm_c6;Sys_Pd_vcm_c7;Sys_Pd_vcm_c8;Sys_Pd_vcm_c9];
Sys_Pd_pzt_all=[Sys_Pd_pzt_c1;Sys_Pd_pzt_c2;Sys_Pd_pzt_c3;Sys_Pd_pzt_c4;Sys_Pd_pzt_c5;Sys_Pd_pzt_c6;Sys_Pd_pzt_c7;Sys_Pd_pzt_c8;Sys_Pd_pzt_c9];

%% Frequency response
f=logspace(1,log10(60e3),3000);
Fr_Pc_vcm_all=squeeze(freqresp(Sys_Pc_vcm_all,f*2*pi)).';
Fr_Pc_pzt_all=squeeze(freqresp(Sys_Pc_pzt_all,f*2*pi)).';
Fr_Pd_vcm_all=squeeze(freqresp(Sys_Pd_vcm_all,f*2*pi)).';
Fr_Pd_pzt_all=squeeze(freqresp(Sys_Pd_pzt_all,f*2*pi)).';

%% figure
figure
subplot(211)
semilogx(f,20*log10(abs(Fr_Pc_vcm_all(:,1:7))))
hold on
semilogx(f,20*log10(abs(Fr_Pc_vcm_all(:,8:9))),'--')
hold off
title('P_{cv}');xlabel('Frequency [Hz]');ylabel('Gain [dB]');grid;axis([1e3 f(end) -90 10])
subplot(212)
semilogx(f,mod(angle(Fr_Pc_vcm_all(:,1:7))*180/pi+360,360)-360)
hold on
semilogx(f,mod(angle(Fr_Pc_vcm_all(:,8:9))*180/pi+360,360)-360,'--')
hold off
xlabel('Frequency [Hz]');ylabel('Phase [deg.]');grid;axis([1e3 f(end) -360 0]);yticks(-360:90:0)
legend('Case 1','Case 2','Case 3','Case 4','Case 5','Case 6','Case 7','Case 8','Case 9','Location','NorthWest')

figure
subplot(211)
semilogx(f,20*log10(abs(Fr_Pc_pzt_all(:,1:7))))
hold on
semilogx(f,20*log10(abs(Fr_Pc_pzt_all(:,8:9))),'--')
hold off
title('P_{cp}');xlabel('Frequency [Hz]');ylabel('Gain [dB]');grid;axis([1e3 f(end) -10 30])
subplot(212)
semilogx(f,angle(Fr_Pc_pzt_all(:,1:7))*180/pi)
hold on
semilogx(f,angle(Fr_Pc_pzt_all(:,8:9))*180/pi,'--')
hold off
xlabel('Frequency [Hz]');ylabel('Phase [deg.]');grid;axis([1e3 f(end) -180 180]);yticks(-180:90:180)
legend('Case 1','Case 2','Case 3','Case 4','Case 5','Case 6','Case 7','Case 8','Case 9','Location','NorthWest')
