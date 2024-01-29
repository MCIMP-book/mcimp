% modeling/hddpzt.m
% MATLAB code to generate the pzt-stage HDD model
num_sector=420;                   % Number of sector
num_rpm=7200;                     % Number of RPM
Ts = 1/(num_rpm/60*num_sector);   % Sampling time

% PZT
omega_pzt=[14800 ,21500 ,28000 ,40200 ,42050,44400,46500 ,100000]*2*pi;
kappa_pzt=[-0.005,-0.01 ,-0.1  ,+0.8  ,0.3  ,-0.25  ,0.3  ,10.0 ];
zeta_pzt =[0.025 ,0.03  ,0.05  ,0.008  ,0.008 ,0.01 ,0.02  ,0.3 ];

Sys_Pc_pzt_c1=0;
for i=1:length(omega_pzt)
	Sys_Pc_pzt_c1=Sys_Pc_pzt_c1+tf([0,0,kappa_pzt(i)],[1, 2*zeta_pzt(i)*omega_pzt(i), (omega_pzt(i))^2]);
end
Sys_Pc_pzt_c1=Sys_Pc_pzt_c1/abs(freqresp(Sys_Pc_pzt_c1,0));

%% Frequency response
f=logspace(1,log10(60e3),3000);
Fr_Pc_pzt_c1=squeeze(freqresp(Sys_Pc_pzt_c1,f*2*pi)).';

figure
subplot(211)
semilogx(f,20*log10(abs(Fr_Pc_pzt_c1)))
title('P_{cp}');xlabel('Frequency [Hz]');ylabel('Gain [dB]');grid;axis([1e3 f(end) -10 30])
subplot(212)
semilogx(f,angle(Fr_Pc_pzt_c1)*180/pi)
xlabel('Frequency [Hz]');ylabel('Phase [deg.]');grid;axis([1e3 f(end) -180 180]);yticks(-180:90:180)
% saveas(gcf,'images/hdd_pcpzt_baseline.png')
% saveas(gcf,'images/hdd_pcpzt_baseline.pdf')
