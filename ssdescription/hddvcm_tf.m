% ssdescription/hddvcm_tf.m
Kp_vcm=3.7976e+07;                % VCM gain
omega_vcm=[0, 5300 ,6100 ,6500 ,8050 ,9600 ,14800 ,17400 ,21000 ,26000 ,26600 ,29000 ,32200 ,38300 ,43300 ,44800]*2*pi;
kappa_vcm=[1, -1.0 ,+0.1 ,-0.1 ,0.04 ,-0.7 ,-0.2  ,-1.0  ,+3.0  ,-3.2  ,2.1   ,-1.5  ,+2.0  ,-0.2  ,+0.3  ,-0.5 ];
zeta_vcm =[0, 0.02 ,0.04 ,0.02 ,0.01 ,0.03 ,0.01  ,0.02  ,0.02  ,0.012 ,0.007 ,0.01  ,0.03  ,0.01  ,0.01  ,0.01 ];

Sys_Pc_vcm_c1=0;
for i=1:length(omega_vcm)
	Sys_Pc_vcm_c1=Sys_Pc_vcm_c1+tf([0,0,kappa_vcm(i)]*Kp_vcm,[1, 2*zeta_vcm(i)*omega_vcm(i), (omega_vcm(i))^2]);
end
