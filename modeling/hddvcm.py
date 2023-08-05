# modeling/hddvcm.py
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import control as ct

num_sector = 420  # Number of sector
num_rpm = 7200  # Number of RPM
Ts = 1 / (num_rpm / 60 * num_sector)  # Sampling time

# VCM
Kp_vcm = 3.7976e+07  # VCM gain
omega_vcm = np.array([0, 5300, 6100, 6500, 8050, 9600, 14800, 17400,
                     21000, 26000, 26600, 29000, 32200, 38300, 43300, 44800]) * 2 * np.pi
kappa_vcm = np.array([1, -1.0, +0.1, -0.1, 0.04, -0.7, -
                     0.2, -1.0, +3.0, -3.2, 2.1, -1.5, +2.0, -0.2, +0.3, -0.5])
zeta_vcm = np.array([0, 0.02, 0.04, 0.02, 0.01, 0.03, 0.01,
                    0.02, 0.02, 0.012, 0.007, 0.01, 0.03, 0.01, 0.01, 0.01])

Sys_Pc_vcm_c1 = ct.TransferFunction(
    [], [1])  # Create an empty transfer function
for i in range(len(omega_vcm)):
    Sys_Pc_vcm_c1 = Sys_Pc_vcm_c1 + ct.TransferFunction(np.array(
        [0, 0, kappa_vcm[i]]) * Kp_vcm, np.array([1, 2 * zeta_vcm[i] * omega_vcm[i], (omega_vcm[i]) ** 2]))

# Frequency response
f = np.logspace(1, np.log10(60e3), 3000)
w = f * 2 * np.pi
magPc_vcm, phase_Pc_vcm, omega_Pc_vcm = ct.freqresp(
    Sys_Pc_vcm_c1, w)  # Get the frequency response

# figure
plt.figure()
plt.subplot(211)
plt.semilogx(f, 20*np.log10(magPc_vcm))
plt.title('$P_{cv}$')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Gain [dB]')
plt.grid()
plt.axis([10, f[-1], -90, 100])
plt.subplot(212)
plt.semilogx(f, np.mod(phase_Pc_vcm*180/np.pi+360, 360)-360)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Phase [deg.]')
plt.grid()
plt.axis([10, f[-1], -360, 0])
plt.yticks(np.arange(-360, 90, 90))
