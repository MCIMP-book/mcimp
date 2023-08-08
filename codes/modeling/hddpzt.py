# modeling/hddpzt.py
import numpy as np
import matplotlib.pyplot as plt
import control as ct

num_sector = 420  # Number of sector
num_rpm = 7200  # Number of RPM
Ts = 1 / (num_rpm / 60 * num_sector)  # Sampling time

# PZT
omega_pzt = np.array([14800, 21500, 28000, 40200, 42050,
                     44400, 46500, 100000]) * 2 * np.pi
kappa_pzt = np.array([-0.005, -0.01, -0.1, +0.8, 0.3, -0.25, 0.3, 10.0])
zeta_pzt = np.array([0.025, 0.03, 0.05, 0.008, 0.008, 0.01, 0.02, 0.3])

s = ct.TransferFunction.s  # Create a variable for the differentiation operator
Sys_Pc_pzt_c1 = 0  # Create an empty transfer function
for i in range(len(omega_pzt)):
    Sys_Pc_pzt_c1 += kappa_pzt[i] / (s**2 + 2 * zeta_pzt[i] * omega_pzt[i]
                                     * s + (omega_pzt[i]) ** 2)  # Add the transfer functions
Sys_Pc_pzt_c1 /= Sys_Pc_pzt_c1(0)  # Normalize the gain at zero frequency

# Frequency response
f = np.logspace(1, np.log10(60e3), 3000)
w = f * 2 * np.pi

magPc_pzt, phase_Pc_pzt, omega_Pc_pzt = ct.freqresp(
    Sys_Pc_pzt_c1, w)  # Get the frequency response

# figure
plt.figure()
plt.subplot(211)
plt.semilogx(f, 20*np.log10(magPc_pzt))
plt.title('$P_{cp}$')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Gain [dB]')
plt.grid()
plt.axis([1000, f[-1], - 10, 30])
plt.subplot(212)
plt.semilogx(f, phase_Pc_pzt*180/np.pi)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Phase [deg.]')
plt.grid()
plt.axis([1000, f[-1], - 180, 180])
plt.yticks(np.arange(-180, 270, 90))
