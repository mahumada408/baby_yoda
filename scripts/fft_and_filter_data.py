import numpy as np
from scipy.fft import fft
from scipy import signal

pose_data = np.genfromtxt('/home/manuel/projects/baby_yoda/pitch.csv', delimiter=',')

# Number of sample points
N = pose_data.shape[0]
# sample spacing
# T = 1.0 / 500.0
T = pose_data[1,0] - pose_data[0,0]
print(T)
yf = fft(pose_data[:,1])
xf = np.linspace(0.0, 1.0/(2.0*T), N//2)
print(yf.shape)
print(xf.shape)
import matplotlib.pyplot as plt
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.grid()
plt.show()
# exit(0)
plt.close()

# Filter order
order = 2

# Cutoff frequency expressed as a fraction of sampling.
wn = 10/100
numerator_coeffs, denominator_coeffs = signal.butter(order, 0.5, fs=1/T)
filtered_signal = signal.lfilter(numerator_coeffs, denominator_coeffs, pose_data[:,1])

plt.plot(pose_data[:,0], pose_data[:,1])
filtered_signal = filtered_signal - 173
plt.scatter(pose_data[:,0], filtered_signal)
plt.show()
plt.close()
new_data = np.concatenate((pose_data[:,0].reshape(N,1), filtered_signal.reshape(N,1)), axis=1).reshape(N,2)
np.savetxt("face_angles_filtered.csv", new_data, delimiter=',')
plt.plot(new_data[:,0], new_data[:,1])
plt.show()