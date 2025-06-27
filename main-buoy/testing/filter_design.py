import numpy as np 
import matplotlib.pyplot as plt
from scipy.signal import freqz

Ts = 0.01
fs = 1.0 / Ts
print("Sampling frequency: ", fs)
num_samples = 10000

f = np.linspace(-fs/2, fs/2, num_samples)
omega = 2 * np.pi * f/fs

# Filter order: 
N1 = 10
N2 = 5

def compute_log_mag(h): 
    """Compute the log magnitude of the filter response in dB."""
    return 20 * np.log10(np.abs(h))

def compute_phase(h):
    """Compute the phase of the filter response in degrees [°]."""
    return np.unwrap(np.angle(h,deg=True),period=360)

# Moving average normal: 
H1z = np.zeros(num_samples, dtype=complex)
for k in range(N1): 
    H1z += np.exp(-1j * omega * k)
H1z /= N1

H1z_mag = compute_log_mag(H1z)
H1z_mag = H1z_mag[num_samples//2:] 
H1z_phase = compute_phase(H1z)

# Recursive Moving Average:
H2z = (1/N1) * (1 / (1 - (N1-1)/N1 * np.exp(-1j * omega)))
H2z_mag = compute_log_mag(H2z)
H2z_mag = H2z_mag[num_samples//2:]
H2z_phase = compute_phase(H2z)

# Frequency axis for positive frequencies only
fpos = f[num_samples//2:]

# --- 3dB cutoff calculation ---
H1z_max = np.max(H1z_mag)
H2z_max = np.max(H2z_mag)

H1z_3db = H1z_max - 3
H2z_3db = H2z_max - 3

# Find first index where response falls below -3 dB point
idx_3db_H1z = np.where(H1z_mag <= H1z_3db)[0][0]
idx_3db_H2z = np.where(H2z_mag <= H2z_3db)[0][0]

f_cutoff_H1z = fpos[idx_3db_H1z]
f_cutoff_H2z = fpos[idx_3db_H2z]

print(f"-3 dB cutoff for H1z: {f_cutoff_H1z:.3f} Hz")
print(f"-3 dB cutoff for H2z: {f_cutoff_H2z:.3f} Hz")

# --- Plotting ---
fig, ax = plt.subplots(2, 1, figsize=(10, 8))

# Magnitude response
ax[0].plot(fpos, H1z_mag, label='Moving Average (Normal)')
ax[0].plot(fpos, H2z_mag, label='Moving Average (Recursive)')

# -3 dB lines
ax[0].axhline(H1z_3db, color='gray', linestyle='--')
ax[0].axvline(f_cutoff_H1z, color='red', linestyle='--', label=f'H1z -3 dB @ {f_cutoff_H1z:.2f} Hz')
ax[0].axhline(H2z_3db, color='gray', linestyle='--')
ax[0].axvline(f_cutoff_H2z, color='green', linestyle='--', label=f'H2z -3 dB @ {f_cutoff_H2z:.2f} Hz')

ax[0].set_title('Frequency Response of Moving Average Filters')
ax[0].set_xlabel('Frequency (Hz)')
ax[0].set_ylabel('Magnitude (dB)')
ax[0].set_ylim(-40, 1)
ax[0].grid()
ax[0].legend()

# Phase response
ax[1].plot(f, np.unwrap(np.angle(H1z,deg=True),period=360), label='Moving Average (Normal)')
ax[1].plot(f, np.unwrap(np.angle(H2z,deg=True),period=360), label='Moving Average (Recursive)')
ax[1].set_title('Phase Response of Moving Average Filters')
ax[1].set_xlabel('Frequency (Hz)')
ax[1].set_ylabel('Phase (degrees)')
ax[1].grid()
ax[1].legend()

plt.tight_layout()
plt.show()
