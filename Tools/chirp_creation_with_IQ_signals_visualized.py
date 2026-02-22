import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 1. Setup Parameters ---
BW = 125000          # 125 kHz
SF = 7               
S = 32               
Fs = BW * 10         # High sample rate for smooth waves
M = 2**SF            
T_symbol = M / BW    

# Time for the 1D plots (Full Symbol)
t_full = np.linspace(0, T_symbol, int(Fs * T_symbol), endpoint=False)
# Time for the 3D plot (Small slice to see the 'Spring' clearly)
t_slice = np.linspace(0, T_symbol * 0.15, int(Fs * T_symbol * 0.15), endpoint=False)

def generate_lora(t_array):
    f_offset = (S * BW) / M
    f_t = ((BW * t_array / T_symbol) + f_offset) % BW - (BW / 2)
    phase = 2 * np.pi * np.cumsum(f_t) * (1/Fs)
    return f_t, np.cos(phase), np.sin(phase)

# Generate data
f_full, I_full, Q_full = generate_lora(t_full)
f_slice, I_slice, Q_slice = generate_lora(t_slice)

# --- 2. Visualization ---
fig = plt.figure(figsize=(15, 10))

# Plot A: Frequency vs Time (The Chirp)
ax1 = plt.subplot(2, 2, 1)
ax1.plot(t_full * 1000, f_full / 1000, color='orange', lw=2)
ax1.set_title(f"1. Frequency Sweep (Symbol {S})")
ax1.set_ylabel("Freq (kHz)")
ax1.set_xlabel("Time (ms)")
ax1.grid(True, alpha=0.3)

# Plot B: Time Domain (The Wiggles)
ax2 = plt.subplot(2, 2, 3)
ax2.plot(t_full * 1000, I_full, label='I', color='blue', alpha=0.8)
ax2.plot(t_full * 1000, Q_full, label='Q', color='red', alpha=0.5)
ax2.set_title("2. IQ Waves (Zoomed View)")
ax2.set_xlim(0.1, 0.25) # Zoom to see the phase relationship
ax2.set_xlabel("Time (ms)")
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot C: The 3D "Time-Spring"
ax3 = fig.add_subplot(1, 2, 2, projection='3d')
sc = ax3.scatter(I_slice, Q_slice, t_slice * 1000, c=t_slice, cmap='plasma', s=10)
ax3.set_xlabel('I')
ax3.set_ylabel('Q')
ax3.set_zlabel('Time (ms)')
ax3.set_title("3. The IQ Spring (First 15% of Symbol)")

# Add a dashed reference circle at the bottom
theta = np.linspace(0, 2*np.pi, 100)
ax3.plot(np.cos(theta), np.sin(theta), 0, color='black', linestyle='--', alpha=0.3)

plt.tight_layout()
plt.show()