import numpy as np
import matplotlib.pyplot as plt

# Parameters for the sine wave
amplitude = 1      # Amplitude of the sine wave
frequency = 1      # Frequency in Hz
phase = 0          # Phase shift in radians
sampling_rate = 1000  # Sampling rate in Hz
duration = 2       # Duration of the signal in seconds

# Generate time values
t = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False)

# Generate sine wave values
y = amplitude * np.sin(2 * np.pi * frequency * t + phase)

# Plot the sine wave
plt.figure(figsize=(10, 4))
plt.plot(t, y, label=f'{frequency} Hz Sine Wave')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.title('Sine Wave')
plt.legend()
plt.grid(True)
plt.show()
