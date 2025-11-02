import numpy as np
import matplotlib as plt

def plot_fft(y, fs, title="FFT"):
    N = len(y)
    if N == 0 or not fs:
        return
    win = np.hanning(N)
    Y = np.fft.rfft(y * win)
    freqs = np.fft.rfftfreq(N, 1.0 / fs)
    amp = np.abs(Y) / N * 2.0
    plt.figure()
    plt.semilogy(freqs, amp)
    plt.xlabel("Frequenz [Hz]")
    plt.ylabel("Amplitude")
    plt.title(title)
    plt.grid(True)