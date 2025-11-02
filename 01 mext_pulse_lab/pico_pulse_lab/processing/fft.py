"""
FFT-Visualisierung für Puls-Messdaten.

Dieses Modul stellt Funktionen zur schnellen Fourier-Transformation (FFT)
und deren Visualisierung bereit. Nützlich zur Analyse der Frequenzanteile
in gemessenen Spannungs- und Strompulsen.
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_fft(y, fs, title="FFT"):
    """
    Berechnet und visualisiert das FFT-Amplitudenspektrum eines Signals.
    
    Die Funktion verwendet eine Hanning-Fensterung zur Reduzierung von
    Spektral-Leakage-Effekten und stellt das Ergebnis in logarithmischer
    Darstellung dar (semilogarithmisch).
    
    Parameters
    ----------
    y : np.ndarray
        Eingangssignal (Zeitbereich) als 1D-Array.
        Typischerweise Spannung (U) oder Strom (I) über die Zeit.
    fs : float
        Abtastfrequenz in Hz (Samples pro Sekunde).
        Wird benötigt für korrekte Frequenzachsen-Kalibrierung.
    title : str, optional
        Titel des Plots. Standard: "FFT".
        Sollte beschreiben, welches Signal dargestellt wird (z.B. "FFT U (pulse 1)").
    
    Returns
    -------
    None
        Die Funktion erstellt direkt einen Matplotlib-Plot. Kein Rückgabewert.
    
    Notes
    -----
    - Verwendet real-FFT (rfft), da das Signal reellwertig ist (effizienter).
    - Hanning-Fensterung reduziert Artefakte an den Signalrändern.
    - Amplitude wird normiert (durch N, dann mal 2 für einseitiges Spektrum).
    - Semilogarithmische Darstellung für bessere Sichtbarkeit verschiedener
      Frequenzkomponenten über mehrere Größenordnungen.
    
    Examples
    --------
    >>> import numpy as np
    >>> t = np.linspace(0, 1, 1000)
    >>> y = np.sin(2 * np.pi * 50 * t)  # 50 Hz Sinus
    >>> plot_fft(y, fs=1000, title="FFT 50 Hz Sinus")
    >>> plt.show()  # Plot anzeigen
    """
    # Länge des Signals prüfen
    N = len(y)
    if N == 0 or not fs:
        return  # Keine gültigen Daten
    
    # Hanning-Fenster anwenden (reduziert Spektral-Leakage)
    win = np.hanning(N)
    
    # Real-FFT berechnen (nur positive Frequenzen, da Signal reell)
    Y = np.fft.rfft(y * win)
    
    # Frequenzachse generieren (nur positive Frequenzen)
    freqs = np.fft.rfftfreq(N, 1.0 / fs)
    
    # Amplitude normieren: durch N für Parseval, dann *2 für einseitiges Spektrum
    amp = np.abs(Y) / N * 2.0
    
    # Plot erstellen
    plt.figure()
    plt.semilogy(freqs, amp)  # Logarithmische Y-Achse für bessere Darstellung
    plt.xlabel("Frequenz [Hz]")
    plt.ylabel("Amplitude")
    plt.title(title)
    plt.grid(True)  # Gitternetz für bessere Lesbarkeit