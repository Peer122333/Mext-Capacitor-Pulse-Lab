"""
Separates Fenster für Parameter-Zeitverlauf (ESR und Kapazität).

Dieses Modul stellt ein separates Tkinter-Fenster bereit, das den
Zeitverlauf der berechneten Parameter (ESR und Kapazität) visualisiert.
Wird vom Control Center geöffnet und verwaltet.
"""

import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from typing import Callable, Optional


class ParamWindow:
    """
    Separates Fenster für Parameter-Zeitverlauf.
    
    Zeigt die Zeitverläufe von ESR (Equivalent Series Resistance) und
    Kapazität in zwei übereinander angeordneten Plots.
    
    Das Fenster wird automatisch aktualisiert, solange es geöffnet ist.
    """
    
    def __init__(self, parent, param_history_getter: Callable[[], list]):
        """
        Initialisiert das Parameter-Fenster.
        
        Parameters
        ----------
        parent : tk.Tk oder tk.Toplevel
            Parent-Fenster (wird für Positionierung verwendet).
        param_history_getter : callable
            Funktion, die eine Liste von (timestamp, esr, cap) Tuples zurückgibt.
            Wird periodisch aufgerufen für Updates.
        
        Examples
        --------
        >>> def get_history():
        ...     return [(time.time(), 0.1, 100e-6), ...]
        >>> win = ParamWindow(root, get_history)
        >>> win.show()
        """
        self.parent = parent
        self.param_history_getter = param_history_getter
        
        # Fenster erstellen
        self.window = tk.Toplevel(parent)
        self.window.title("Parameter-Zeitverlauf")
        self.window.geometry("800x600")
        
        # Plot erstellen
        fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_esr = fig.add_subplot(2, 1, 1)
        self.ax_cap = fig.add_subplot(2, 1, 2, sharex=self.ax_esr)
        
        self.ax_esr.set_ylabel("ESR [Ω]", fontsize=12)
        self.ax_cap.set_ylabel("Kapazität [µF]", fontsize=12)
        self.ax_cap.set_xlabel("Zeit t [s]", fontsize=12)
        self.ax_esr.grid(True, alpha=0.3)
        self.ax_cap.grid(True, alpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(fig, self.window)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Update-Timer
        self.update_timer_id = None
        self.is_updating = False
        
        # Starte Updates
        self._start_updates()
        
        # Cleanup beim Schließen
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def _start_updates(self):
        """
        Startet die periodischen Plot-Updates.
        
        Die Funktion wird automatisch aufgerufen und ruft sich selbst
        alle 1 Sekunde erneut auf, solange das Fenster geöffnet ist.
        """
        if not self.window.winfo_exists():
            return
        
        self._update_plot()
        self.update_timer_id = self.window.after(1000, self._start_updates)
    
    def _update_plot(self):
        """
        Aktualisiert die Plots mit den neuesten Parametern.
        
        Liest die Parameter-Historie über die getter-Funktion und
        visualisiert sie in den Plots.
        """
        if self.is_updating:
            return
        
        self.is_updating = True
        
        try:
            # Historie holen
            history = self.param_history_getter()
            
            if len(history) == 0:
                # Keine Daten: leere Plots zeigen
                self.ax_esr.clear()
                self.ax_cap.clear()
                self.ax_esr.set_ylabel("ESR [Ω]", fontsize=12)
                self.ax_cap.set_ylabel("Kapazität [µF]", fontsize=12)
                self.ax_cap.set_xlabel("Zeit t [s]", fontsize=12)
                self.ax_esr.grid(True, alpha=0.3)
                self.ax_cap.grid(True, alpha=0.3)
                self.canvas.draw()
                return
            
            # Daten extrahieren
            timestamps = np.array([ts for ts, _, _ in history])
            esr_vals = np.array([esr for _, esr, _ in history])
            cap_vals = np.array([cap for _, _, cap in history])
            
            # Relative Zeit (erster Eintrag = 0)
            if len(timestamps) > 0:
                timestamps = timestamps - timestamps[0]
            
            # Plots aktualisieren
            self.ax_esr.clear()
            self.ax_cap.clear()
            
            # ESR-Plot
            self.ax_esr.plot(timestamps, esr_vals, linewidth=1.5, color='blue', label='ESR')
            self.ax_esr.set_ylabel("ESR [Ω]", fontsize=12)
            self.ax_esr.grid(True, alpha=0.3)
            self.ax_esr.legend()
            
            # Kapazitäts-Plot (in µF)
            self.ax_cap.plot(timestamps, cap_vals * 1e6, linewidth=1.5, color='green', label='Kapazität')
            self.ax_cap.set_ylabel("Kapazität [µF]", fontsize=12)
            self.ax_cap.set_xlabel("Zeit t [s]", fontsize=12)
            self.ax_cap.grid(True, alpha=0.3)
            self.ax_cap.legend()
            
            # Plots anzeigen
            self.canvas.draw()
        
        except Exception as e:
            # Fehler ignorieren (z.B. wenn Fenster geschlossen wird)
            pass
        
        finally:
            self.is_updating = False
    
    def on_close(self):
        """
        Wird aufgerufen beim Schließen des Fensters.
        
        Stoppt die Update-Timer und zerstört das Fenster.
        """
        if self.update_timer_id:
            self.window.after_cancel(self.update_timer_id)
        self.window.destroy()
    
    def show(self):
        """
        Zeigt das Fenster an.
        
        Das Fenster ist bereits sichtbar nach __init__, aber diese
        Funktion kann verwendet werden, um es in den Vordergrund zu bringen.
        """
        self.window.lift()
        self.window.focus_force()

