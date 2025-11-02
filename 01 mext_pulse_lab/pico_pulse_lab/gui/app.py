"""
Vollständiges Control Center für Pulse Lab Messungen.

Diese GUI integriert:
- STM32 UART Steuerung (beide Timer nebeneinander)
- Picoscope-Messung mit Live-Konfiguration
- Temperatur-Logger mit Live-Monitoring
- Live-Plots (U/I übereinander, Temperatur)
- Automatische Parameter-Berechnung (ESR, Kapazität)
- Speicherung (.npz + optional CSV)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import os
import sys
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from typing import Optional, Callable

# Python-Pfad korrigieren: Füge das übergeordnete Verzeichnis hinzu
# damit pico_pulse_lab als Modul gefunden wird
# Struktur: 01 mext_pulse_lab/pico_pulse_lab/gui/app.py
#           -> 01 mext_pulse_lab/ muss im sys.path sein
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(os.path.dirname(_current_dir))  # Zwei Ebenen hoch: gui -> pico_pulse_lab -> 01 mext_pulse_lab
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# Imports für Pulse Lab Module
from pico_pulse_lab.control.stm32_uart import NucleoUART
from pico_pulse_lab.acquisition.picoscope_reader import PicoReader
from pico_pulse_lab.acquisition.temp_logger import TempLogger
from pico_pulse_lab.processing.cap_params import estimate_cap_params


class App:
    """
    Hauptklasse für das Pulse Lab Control Center.
    
    Layout:
    - Links: STM32-Steuerung (beide Timer nebeneinander)
    - Rechts: Messteuerung (Picoscope + Temp-Logger)
    - Unten: Live-Plots (U/I links übereinander, Temperatur rechts, Parameter darunter)
    """
    
    def __init__(self, root):
        """
        Initialisiert das Control Center.
        
        Parameters
        ----------
        root : tk.Tk
            Hauptfenster der Anwendung.
        """
        self.root = root
        self.root.title("Pulse Lab Control Center")
        self.root.geometry("1400x900")
        
        # STM32 UART
        self.nuc = None
        self._rx_q = queue.Queue()
        self._reader_stop = True
        
        # Picoscope
        self.pico_reader: Optional[PicoReader] = None
        self.pico_thread: Optional[threading.Thread] = None
        self.pico_queue = queue.Queue()  # Für Pulse-Updates
        
        # Temp-Logger
        self.temp_logger: Optional[TempLogger] = None
        self.temp_queue = queue.Queue()  # Für Temperatur-Updates
        
        # Live-Daten (thread-sicher)
        self.latest_pulse = None  # (pulse_id, t, u, i)
        self.pulse_count = 0
        self.latest_params = None  # (esr, cap, timestamp)
        self.param_history = []  # Liste von (timestamp, esr, cap)
        
        # Parameter-Berechnungs-Timer
        self.param_timer_active = False
        
        # Build UI
        self._build_ui()
        self._set_connected(False)
        
        # Start Queue-Drainer für Thread-zu-GUI Kommunikation
        self.root.after(100, self._drain_queues)
        
        # Start Parameter-Berechnung Timer (alle 2 Sekunden)
        self._start_param_calculation()
    
    def _build_ui(self):
        """
        Baut die komplette GUI auf.
        
        Layout:
        - Zeile 0: Verbindung (spaltenübergreifend)
        - Zeile 1: Links STM32, Rechts Messteuerung
        - Zeile 2+: Plots unten
        """
        pad = {"padx": 6, "pady": 4}
        
        # ============ Zeile 0: Verbindung (spaltenübergreifend) ============
        frm_conn = ttk.LabelFrame(self.root, text="Verbindung")
        frm_conn.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)
        
        ttk.Label(frm_conn, text="Port:").grid(row=0, column=0, sticky="w")
        self.cmb_port = ttk.Combobox(frm_conn, width=28, state="readonly")
        self.cmb_port.grid(row=0, column=1, sticky="ew")
        ttk.Button(frm_conn, text="↻", width=3, command=self.refresh_ports).grid(row=0, column=2)
        ttk.Label(frm_conn, text="Baud:").grid(row=0, column=3, sticky="w")
        self.cmb_baud = ttk.Combobox(frm_conn, width=8, state="readonly",
                                     values=["115200", "230400", "460800", "921600"])
        self.cmb_baud.set("115200")
        self.cmb_baud.grid(row=0, column=4, sticky="w")
        
        self.btn_connect = ttk.Button(frm_conn, text="Verbinden", command=self.connect)
        self.btn_disconnect = ttk.Button(frm_conn, text="Trennen", command=self.disconnect)
        self.btn_connect.grid(row=0, column=5, padx=(12, 0))
        self.btn_disconnect.grid(row=0, column=6)
        
        frm_conn.columnconfigure(1, weight=1)
        
        # ============ Zeile 1: Links STM32, Rechts Messteuerung ============
        
        # --- Links: STM32-Steuerung ---
        frm_stm32 = ttk.LabelFrame(self.root, text="STM32 Steuerung")
        frm_stm32.grid(row=1, column=0, sticky="nsew", **pad)
        
        # Timer 1 und Timer 2 nebeneinander
        # Timer 1 (links)
        frm_t1 = ttk.LabelFrame(frm_stm32, text="Timer 1 (µs)")
        frm_t1.grid(row=0, column=0, sticky="ew", **pad)
        
        ttk.Label(frm_t1, text="Periode (µs):").grid(row=0, column=0, sticky="e")
        self.ent_period_t1 = ttk.Entry(frm_t1, width=12)
        self.ent_period_t1.insert(0, "200")
        self.ent_period_t1.grid(row=0, column=1, sticky="w")
        
        self.btn_set_t1 = ttk.Button(frm_t1, text="SET T1", command=lambda: self.on_set_timer(1))
        self.btn_set_t1.grid(row=0, column=2, padx=(12, 0))
        
        # Timer 2 (rechts)
        frm_t2 = ttk.LabelFrame(frm_stm32, text="Timer 2 (ms)")
        frm_t2.grid(row=0, column=1, sticky="ew", **pad)
        
        ttk.Label(frm_t2, text="Periode (ms):").grid(row=0, column=0, sticky="e")
        self.ent_period_t2 = ttk.Entry(frm_t2, width=12)
        self.ent_period_t2.insert(0, "10")
        self.ent_period_t2.grid(row=0, column=1, sticky="w")
        
        self.btn_set_t2 = ttk.Button(frm_t2, text="SET T2", command=lambda: self.on_set_timer(2))
        self.btn_set_t2.grid(row=0, column=2, padx=(12, 0))
        
        frm_stm32.columnconfigure(0, weight=1)
        frm_stm32.columnconfigure(1, weight=1)
        
        # Sequenz (unter beiden Timern)
        frm_seq = ttk.LabelFrame(frm_stm32, text="Sequenz")
        frm_seq.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)
        
        ttk.Label(frm_seq, text="Pulse-Count (0=endlos):").grid(row=0, column=0, sticky="e")
        self.ent_pulses = ttk.Entry(frm_seq, width=10)
        self.ent_pulses.insert(0, "0")
        self.ent_pulses.grid(row=0, column=1, sticky="w")
        
        self.btn_start = ttk.Button(frm_seq, text="START", command=self.on_start)
        self.btn_start.grid(row=0, column=2, padx=(12, 0))
        
        ttk.Label(frm_seq, text="Stop-Mode:").grid(row=0, column=3, sticky="e")
        self.stop_mode = ttk.Combobox(frm_seq, state="readonly", width=8, values=["Soft", "Hard"])
        self.stop_mode.set("Soft")
        self.stop_mode.grid(row=0, column=4, sticky="w")
        
        self.btn_stop = ttk.Button(frm_seq, text="STOP", command=self.on_stop)
        self.btn_stop.grid(row=0, column=5)
        
        # Readback
        frm_rb = ttk.LabelFrame(frm_stm32, text="Readback")
        frm_rb.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
        
        self.btn_rb_t1 = ttk.Button(frm_rb, text="READBACK T1", command=lambda: self.on_readback(1))
        self.btn_rb_t2 = ttk.Button(frm_rb, text="READBACK T2", command=lambda: self.on_readback(2))
        self.btn_rb_t1.grid(row=0, column=0, padx=(0, 6))
        self.btn_rb_t2.grid(row=0, column=1, padx=(6, 0))
        
        # Log für STM32
        frm_log = ttk.LabelFrame(frm_stm32, text="Log")
        frm_log.grid(row=3, column=0, columnspan=2, sticky="nsew", **pad)
        
        self.txt_log = tk.Text(frm_log, height=8, width=50)
        self.txt_log.grid(row=0, column=0, sticky="nsew")
        self.txt_log.configure(state="disabled")
        scroll = ttk.Scrollbar(frm_log, command=self.txt_log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.txt_log["yscrollcommand"] = scroll.set
        
        frm_log.rowconfigure(0, weight=1)
        frm_log.columnconfigure(0, weight=1)
        
        # --- Rechts: Messteuerung ---
        frm_measure = ttk.LabelFrame(self.root, text="Messteuerung")
        frm_measure.grid(row=1, column=1, sticky="nsew", **pad)
        
        # Run-Name und Speicherung
        frm_run = ttk.LabelFrame(frm_measure, text="Run-Konfiguration")
        frm_run.grid(row=0, column=0, sticky="ew", **pad)
        
        ttk.Label(frm_run, text="Run-Name:").grid(row=0, column=0, sticky="e")
        self.ent_run_name = ttk.Entry(frm_run, width=20)
        self.ent_run_name.insert(0, "run_01")
        self.ent_run_name.grid(row=0, column=1, sticky="w")
        
        self.chk_save_csv = ttk.Checkbutton(frm_run, text="CSV speichern", state="normal")
        self.chk_save_csv.grid(row=0, column=2, padx=(12, 0))
        self.chk_save_csv.state(['selected'])  # Default aktiviert
        
        # Picoscope-Konfiguration
        frm_pico = ttk.LabelFrame(frm_measure, text="Picoscope")
        frm_pico.grid(row=1, column=0, sticky="ew", **pad)
        
        # Abtastrate
        ttk.Label(frm_pico, text="Target FS (MS/s):").grid(row=0, column=0, sticky="e")
        self.ent_target_fs = ttk.Entry(frm_pico, width=12)
        self.ent_target_fs.insert(0, "20")
        self.ent_target_fs.grid(row=0, column=1, sticky="w")
        
        # Trigger
        ttk.Label(frm_pico, text="Trigger (V):").grid(row=0, column=2, sticky="e")
        self.ent_trig_level = ttk.Entry(frm_pico, width=10)
        self.ent_trig_level.insert(0, "-0.2")
        self.ent_trig_level.grid(row=0, column=3, sticky="w")
        
        # Kanal A (Spannung)
        ttk.Label(frm_pico, text="CH A (Spannung):").grid(row=1, column=0, sticky="e")
        self.cmb_coupling_a = ttk.Combobox(frm_pico, state="readonly", width=8, values=["AC", "DC"])
        self.cmb_coupling_a.set("AC")
        self.cmb_coupling_a.grid(row=1, column=1, sticky="w")
        
        self.cmb_range_a = ttk.Combobox(frm_pico, state="readonly", width=8,
                                        values=["20MV", "50MV", "100MV", "200MV", "500MV", "1V", "2V", "5V", "10V", "20V", "50V"])
        self.cmb_range_a.set("50MV")
        self.cmb_range_a.grid(row=1, column=2, sticky="w")
        
        # Kanal B (Strom)
        ttk.Label(frm_pico, text="CH B (Strom):").grid(row=2, column=0, sticky="e")
        self.cmb_coupling_b = ttk.Combobox(frm_pico, state="readonly", width=8, values=["AC", "DC"])
        self.cmb_coupling_b.set("AC")
        self.cmb_coupling_b.grid(row=2, column=1, sticky="w")
        
        self.cmb_range_b = ttk.Combobox(frm_pico, state="readonly", width=8,
                                         values=["20MV", "50MV", "100MV", "200MV", "500MV", "1V", "2V", "5V", "10V", "20V", "50V"])
        self.cmb_range_b.set("10V")
        self.cmb_range_b.grid(row=2, column=2, sticky="w")
        
        # Picoscope-Buttons
        self.btn_pico_start = ttk.Button(frm_pico, text="Start Messung", command=self.on_pico_start)
        self.btn_pico_start.grid(row=3, column=0, columnspan=2, pady=(4, 0))
        
        self.btn_pico_stop = ttk.Button(frm_pico, text="Stop Messung", command=self.on_pico_stop)
        self.btn_pico_stop.grid(row=3, column=2, columnspan=2, pady=(4, 0))
        
        # Temp-Logger
        frm_temp = ttk.LabelFrame(frm_measure, text="Temperatur-Logger")
        frm_temp.grid(row=2, column=0, sticky="ew", **pad)
        
        ttk.Label(frm_temp, text="Update-Interval (s):").grid(row=0, column=0, sticky="e")
        self.ent_temp_interval = ttk.Entry(frm_temp, width=10)
        self.ent_temp_interval.insert(0, "0.5")
        self.ent_temp_interval.grid(row=0, column=1, sticky="w")
        
        self.btn_temp_start = ttk.Button(frm_temp, text="Start", command=self.on_temp_start)
        self.btn_temp_stop = ttk.Button(frm_temp, text="Stop", command=self.on_temp_stop)
        self.btn_temp_start.grid(row=0, column=2, padx=(12, 0))
        self.btn_temp_stop.grid(row=0, column=3)
        
        # Status-Anzeige
        frm_status = ttk.LabelFrame(frm_measure, text="Status")
        frm_status.grid(row=3, column=0, sticky="ew", **pad)
        
        self.lbl_pulse_count = ttk.Label(frm_status, text="Pulse: 0")
        self.lbl_pulse_count.grid(row=0, column=0, sticky="w")
        
        self.lbl_temp = ttk.Label(frm_status, text="Temp: -- °C")
        self.lbl_temp.grid(row=0, column=1, sticky="w", padx=(12, 0))
        
        # ============ Zeile 2+: Plots ============
        frm_plots = ttk.Frame(self.root)
        frm_plots.grid(row=2, column=0, columnspan=2, sticky="nsew", **pad)
        
        # Links: U/I-Plots übereinander
        frm_ui_plots = ttk.LabelFrame(frm_plots, text="Spannung & Strom")
        frm_ui_plots.grid(row=0, column=0, sticky="nsew", **pad)
        
        fig_ui = Figure(figsize=(6, 6), dpi=100)
        self.ax_u = fig_ui.add_subplot(2, 1, 1)
        self.ax_i = fig_ui.add_subplot(2, 1, 2, sharex=self.ax_u)
        
        self.ax_u.set_ylabel("Spannung U [V]")
        self.ax_i.set_ylabel("Strom I [A]")
        self.ax_i.set_xlabel("Zeit t [s]")
        self.ax_u.grid(True, alpha=0.3)
        self.ax_i.grid(True, alpha=0.3)
        
        self.canvas_ui = FigureCanvasTkAgg(fig_ui, frm_ui_plots)
        self.canvas_ui.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Rechts: Temperatur + Parameter
        frm_right_plots = ttk.Frame(frm_plots)
        frm_right_plots.grid(row=0, column=1, sticky="nsew", **pad)
        
        # Temperatur-Plot
        frm_temp_plot = ttk.LabelFrame(frm_right_plots, text="Temperatur")
        frm_temp_plot.grid(row=0, column=0, sticky="nsew", **pad)
        
        fig_temp = Figure(figsize=(4, 3), dpi=100)
        self.ax_temp = fig_temp.add_subplot(1, 1, 1)
        self.ax_temp.set_ylabel("Temperatur [°C]")
        self.ax_temp.set_xlabel("Zeit t [s]")
        self.ax_temp.grid(True, alpha=0.3)
        
        self.canvas_temp = FigureCanvasTkAgg(fig_temp, frm_temp_plot)
        self.canvas_temp.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Parameter-Anzeige
        frm_params = ttk.LabelFrame(frm_right_plots, text="Parameter")
        frm_params.grid(row=1, column=0, sticky="ew", **pad)
        
        self.lbl_esr = ttk.Label(frm_params, text="ESR: -- Ω", font=("Arial", 12))
        self.lbl_esr.grid(row=0, column=0, sticky="w", padx=8, pady=4)
        
        self.lbl_cap = ttk.Label(frm_params, text="C: -- µF", font=("Arial", 12))
        self.lbl_cap.grid(row=1, column=0, sticky="w", padx=8, pady=4)
        
        self.btn_param_window = ttk.Button(frm_params, text="Parameter-Zeitverlauf öffnen",
                                           command=self.open_param_window)
        self.btn_param_window.grid(row=2, column=0, pady=4)
        
        frm_plots.columnconfigure(0, weight=2)  # U/I-Plots nehmen mehr Platz
        frm_plots.columnconfigure(1, weight=1)  # Temperatur/Parameter rechts
        
        # Grid-Konfiguration
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        
        # Serial Monitor (optional, klein am Ende)
        frm_mon = ttk.LabelFrame(self.root, text="Serial Monitor (RX)")
        frm_mon.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)
        
        self.txt_mon = tk.Text(frm_mon, height=4, width=90)
        self.txt_mon.grid(row=0, column=0, sticky="ew")
        self.txt_mon.configure(state="disabled")
        
        frm_mon.columnconfigure(0, weight=1)
        
        # Initialisierung
        self.refresh_ports()
        
        # Parameter-Fenster (wird bei Bedarf erstellt)
        self.param_window = None
    
    # ============ STM32-Funktionen (bestehend) ============
    
    def _set_connected(self, ok: bool):
        """Aktiviert/deaktiviert STM32-Buttons basierend auf Verbindungsstatus."""
        self.btn_connect.configure(state=("disabled" if ok else "normal"))
        self.btn_disconnect.configure(state=("normal" if ok else "disabled"))
        state = "normal" if ok else "disabled"
        for w in [self.btn_set_t1, self.btn_set_t2, self.btn_start, self.btn_stop,
                  self.btn_rb_t1, self.btn_rb_t2]:
            w.configure(state=state)
    
    def log(self, msg: str):
        """Schreibt eine Nachricht in das Log-Fenster."""
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", msg + "\n")
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")
    
    def refresh_ports(self):
        """Aktualisiert die Liste der verfügbaren COM-Ports."""
        ports = []
        if list_ports:
            ports = [p.device for p in list_ports.comports()]
        if not ports:
            ports = ["/dev/tty.usbmodem1103", "/dev/tty.usbserial", "/dev/cu.usbmodem", "/dev/cu.usbserial"]
        self.cmb_port["values"] = ports
        if ports and not self.cmb_port.get():
            self.cmb_port.set(ports[0])
    
    def connect(self):
        """Stellt Verbindung zum STM32 über UART her."""
        port = self.cmb_port.get().strip()
        baud = int(self.cmb_baud.get().strip())
        if not port:
            messagebox.showwarning("Hinweis", "Bitte zuerst einen Port auswählen.")
            return
        try:
            self.nuc = NucleoUART(port=port, baudrate=baud, timeout=1.0)
            self._set_connected(True)
            self.log(f"[OK] Verbunden mit {port} @ {baud} Baud")
            self._reader_stop = False
            threading.Thread(target=self._serial_monitor_reader, daemon=True).start()
            self.root.after(50, self._drain_monitor_queue)
        except Exception as e:
            messagebox.showerror("Verbindung fehlgeschlagen", str(e))
            self.log(f"[ERR] {e}")
    
    def disconnect(self):
        """Trennt die UART-Verbindung."""
        self._reader_stop = True
        if self.nuc:
            try:
                self.nuc.close()
            except Exception:
                pass
            self.nuc = None
        self._set_connected(False)
        self.log("[i] Verbindung getrennt")
    
    def _in_thread(self, target):
        """Startet eine Funktion in einem separaten Thread."""
        threading.Thread(target=target, daemon=True).start()
    
    def on_set_timer(self, timer: int):
        """SET-Befehl für Timer 1 oder 2."""
        def work():
            if not self.nuc:
                return
            try:
                period = int(self.ent_period_t1.get() if timer == 1 else self.ent_period_t2.get())
                self.log(f"> SET T{timer} period={period}")
                self.nuc.set_timer(timer, period)
            except Exception as e:
                self.log(f"[ERR] SET T{timer}: {e}")
        self._in_thread(work)
    
    def on_start(self):
        """START-Befehl für die Sequenz."""
        def work():
            if not self.nuc:
                return
            try:
                pulses = int(self.ent_pulses.get())
                self.log(f"> START sequence pulse_count={pulses}")
                self.nuc.start_sequence(pulses)
            except Exception as e:
                self.log(f"[ERR] START: {e}")
        self._in_thread(work)
    
    def on_stop(self):
        """STOP-Befehl für die Sequenz."""
        def work():
            if not self.nuc:
                return
            try:
                mode = self.stop_mode.get()
                hard = (mode == "Hard")
                self.log(f"> STOP ({mode})")
                self.nuc.stop_timer(hard=hard, timer_for_cmd=1)
            except Exception as e:
                self.log(f"[ERR] STOP: {e}")
        self._in_thread(work)
    
    def on_readback(self, timer: int):
        """READBACK für Timer 1 oder 2."""
        def work():
            if not self.nuc:
                return
            try:
                val, flags = self.nuc.readback(timer)
                unit = "µs" if timer == 1 else "ms"
                self.log(f"< READBACK T{timer}: {val} {unit}, flags=0x{flags:02X}")
            except Exception as e:
                self.log(f"[ERR] READBACK T{timer}: {e}")
        self._in_thread(work)
    
    # ============ Picoscope-Funktionen ============
    
    def on_pico_start(self):
        """Startet eine Picoscope-Messung in separatem Thread."""
        if self.pico_reader and self.pico_reader.is_running:
            messagebox.showwarning("Hinweis", "Messung läuft bereits.")
            return
        
        run_name = self.ent_run_name.get().strip()
        if not run_name:
            messagebox.showwarning("Hinweis", "Bitte Run-Name eingeben.")
            return
        
        # Konfiguration aus GUI lesen
        try:
            target_fs = float(self.ent_target_fs.get()) * 1e6  # MS/s -> Hz
            trigger_level = float(self.ent_trig_level.get())
            
            # Reader erstellen und konfigurieren
            self.pico_reader = PicoReader()
            self.pico_reader.configure(
                run_name=run_name,
                target_fs=target_fs,
                trigger_level_v=trigger_level,
                coupling_a=self.cmb_coupling_a.get(),
                range_a=self.cmb_range_a.get(),
                coupling_b=self.cmb_coupling_b.get(),
                range_b=self.cmb_range_b.get()
            )
            
            # Callback setzen
            self.pico_reader.set_callback(self._on_pico_pulse)
            
            # Thread starten
            save_csv = self.chk_save_csv.instate(['selected'])
            self.pico_thread = threading.Thread(
                target=self._pico_measurement_thread,
                args=(save_csv,),
                daemon=True
            )
            self.pico_thread.start()
            
            self.btn_pico_start.configure(state="disabled")
            self.log(f"[Pico] Messung gestartet: {run_name}")
        
        except Exception as e:
            messagebox.showerror("Fehler", f"Fehler beim Starten der Messung: {e}")
            self.log(f"[ERR] Pico-Start: {e}")
    
    def _pico_measurement_thread(self, save_csv: bool):
        """Thread-Funktion für Picoscope-Messung."""
        try:
            # Endlos-Messung (kann später erweitert werden)
            self.pico_reader.start_measurement(
                n_pulses=1000,  # Groß genug für praktisch endlos
                save_csv=save_csv,
                save_npz=True
            )
        except Exception as e:
            self.pico_queue.put(("error", str(e)))
    
    def _on_pico_pulse(self, pulse_id: int, t: np.ndarray, u: np.ndarray, i: np.ndarray):
        """Callback für jeden erfassten Puls (wird vom Reader aufgerufen)."""
        self.pico_queue.put(("pulse", (pulse_id, t, u, i)))
    
    def on_pico_stop(self):
        """Stoppt die Picoscope-Messung."""
        if self.pico_reader:
            self.pico_reader.stop()
            self.btn_pico_start.configure(state="normal")
            self.log("[Pico] Messung gestoppt")
    
    # ============ Temp-Logger-Funktionen ============
    
    def on_temp_start(self):
        """Startet den Temperatur-Logger."""
        try:
            interval = float(self.ent_temp_interval.get())
            self.temp_logger = TempLogger(update_interval_s=interval)
            
            # Callback setzen
            self.temp_logger.set_callback(self._on_temp_update)
            
            self.temp_logger.start()
            self.btn_temp_start.configure(state="disabled")
            self.log("[Temp] Temperaturmessung gestartet")
        
        except Exception as e:
            messagebox.showerror("Fehler", f"Fehler beim Starten: {e}")
            self.log(f"[ERR] Temp-Start: {e}")
    
    def _on_temp_update(self, channel: int, temp: float, timestamp: float):
        """Callback für Temperatur-Updates."""
        self.temp_queue.put(("temp", (channel, temp, timestamp)))
    
    def on_temp_stop(self):
        """Stoppt den Temperatur-Logger."""
        if self.temp_logger:
            self.temp_logger.stop()
            self.btn_temp_start.configure(state="normal")
            self.log("[Temp] Temperaturmessung gestoppt")
    
    # ============ Queue-Drainer und Updates ============
    
    def _drain_queues(self):
        """Drainiert alle Queues und aktualisiert die GUI (wird periodisch aufgerufen)."""
        # Picoscope-Updates
        try:
            while True:
                msg_type, data = self.pico_queue.get_nowait()
                if msg_type == "pulse":
                    pulse_id, t, u, i = data
                    self.latest_pulse = (pulse_id, t, u, i)
                    self.pulse_count += 1
                    self.lbl_pulse_count.configure(text=f"Pulse: {self.pulse_count}")
                    self._update_ui_plots()
                elif msg_type == "error":
                    self.log(f"[ERR] Pico: {data}")
                    self.btn_pico_start.configure(state="normal")
        except queue.Empty:
            pass
        
        # Temperatur-Updates
        try:
            while True:
                msg_type, data = self.temp_queue.get_nowait()
                if msg_type == "temp":
                    channel, temp, timestamp = data
                    self.lbl_temp.configure(text=f"Temp: {temp:.2f} °C")
                    self._update_temp_plot()
        except queue.Empty:
            pass
        
        # Serial Monitor
        try:
            while True:
                line = self._rx_q.get_nowait()
                self.txt_mon.configure(state="normal")
                self.txt_mon.insert("end", line + "\n")
                self.txt_mon.see("end")
                self.txt_mon.configure(state="disabled")
        except queue.Empty:
            pass
        
        # Wieder aufrufen
        self.root.after(100, self._drain_queues)
    
    def _update_ui_plots(self):
        """Aktualisiert die U/I-Plots mit dem neuesten Puls."""
        if self.latest_pulse is None:
            return
        
        pulse_id, t, u, i = self.latest_pulse
        
        # Plots aktualisieren
        self.ax_u.clear()
        self.ax_i.clear()
        
        self.ax_u.plot(t, u, linewidth=1.0, label=f"U (pulse {pulse_id})")
        self.ax_i.plot(t, i, linewidth=1.0, label=f"I (pulse {pulse_id})")
        
        self.ax_u.set_ylabel("Spannung U [V]")
        self.ax_i.set_ylabel("Strom I [A]")
        self.ax_i.set_xlabel("Zeit t [s]")
        self.ax_u.grid(True, alpha=0.3)
        self.ax_i.grid(True, alpha=0.3)
        self.ax_u.legend()
        self.ax_i.legend()
        
        self.canvas_ui.draw()
    
    def _update_temp_plot(self):
        """Aktualisiert den Temperatur-Plot."""
        if not self.temp_logger:
            return
        
        ts, temps = self.temp_logger.get_temperature_history(channel=1, max_points=200)
        
        if len(ts) == 0:
            return
        
        self.ax_temp.clear()
        self.ax_temp.plot(ts, temps, linewidth=1.0, color='red')
        self.ax_temp.set_ylabel("Temperatur [°C]")
        self.ax_temp.set_xlabel("Zeit t [s]")
        self.ax_temp.grid(True, alpha=0.3)
        
        self.canvas_temp.draw()
    
    # ============ Parameter-Berechnung ============
    
    def _start_param_calculation(self):
        """Startet periodische Parameter-Berechnung (alle 2 Sekunden)."""
        self.param_timer_active = True
        self._calculate_params()
    
    def _calculate_params(self):
        """Berechnet ESR und Kapazität aus dem neuesten Puls."""
        if not self.param_timer_active:
            return
        
        if self.latest_pulse is None:
            # Nächstes Mal versuchen
            self.root.after(2000, self._calculate_params)
            return
        
        try:
            pulse_id, t, u, i = self.latest_pulse
            
            # Parameter berechnen
            esr, cap = estimate_cap_params(t, u, i)
            
            # Parameter speichern
            self.latest_params = (esr, cap, time.time())
            self.param_history.append((time.time(), esr, cap))
            
            # Historie begrenzen
            if len(self.param_history) > 1000:
                self.param_history = self.param_history[-1000:]
            
            # GUI aktualisieren
            self.lbl_esr.configure(text=f"ESR: {esr:.6f} Ω")
            self.lbl_cap.configure(text=f"C: {cap*1e6:.6f} µF")
        
        except Exception as e:
            # Fehler ignorieren (z.B. wenn Daten noch nicht ausreichend)
            pass
        
        # Wieder aufrufen in 2 Sekunden
        self.root.after(2000, self._calculate_params)
    
    def open_param_window(self):
        """Öffnet separates Fenster für Parameter-Zeitverlauf."""
        from pico_pulse_lab.gui.param_window import ParamWindow
        
        if self.param_window is not None:
            try:
                self.param_window.window.lift()
                return
            except:
                pass
        
        # Neues Fenster mit ParamWindow-Klasse
        def get_history():
            """Getter-Funktion für Parameter-Historie."""
            return self.param_history
        
        self.param_window = ParamWindow(self.root, get_history)
        
        # Cleanup beim Schließen
        def on_close():
            self.param_window = None
        
        self.param_window.window.protocol("WM_DELETE_WINDOW", 
                                         lambda: (on_close(), self.param_window.on_close()))
    
    # ============ Serial Monitor ============
    
    def _drain_monitor_queue(self):
        """Drainiert die Serial-Monitor-Queue (separat, wird periodisch aufgerufen)."""
        try:
            while True:
                line = self._rx_q.get_nowait()
                self.txt_mon.configure(state="normal")
                self.txt_mon.insert("end", line + "\n")
                self.txt_mon.see("end")
                self.txt_mon.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(50, self._drain_monitor_queue)
    
    def _serial_monitor_reader(self):
        """Thread: Liest serielle Daten und schreibt in Queue."""
        ser = self.nuc.ser if (self.nuc and hasattr(self.nuc, "ser")) else None
        if not ser:
            return
        
        buf = bytearray()
        while not self._reader_stop and ser and ser.is_open:
            try:
                n = ser.in_waiting
                if n:
                    chunk = ser.read(n)
                    if not chunk:
                        continue
                    buf.extend(chunk)
                    
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        text = line.decode(errors="replace").strip()
                        if text:
                            self._rx_q.put(text)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self._rx_q.put(f"[RX-ERR] {e}")
                break


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
