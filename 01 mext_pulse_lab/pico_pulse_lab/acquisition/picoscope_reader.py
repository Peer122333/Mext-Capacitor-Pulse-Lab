"""
PS3000A Acquisition – U/I synchron, Append-CSV pro Messlauf (Mehrfach-Pulse)
- Kanal A: Spannung (AC, kleiner Bereich)
- Kanal B: Rogowski (AC, optional Volt->Ampere)
- Jede Erfassung wird als neuer 'pulse_id' in EINE CSV angehängt
  Spalten: pulse_id, sample_idx, time_s, u_V, i_{V|A}
"""

import os
import sys
import time
import json
import ctypes as ct # C-Typen für Picoscope SDK
import numpy as np  
from datetime import datetime

# Python-Pfad korrigieren: Füge das übergeordnete Verzeichnis hinzu
# damit pico_pulse_lab als Modul gefunden wird
# Struktur: 01 mext_pulse_lab/pico_pulse_lab/acquisition/picoscope_reader.py
#           -> 01 mext_pulse_lab/ muss im sys.path sein
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(os.path.dirname(_current_dir))  # Zwei Ebenen hoch: acquisition -> pico_pulse_lab -> 01 mext_pulse_lab
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

# Optional: Picoscope SDK importieren (wenn verfügbar)
try:
    from picosdk.ps3000a import ps3000a as ps    # Picoscope PS3000A SDK 
    from picosdk.functions import assert_pico_ok # Fehlerprüfung SDK-Aufrufe
    PICO_SDK_AVAILABLE = True
except (ImportError, OSError) as e:
    # SDK nicht verfügbar (z.B. DLL nicht gefunden)
    print(f"[Warnung] PicoSDK nicht verfügbar: {e}")
    print("[Warnung] Picoscope-Funktionalität wird im Mock-Modus laufen")
    ps = None
    assert_pico_ok = lambda x: None  # Dummy-Funktion
    PICO_SDK_AVAILABLE = False

from pico_pulse_lab.storage.csv_writer import (
    ensure_csv,
    scan_next_pulse_id,
    append_pulse_to_csv,
    write_meta,
)


# ============================================================
# 1) KONFIGURATION
# ============================================================

# Messlauf
RUN_NAME            = "90V_DC_300A-3"   # Messlauf-Name (Ordner+Datei) Pulse_Test_30V_Source_1

# Trigger
TRIG_LEVEL_V        = -0.2              # Trigger auf CH A (AC), in Volt
AUTO_TRIG_MS        = 0                 # 0 = Warten auf echt Trigger, Zahl = auslösen nach definierter Dauer in ms

# Abtastung / Blocklänge
TARGET_FS           = 20e6              # gewünschte Abtastrate
PRETRIG_RATIO       = 0.2               # 20% vor Trigger
BASE_SAMPLE         = 400_000           # "sichtbares" Fenster nach Trigger
N_SAMPLES           = 400_000 + int(PRETRIG_RATIO * 400_000)   # Gesamtanzahl Samples
OVERSAMPLE          = 1                 # Anazhl der gemittelten ADC Werte pro Sample im Puffer (1 = kein Oversampling, 2 = Mittelung über 2 interne ADC-Samples, 4 = über 4 Samples)

# Anzahl Pulse pro Session
N_PULSES            = 3
INTER_PULSE_DELAY_S = 0.0            # z.B. 0.01 für 10 ms Pause

# Kanal A: Spannung (kleiner Bereich für höhere Auflösung)
# Werte werden nur gesetzt, wenn SDK verfügbar ist
if PICO_SDK_AVAILABLE:
    CH_A                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
    COUPLING_A          = ps.PS3000A_COUPLING["PS3000A_AC"]
    RANGE_A             = ps.PS3000A_RANGE["PS3000A_50MV"]   # ±2 V -> eigentlich sollte beim Spannungsmessung hier bei 1:100 kleinere Werte besser klappen
    CH_B                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_B"]
    COUPLING_B          = ps.PS3000A_COUPLING["PS3000A_AC"]
    RANGE_B             = ps.PS3000A_RANGE["PS3000A_10V"]   # anpassen
else:
    # Dummy-Werte für Mock-Modus
    CH_A = CH_B = 0
    COUPLING_A = COUPLING_B = 0
    RANGE_A = RANGE_B = 0

DC_OFFSET_A         = 0.0  # Gleichanteil-Offset
U_PROBE_ATTENUATION = 50.0 # 1:50 Tastkopf, heißt bei 10V => 0,2V an PicoScope
# u_real = u_measured * u_probe_attenuation

DC_OFFSET_B         = 0.0  # Gleichanteil-Offset
ROGOWSKI_V_PER_A    = 0.02          # V/A -- wenn None oder 0 -> CSV in Volt
# 1A = 0.02 V => i_real = 1/rogowski_v_per_a * u_measured


# Basisordner & Run-Verzeichnis
BASE_DIR   = r"C:\Users\mext\Documents\02 Python Schnittstelle STM32 serielle Steuerung\mext_cap_testbench_control_code\picoscope"
RUN_DIR    = os.path.join(BASE_DIR, "Runs", RUN_NAME) 
CSV_PATH   = os.path.join(RUN_DIR, f"{RUN_NAME}.csv")
META_PATH  = os.path.join(RUN_DIR, f"{RUN_NAME}.meta.json")
os.makedirs(RUN_DIR, exist_ok=True)


# ============================================================
# 2) HELFER
# ============================================================
def range_fullscale_volts(v_range_enum):
    """
    Mapping Pico-Range-Enum -> realer Messbereich in Volt.
    
    Quantisierungsauflösung
    -------------------------
    - ADC des 3205A hat 8 Bit Auflösung = 256 Stufen
    - zB delta_U = 100mV/256 = 0.39mV
    - zB delta_U = 40V/256 = 156mV

    Hinweise
    -------------------------
    - Effektive Auflösung (ENOB): 7.6 Bit
    - (durch Rauschen und Nichtlinearitäten etwas geringer als 8 Bit)
    
    Parameters
    ----------
    v_range_enum
        Pico-Range-Enum oder String (z.B. "50MV")
    
    Returns
    -------
    float
        Messbereich in Volt (z.B. 0.05 für 50MV)
    """
    # Dictionary für Range-Mapping (wenn SDK verfügbar)
    if PICO_SDK_AVAILABLE:
        table = {
            ps.PS3000A_RANGE["PS3000A_20MV"]: 0.02,
            ps.PS3000A_RANGE["PS3000A_50MV"]: 0.05,
            ps.PS3000A_RANGE["PS3000A_100MV"]: 0.1,
            ps.PS3000A_RANGE["PS3000A_200MV"]: 0.2,
            ps.PS3000A_RANGE["PS3000A_500MV"]: 0.5,
            ps.PS3000A_RANGE["PS3000A_1V"]: 1.0,
            ps.PS3000A_RANGE["PS3000A_2V"]: 2.0,
            ps.PS3000A_RANGE["PS3000A_5V"]: 5.0,
            ps.PS3000A_RANGE["PS3000A_10V"]: 10.0,
            ps.PS3000A_RANGE["PS3000A_20V"]: 20.0,
            ps.PS3000A_RANGE["PS3000A_50V"]: 50.0,
        }
        # Prüfen ob v_range_enum direkt ein Key ist
        if v_range_enum in table:
            return table[v_range_enum]
    
    # Fallback: String-basiertes Mapping (funktioniert auch ohne SDK)
    string_table = {
        "20MV": 0.02, "50MV": 0.05, "100MV": 0.1, "200MV": 0.2, "500MV": 0.5,
        "1V": 1.0, "2V": 2.0, "5V": 5.0, "10V": 10.0, "20V": 20.0, "50V": 50.0
    }
    
    # Wenn v_range_enum ein String ist
    if isinstance(v_range_enum, str):
        range_str = v_range_enum.replace("PS3000A_", "").replace("PS3000A", "")
        if range_str in string_table:
            return string_table[range_str]
    
    # Fallback: Standard-Wert
    return 0.05  # 50MV als Default


def pick_timebase(handle, target_fs: float, n_samples: int):
    """
    Sucht eine Timebase, deren reale Abtastrate möglichst nah an target_fs liegt.
    Das ist nötig, weil der Pico nicht jede beliebige fs direkt unterstützt.

    Parameters
    ----------
    handle : 
        handle des PicoScopes (kann None sein im Mock-Modus)
    target_fs : float
        gewünschte Abtastrate (z.B. 20e6 für 20 MS/s)
    n_samples : int
        Anzahl der Samples in einem "Messblock"

    Returns
    -------
    tuple
        (tb, dt, fs) - Timebase, Zeitdauer pro Sample [s], Abtastrate [Hz]

    Raises
    ------
    RuntimeError
        Keine gültige Zeitbasis gefunden

    Notes
    -----
    Zeitauflösung:
    - 2 Kanäle aktiv = max. Abtastrate 250MS/s = 4ns pro Sample
    - kontinuierliche Messung = > 10MS/s = 100ns pro Sample
    - Jitter der Zeitbasis: < 5 ps RMS
    - Zeitbasis-Genauigkeit: ± 50 ppm
    
    Im Mock-Modus wird eine geschätzte Timebase zurückgegeben.
    """
    # Mock-Modus: Wenn SDK nicht verfügbar, geschätzte Werte zurückgeben
    if not PICO_SDK_AVAILABLE or handle is None:
        # Geschätzte Timebase (vereinfacht)
        dt = 1.0 / target_fs
        tb = int(target_fs / 1e6)  # Grobe Schätzung
        fs = target_fs
        print(f"[Mock] Timebase geschätzt: tb={tb}, dt={dt*1e9:.2f} ns, fs={fs/1e6:.2f} MS/s")
        return tb, dt, fs
    
    # Normale Timebase-Suche
    best = None                             # beste Kandidat-Paket (Fehler, Timebase, dt, fs)
    for tb in range(1, 50000):
        time_interval_ns = ct.c_float()     # Deklaration - Zeitintervall pro Sample in ns
        max_samples = ct.c_int32()          # Deklaration - Maximal mögliche Samples bei dieser Timebase
        status = ps.ps3000aGetTimebase2(    # Versuch mit Timebase 'tb'
            handle,                         # Welches Gerät?
            tb,                             # Zu prüfende Timabase
            n_samples,                      # Anzahl der gewünschten Samples
            ct.byref(time_interval_ns),     # Zeit pro samples schreiben
            0,                              # Segment Index - nicht gebraucht
            ct.byref(max_samples),          # Wie viele samples gehen maximal?
            0                               # Oversample - hier nicht genutzt - schon vorher gesetzt
        ) # status variable oben ergibt danach => 0 = OK, alles andere = NICHT OK
        if status == 0:                         # Status PicoScope OK
            dt = time_interval_ns.value * 1e-9  # Nanosekunden Angabe Umwandlung in Sekunden
            if dt <= 0:                         # filtert fehlerhafte Werte / durch 0 usw. 
                continue
            fs = 1.0 / dt                           # Abtastfrequenz bei gegebenem dt
            err = abs(fs - target_fs) / target_fs   # relativer Fehler zur Wunschfrequenzs
            if best is None or err < best[0]:       # Speichern aktueller bester Kandidat
                best = (err, tb, dt, fs)                # Fehler, Timebase, Zeitabschnitt, Abtastfrequenz
            if err < 0.02:                          # Abbruch bei Fehler unter 2%
                break
    if best is None:    # Fehler abfangen
        raise RuntimeError("Keine gültige Timebase gefunden.")
    _, tb, dt, fs = best
    return tb, dt, fs


# ============================================================
# 3) HAUPTFUNKTION
# ============================================================
def acquire_n_pulses(n_pulses: int = N_PULSES, 
                     inter_pulse_delay_s : float =  INTER_PULSE_DELAY_S) -> None:
    """
    Erfasst n_pulses synchron auf CH A/B und hängt sie an die Run-CSV an.
    Gerät wird nur einmal geöffnet/konfiguriert.
    
    Notes
    -----
    Diese Funktion verwendet die alten Konstanten (oben definiert).
    Für neue Projekte sollte die PicoReader-Klasse verwendet werden.
    Falls PicoSDK nicht verfügbar ist, wird die Funktion mit einer Fehlermeldung beendet.
    """
    # Prüfen ob SDK verfügbar ist
    if not PICO_SDK_AVAILABLE:
        raise RuntimeError("PicoSDK nicht verfügbar. Für Mock-Messungen verwende PicoReader-Klasse.")
    
    # --------------------------------------------------------
    # 3.1 Gerät öffnen
    # --------------------------------------------------------
    handle = ct.c_int16()                               # Platzhalter - Gerät-Handle
    status = ps.ps3000aOpenUnit(ct.byref(handle), None)

    try:
        assert_pico_ok(status)
    except:
        if status in (ps.PICO_POWER_SUPPLY_NOT_CONNECTED, 
                      ps.PICO_USB3_0_DEVICE_NON_USB3_0_PORT
        ):
            # Versuche, mit externer/anderer Versorgung weiterzumachen
            status = ps.ps3000aChangePowerSource(handle, status)
            assert_pico_ok(status)
        else:
            raise

    try:
        # --------------------------------------------------------
        # 2) Kanäle konfigurieren
        # --------------------------------------------------------
        # CH A: Spannung
        assert_pico_ok(ps.ps3000aSetChannel(
            handle, 
            CH_A,           # Kanal A   
            1,              # enabled
            COUPLING_A,     # AC/DC
            RANGE_A,        # Messbereich
            DC_OFFSET_A     # DC-Offset
            )
        )
        # CH B: Rogowski / Strom
        assert_pico_ok(ps.ps3000aSetChannel(
            handle, 
            CH_B,           # Kanal B
            1,              # enabled
            COUPLING_B,     # AC/DC
            RANGE_B,        # Messbereich
            DC_OFFSET_B     # DC-Offset
            )
        )

        # --------------------------------------------------------
        # 3) Maximalwert des ADC abfragen
        # --------------------------------------------------------
        # braucht man später für: ADC-Zählwert → Volt
        max_adc = ct.c_int16() 
        assert_pico_ok(ps.ps3000aMaximumValue(handle, ct.byref(max_adc)))

        # --------------------------------------------------------
        # 4) Timebase bestimmen
        # --------------------------------------------------------
        # Welche Timebase am nächsten an TARGET_FS?
        timebase, dt, fs = pick_timebase(handle, TARGET_FS, N_SAMPLES) 
        print(f"[Info] Timebase={timebase}, dt={dt*1e9:.2f} ns, fs={fs/1e6:.2f} MS/s")

        # --------------------------------------------------------
        # 5) Trigger einrichten (hier: CH A, fallende Flanke)
        # --------------------------------------------------------
        vfs_a = range_fullscale_volts(RANGE_A) # voller Bereich in Volt (CH A)
        vfs_b = range_fullscale_volts(RANGE_B) # voller Bereich in Volt (CH B)
        
        # ADC-Schwellwert aus gewünschtem Trigger-Pegel in Volt berechnen:
        # adc = (U_trig / U_fullscale) * adc_max
        trig_adc = int((TRIG_LEVEL_V / vfs_a) * max_adc.value)
        
        
        assert_pico_ok(ps.ps3000aSetSimpleTrigger(
            handle, 
            1,                  # trigger aktiv
            CH_A,               # Trigger Channel
            trig_adc,           # ADC Value for Trigger
            ps.PS3000A_THRESHOLD_DIRECTION["PS3000A_FALLING"], # Trigger auf fallend
            0,                  # delay
            int(AUTO_TRIG_MS)   # Auto-Trigger als Fallback
        ))



        # --------------------------------------------------------
        # 6) Datenpuffer zuordnen (Block-Mode)
        # --------------------------------------------------------
        # Kommende Samples hier reinschreiben
        bufA = (ct.c_int16 * N_SAMPLES)()
        bufB = (ct.c_int16 * N_SAMPLES)()


        assert_pico_ok(ps.ps3000aSetDataBuffer(
            handle, 
            CH_A, 
            ct.byref(bufA), 
            N_SAMPLES, 
            0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]))
        
        assert_pico_ok(ps.ps3000aSetDataBuffer(
            handle, 
            CH_B, 
            ct.byref(bufB), 
            N_SAMPLES, 
            0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]
            )
        )

        # --------------------------------------------------------
        # 7) Zeitachse und Pre/Posttrigger berechnen
        # --------------------------------------------------------
        pre_samples  = int(PRETRIG_RATIO * N_SAMPLES)
        post_samples = N_SAMPLES - pre_samples
        #Zeitvektor in Sekunden (für Auswertung)
        t = np.arange(N_SAMPLES) * dt

        # --------------------------------------------------------  
        # 8) CSV-Header + Meta-Datei genau einmal schreiben
        # --------------------------------------------------------
        i_unit = "A" if (ROGOWSKI_V_PER_A and ROGOWSKI_V_PER_A > 0) else "V"
        ensure_csv(CSV_PATH, RUN_NAME, i_unit)
        write_meta(dict(
            run_name=RUN_NAME, 
            fs=fs, 
            dt_s=dt,
            pretrigger_samples=pre_samples, 
            posttrigger_samples=post_samples,
            ch_a=dict(coupling="AC", v_range=vfs_a),
            ch_b=dict(coupling="AC", 
                      v_range=vfs_b, 
                      rogowski_v_per_a=ROGOWSKI_V_PER_A
            ),
            trigger_level_v=TRIG_LEVEL_V,
        ))

        # Start-pulse_id ermitteln, damit nicht doppelt geschrieben wird
        pulse_id = scan_next_pulse_id(CSV_PATH)

        # (Optional) Wartezeit, falls deine Hardware erst noch "Puls laden" muss
        # time.sleep(10)

        # --------------------------------------------------------
        # 9) Messschleife über n_pulses
        # --------------------------------------------------------
        for k in range(n_pulses):
            # 9.1 Messungen starten
            time_indisposed_ms = ct.c_int32(0)
            assert_pico_ok(
                ps.ps3000aRunBlock(
                    handle, 
                    pre_samples, 
                    post_samples, 
                    timebase, 
                    int(OVERSAMPLE),
                    ct.byref(time_indisposed_ms), 
                    0, 
                    None, 
                    None
                )
            )

            # 9.2 Warten bis Erfassung wirklich fertig ist
            ready = ct.c_int16(0)
            while not ready.value:
                ps.ps3000aIsReady(handle, ct.byref(ready))
                time.sleep(0.001) # 1ms Polling-Intervall

            # 9.3 Werte aus dem Gerät holen
            n = ct.c_int32(N_SAMPLES)
            overflow = ct.c_int16()
            assert_pico_ok(
                ps.ps3000aGetValues(
                    handle, 
                    0, 
                    ct.byref(n), 
                    1,
                    ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
                    0, 
                    ct.byref(overflow)
                )
            )

            # 9.4 Raw -> numpy
            adcA = np.frombuffer(bufA, dtype=np.int16, count=n.value).astype(np.float64, copy=False)
            adcB = np.frombuffer(bufB, dtype=np.int16, count=n.value).astype(np.float64, copy=False)
            
            # 9.5 ADC -> echte Spannung
            #   adc_wert / max_adc → relativer Anteil
            #   * vfs_a → Volt am Pico
            #   * U_PROBE_ATTENUATION → zurückrechnen auf DUT
            u = adcA * (vfs_a / max_adc.value) * U_PROBE_ATTENUATION

            # 9.6 ADC → Strompfad (erst Volt)
            i_v = adcB * (vfs_b / max_adc.value)
            i = i_v / ROGOWSKI_V_PER_A # Volt -> Ampere mit passendem Faktor

            # 9.8 etwas Debug ausgeben
            print(
                f"[pico] pulse {k+1}/{n_pulses}: "
                f"U=[{u.min():.3f}, {u.max():.3f}] V "
                f"I=[{i.min():.3f}, {i.max():.3f}] {i_unit}"
            )

            # 9.9 in CSV schreiben (eine Zeile pro Sample)
            append_pulse_to_csv(t, u, i, i_unit, pulse_id)
            print(
                f"[pico] -> written pulse_id={pulse_id}  "
                f"samples={n.value}  overflow={overflow.value}  "
                f"timeIndisposed={time_indisposed_ms.value} ms"
            )

            pulse_id += 1  # nächster Puls
            
            # 9.10 optionale Pause zwischen Messungen
            if inter_pulse_delay_s > 0:
                time.sleep(inter_pulse_delay_s)

            # (Optional) Stop ist bei Block-Mode nicht notwendig; Treiber handled nächste Armierung.
            # Falls nötig: ps.ps3000aStop(handle)

    finally:
        # --------------------------------------------------------
        # 10) Aufräumen – Gerät immer schließen!
        # --------------------------------------------------------
        try:
            ps.ps3000aStop(handle)
        except Exception:
            pass
        ps.ps3000aCloseUnit(handle)
        print("[pico] device closed")

# --------- Start ---------
if __name__ == "__main__":
    acquire_n_pulses()


# ============================================================
# 4) PICO READER KLASSE (für GUI-Integration)
# ============================================================

class PicoReader:
    """
    Picoscope PS3000A Reader-Klasse für GUI-Integration.
    
    Diese Klasse kapselt die Picoscope-Messfunktionalität und ermöglicht
    Live-Konfiguration zur Laufzeit sowie Callbacks für Live-Updates.
    Die bestehende Funktion `acquire_n_pulses()` wird als Wrapper beibehalten.
    
    Die Klasse unterstützt:
    - Konfiguration zur Laufzeit (AC/DC, Range, Trigger, etc.)
    - Start/Stop aus GUI
    - Callbacks pro Puls für Live-Plots
    - Thread-sichere Datenübergabe an GUI
    
    Examples
    --------
    >>> reader = PicoReader()
    >>> reader.configure(run_name="test_01", target_fs=20e6, ...)
    >>> reader.set_callback(lambda pulse_id, t, u, i: print(f"Pulse {pulse_id}"))
    >>> reader.start_measurement(n_pulses=5)
    >>> # Warten bis fertig...
    >>> reader.stop()
    >>> reader.close()
    """
    
    def __init__(self):
        """
        Initialisiert den PicoReader.
        
        Das Gerät wird noch nicht geöffnet. Verwende `configure()` und
        `start_measurement()` um Messungen zu starten.
        """
        # Gerät-Handle (wird beim Öffnen gesetzt)
        self.handle = None
        
        # Konfiguration (Default-Werte aus Konstanten oben)
        self.run_name = None
        self.run_dir = None
        self.csv_path = None
        self.meta_path = None
        self.npz_path = None
        
        # Trigger-Konfiguration
        self.trigger_level_v = -0.2
        self.auto_trig_ms = 0
        
        # Abtastung
        self.target_fs = 20e6
        self.pretrig_ratio = 0.2
        self.base_samples = 400_000
        self.n_samples = None  # Wird aus base_samples + pretrig berechnet
        self.oversample = 1
        
        # Kanal A (Spannung)
        if PICO_SDK_AVAILABLE:
            self.ch_a = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
            self.coupling_a = ps.PS3000A_COUPLING["PS3000A_AC"]
            self.range_a = ps.PS3000A_RANGE["PS3000A_50MV"]
        else:
            # Mock-Werte
            self.ch_a = 0
            self.coupling_a = 0
            self.range_a = 0  # Wird später durch String-Mapping ersetzt
        
        self.dc_offset_a = 0.0
        self.u_probe_attenuation = 50.0
        
        # Kanal B (Strom/Rogowski)
        if PICO_SDK_AVAILABLE:
            self.ch_b = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_B"]
            self.coupling_b = ps.PS3000A_COUPLING["PS3000A_AC"]
            self.range_b = ps.PS3000A_RANGE["PS3000A_10V"]
        else:
            # Mock-Werte
            self.ch_b = 0
            self.coupling_b = 0
            self.range_b = 0  # Wird später durch String-Mapping ersetzt
        
        self.dc_offset_b = 0.0
        self.rogowski_v_per_a = 0.02
        
        # Messung-Status
        self.is_running = False
        self.is_configured = False
        self.pulse_id = 1
        self.pulse_count = 0
        
        # Callbacks
        self.on_pulse_callback = None  # Callback: (pulse_id, t, u, i) -> None
        
        # Datenpuffer (werden beim Konfigurieren erstellt)
        self.buf_a = None
        self.buf_b = None
        
        # Timebase und Sampling
        self.timebase = None
        self.dt = None
        self.fs = None
        self.max_adc = None
        
        # Meta-Daten für Speicherung
        self.meta = {}
        
    def configure(
        self,
        run_name: str,
        base_dir: str = None,
        target_fs: float = None,
        trigger_level_v: float = None,
        coupling_a: str = None,
        range_a: str = None,
        coupling_b: str = None,
        range_b: str = None,
        u_probe_attenuation: float = None,
        rogowski_v_per_a: float = None,
        pretrig_ratio: float = None,
        base_samples: int = None,
        oversample: int = None
    ) -> None:
        """
        Konfiguriert den PicoReader für Messungen.
        
        Diese Funktion muss vor `start_measurement()` aufgerufen werden.
        Alle Parameter sind optional und verwenden Defaults aus den Konstanten,
        falls nicht angegeben.
        
        Parameters
        ----------
        run_name : str
            Name des Messlaufs (für Speicherung: Ordner + Dateiname).
        base_dir : str, optional
            Basisverzeichnis für Speicherung (Standard: aktuelles Arbeitsverzeichnis).
        target_fs : float, optional
            Gewünschte Abtastfrequenz in Hz (Standard: 20e6 = 20 MS/s).
        trigger_level_v : float, optional
            Trigger-Pegel in Volt auf Kanal A (Standard: -0.2 V).
        coupling_a : str, optional
            Kopplung Kanal A: "AC" oder "DC" (Standard: "AC").
        range_a : str, optional
            Messbereich Kanal A: "20MV", "50MV", "100MV", "200MV", "500MV",
            "1V", "2V", "5V", "10V", "20V", "50V" (Standard: "50MV").
        coupling_b : str, optional
            Kopplung Kanal B: "AC" oder "DC" (Standard: "AC").
        range_b : str, optional
            Messbereich Kanal B: s.o. (Standard: "10V").
        u_probe_attenuation : float, optional
            Tastkopf-Dämpfung für Kanal A (z.B. 50.0 für 1:50, Standard: 50.0).
        rogowski_v_per_a : float, optional
            Rogowski-Kalibrierung: Volt pro Ampere (Standard: 0.02 V/A).
            Falls 0 oder None: Strom bleibt in Volt.
        pretrig_ratio : float, optional
            Anteil Samples vor Trigger (0.0-1.0, Standard: 0.2 = 20%).
        base_samples : int, optional
            Anzahl Samples nach Trigger (Standard: 400000).
        oversample : int, optional
            Oversampling-Faktor (1=kein, 2=mittel über 2 Samples, Standard: 1).
        
        Returns
        -------
        None
        
        Notes
        -----
        - Die Konfiguration wird nicht an das Gerät gesendet, bis `start_measurement()`
          aufgerufen wird.
        - Verzeichnis für Speicherung wird erstellt falls nötig.
        """
        # Run-Name und Verzeichnis
        self.run_name = run_name
        if base_dir is None:
            # Standard: aktuelles Arbeitsverzeichnis / Runs / run_name
            base_dir = os.path.join(os.getcwd(), "Runs")
        self.run_dir = os.path.join(base_dir, run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        
        # Dateipfade
        self.csv_path = os.path.join(self.run_dir, f"{run_name}.csv")
        self.meta_path = os.path.join(self.run_dir, f"{run_name}.meta.json")
        self.npz_path = os.path.join(self.run_dir, f"{run_name}.npz")
        
        # Trigger
        if trigger_level_v is not None:
            self.trigger_level_v = trigger_level_v
        
        # Abtastung
        if target_fs is not None:
            self.target_fs = target_fs
        if pretrig_ratio is not None:
            self.pretrig_ratio = pretrig_ratio
        if base_samples is not None:
            self.base_samples = base_samples
        if oversample is not None:
            self.oversample = oversample
        
        # Gesamtanzahl Samples berechnen
        self.n_samples = self.base_samples + int(self.pretrig_ratio * self.base_samples)
        
        # Kanal A
        if coupling_a is not None:
            if PICO_SDK_AVAILABLE:
                self.coupling_a = ps.PS3000A_COUPLING[f"PS3000A_{coupling_a}"]
            else:
                # Mock: String speichern für später
                self.coupling_a_str = coupling_a
                self.coupling_a = 0
        
        if range_a is not None:
            if PICO_SDK_AVAILABLE:
                self.range_a = ps.PS3000A_RANGE[f"PS3000A_{range_a}"]
            else:
                # Mock: String speichern
                self.range_a_str = range_a
                self.range_a = range_a  # String wird direkt verwendet
        if u_probe_attenuation is not None:
            self.u_probe_attenuation = u_probe_attenuation
        
        # Kanal B
        if coupling_b is not None:
            if PICO_SDK_AVAILABLE:
                self.coupling_b = ps.PS3000A_COUPLING[f"PS3000A_{coupling_b}"]
            else:
                # Mock: String speichern
                self.coupling_b_str = coupling_b
                self.coupling_b = 0
        
        if range_b is not None:
            if PICO_SDK_AVAILABLE:
                self.range_b = ps.PS3000A_RANGE[f"PS3000A_{range_b}"]
            else:
                # Mock: String speichern
                self.range_b_str = range_b
                self.range_b = range_b  # String wird direkt verwendet
        
        if rogowski_v_per_a is not None:
            self.rogowski_v_per_a = rogowski_v_per_a
        
        # Als konfiguriert markieren
        self.is_configured = True
        
        # Pulse-ID zurücksetzen
        self.pulse_id = 1
        self.pulse_count = 0
    
    def set_callback(self, callback):
        """
        Setzt einen Callback für jeden erfassten Puls.
        
        Der Callback wird nach jedem erfassten Puls aufgerufen, bevor
        die Daten gespeichert werden. Nützlich für Live-Updates in der GUI.
        
        Parameters
        ----------
        callback : callable, optional
            Funktion mit Signatur: (pulse_id, t, u, i) -> None
            - pulse_id: int - Eindeutige ID des Pulses
            - t: np.ndarray - Zeitvektor in Sekunden
            - u: np.ndarray - Spannungswerte in Volt
            - i: np.ndarray - Stromwerte in Ampere (oder Volt)
            Falls None: Callback wird entfernt.
        
        Examples
        --------
        >>> def on_pulse(pulse_id, t, u, i):
        ...     print(f"Pulse {pulse_id}: {len(t)} Samples")
        >>> reader.set_callback(on_pulse)
        """
        self.on_pulse_callback = callback
    
    def _open_device(self):
        """
        Öffnet das Picoscope-Gerät (interne Funktion).
        
        Raises
        ------
        RuntimeError
            Wenn das Gerät nicht geöffnet werden kann oder SDK nicht verfügbar ist.
        """
        if not PICO_SDK_AVAILABLE:
            raise RuntimeError("PicoSDK nicht verfügbar. Picoscope-Gerät kann nicht geöffnet werden.")
        
        # Gerät öffnen
        self.handle = ct.c_int16()
        status = ps.ps3000aOpenUnit(ct.byref(self.handle), None)
        
        try:
            assert_pico_ok(status)
        except:
            # Versuche alternative Stromversorgung
            if status in (ps.PICO_POWER_SUPPLY_NOT_CONNECTED, 
                          ps.PICO_USB3_0_DEVICE_NON_USB3_0_PORT):
                status = ps.ps3000aChangePowerSource(self.handle, status)
                assert_pico_ok(status)
            else:
                raise RuntimeError(f"Fehler beim Öffnen des Picoscope-Geräts: Status {status}")
    
    def _setup_channels(self):
        """
        Konfiguriert die Kanäle (interne Funktion).
        """
        if not PICO_SDK_AVAILABLE:
            # Mock-Modus: Kanal-Setup übersprungen
            print("[Mock] Kanal-Setup übersprungen (SDK nicht verfügbar)")
            return
        
        # Kanal A: Spannung
        assert_pico_ok(ps.ps3000aSetChannel(
            self.handle,
            self.ch_a,
            1,  # enabled
            self.coupling_a,
            self.range_a,
            self.dc_offset_a
        ))
        
        # Kanal B: Strom/Rogowski
        assert_pico_ok(ps.ps3000aSetChannel(
            self.handle,
            self.ch_b,
            1,  # enabled
            self.coupling_b,
            self.range_b,
            self.dc_offset_b
        ))
    
    def _setup_trigger(self):
        """
        Konfiguriert den Trigger (interne Funktion).
        """
        if not PICO_SDK_AVAILABLE:
            # Mock-Modus: Trigger wird übersprungen
            print("[Mock] Trigger-Setup übersprungen (SDK nicht verfügbar)")
            return
        
        # Vollständiger Bereich in Volt
        vfs_a = range_fullscale_volts(self.range_a)
        
        # ADC-Schwellwert berechnen
        trig_adc = int((self.trigger_level_v / vfs_a) * self.max_adc.value)
        
        # Trigger setzen
        assert_pico_ok(ps.ps3000aSetSimpleTrigger(
            self.handle,
            1,  # aktiv
            self.ch_a,  # Trigger-Kanal
            trig_adc,  # ADC-Schwellwert
            ps.PS3000A_THRESHOLD_DIRECTION["PS3000A_FALLING"],  # fallende Flanke
            0,  # delay
            int(self.auto_trig_ms)  # Auto-Trigger
        ))
    
    def _setup_data_buffers(self):
        """
        Erstellt und konfiguriert die Datenpuffer (interne Funktion).
        """
        if not PICO_SDK_AVAILABLE:
            # Mock-Modus: Puffer mit Dummy-Werten erstellen
            print("[Mock] Puffer-Setup übersprungen (SDK nicht verfügbar)")
            self.buf_a = (ct.c_int16 * self.n_samples)()
            self.buf_b = (ct.c_int16 * self.n_samples)()
            return
        
        # Puffer erstellen
        self.buf_a = (ct.c_int16 * self.n_samples)()
        self.buf_b = (ct.c_int16 * self.n_samples)()
        
        # Puffer zuordnen
        assert_pico_ok(ps.ps3000aSetDataBuffer(
            self.handle,
            self.ch_a,
            ct.byref(self.buf_a),
            self.n_samples,
            0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]
        ))
        
        assert_pico_ok(ps.ps3000aSetDataBuffer(
            self.handle,
            self.ch_b,
            ct.byref(self.buf_b),
            self.n_samples,
            0,
            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"]
        ))
    
    def start_measurement(
        self,
        n_pulses: int = 1,
        inter_pulse_delay_s: float = 0.0,
        save_csv: bool = True,
        save_npz: bool = True
    ) -> None:
        """
        Startet eine Messung mit n Pulsen.
        
        Diese Funktion öffnet das Gerät, konfiguriert es und startet die
        Messung. Die Funktion läuft blockierend, bis alle Pulse erfasst sind.
        Für nicht-blockierende Ausführung in einem Thread starten.
        
        Parameters
        ----------
        n_pulses : int, optional
            Anzahl zu erfassender Pulse (Standard: 1).
        inter_pulse_delay_s : float, optional
            Pause zwischen Pulsen in Sekunden (Standard: 0.0).
        save_csv : bool, optional
            Daten in CSV speichern (Standard: True).
        save_npz : bool, optional
            Daten in .npz speichern (Standard: True).
        
        Returns
        -------
        None
        
        Raises
        ------
        RuntimeError
            Wenn der Reader nicht konfiguriert ist oder das Gerät nicht
            geöffnet werden kann.
        
        Notes
        -----
        - Diese Funktion ist thread-sicher und kann aus einem GUI-Thread
          aufgerufen werden (blockiert GUI während Messung).
        - Für nicht-blockierende Ausführung in separatem Thread starten.
        - Wenn PicoSDK nicht verfügbar ist, läuft die Messung im Mock-Modus
          und erzeugt synthetische Testdaten.
        """
        if not self.is_configured:
            raise RuntimeError("Reader muss zuerst mit configure() konfiguriert werden")
        
        if self.is_running:
            raise RuntimeError("Messung läuft bereits")
        
        # Mock-Modus: Wenn SDK nicht verfügbar, Mock-Messung durchführen
        if not PICO_SDK_AVAILABLE:
            print("[Mock] PicoSDK nicht verfügbar - Messung im Mock-Modus")
            self._run_mock_measurement(n_pulses, inter_pulse_delay_s, save_csv, save_npz)
            return
        
        self.is_running = True
        
        try:
            # Gerät öffnen
            self._open_device()
            
            try:
                # Kanäle konfigurieren
                self._setup_channels()
                
                # Maximalwert ADC abfragen
                self.max_adc = ct.c_int16()
                assert_pico_ok(ps.ps3000aMaximumValue(self.handle, ct.byref(self.max_adc)))
                
                # Timebase bestimmen
                self.timebase, self.dt, self.fs = pick_timebase(
                    self.handle, self.target_fs, self.n_samples
                )
                
                # Trigger einrichten
                self._setup_trigger()
                
                # Datenpuffer zuordnen
                self._setup_data_buffers()
                
                # Zeitvektor berechnen
                pre_samples = int(self.pretrig_ratio * self.n_samples)
                post_samples = self.n_samples - pre_samples
                t = np.arange(self.n_samples) * self.dt
                
                # Speicherung vorbereiten
                i_unit = "A" if (self.rogowski_v_per_a and self.rogowski_v_per_a > 0) else "V"
                
                if save_csv:
                    # CSV-Header schreiben
                    ensure_csv(self.csv_path, self.run_name, i_unit)
                
                if save_npz:
                    # .npz importieren (falls nicht schon importiert)
                    from pico_pulse_lab.storage.npz_writer import append_pulse_npz, save_pulse_npz
                
                # Metadaten vorbereiten
                vfs_a = range_fullscale_volts(self.range_a)
                vfs_b = range_fullscale_volts(self.range_b)
                
                self.meta = {
                    'run_name': self.run_name,
                    'fs': self.fs,
                    'dt_s': self.dt,
                    'pretrigger_samples': pre_samples,
                    'posttrigger_samples': post_samples,
                    'ch_a': {
                        'coupling': "AC" if self.coupling_a == ps.PS3000A_COUPLING["PS3000A_AC"] else "DC",
                        'v_range': vfs_a
                    },
                    'ch_b': {
                        'coupling': "AC" if self.coupling_b == ps.PS3000A_COUPLING["PS3000A_AC"] else "DC",
                        'v_range': vfs_b,
                        'rogowski_v_per_a': self.rogowski_v_per_a
                    },
                    'trigger_level_v': self.trigger_level_v,
                    'csv_path': self.csv_path if save_csv else None,
                    'npz_path': self.npz_path if save_npz else None
                }
                
                if save_csv:
                    # Meta-JSON schreiben (write_meta benötigt meta_path, aber meta enthält bereits run_name und csv_path)
                    write_meta(self.meta_path, self.meta)
                
                if save_npz:
                    # .npz Meta initialisieren
                    save_pulse_npz(
                        self.npz_path, 0,  # pulse_id 0 für Meta
                        np.array([0.0]), np.array([0.0]), np.array([0.0]),
                        meta=self.meta
                    )
                
                # Pulse-ID ermitteln
                if save_csv:
                    self.pulse_id = scan_next_pulse_id(self.csv_path)
                elif save_npz:
                    from pico_pulse_lab.storage.npz_writer import get_all_pulse_ids
                    ids = get_all_pulse_ids(self.npz_path)
                    self.pulse_id = max(ids) + 1 if ids else 1
                else:
                    self.pulse_id = 1
                
                # Messschleife
                for k in range(n_pulses):
                    # Block-Messung starten
                    time_indisposed_ms = ct.c_int32(0)
                    assert_pico_ok(
                        ps.ps3000aRunBlock(
                            self.handle,
                            pre_samples,
                            post_samples,
                            self.timebase,
                            int(self.oversample),
                            ct.byref(time_indisposed_ms),
                            0,
                            None,
                            None
                        )
                    )
                    
                    # Warten bis fertig
                    ready = ct.c_int16(0)
                    while not ready.value:
                        ps.ps3000aIsReady(self.handle, ct.byref(ready))
                        time.sleep(0.001)
                    
                    # Werte holen
                    n = ct.c_int32(self.n_samples)
                    overflow = ct.c_int16()
                    assert_pico_ok(
                        ps.ps3000aGetValues(
                            self.handle,
                            0,
                            ct.byref(n),
                            1,
                            ps.PS3000A_RATIO_MODE["PS3000A_RATIO_MODE_NONE"],
                            0,
                            ct.byref(overflow)
                        )
                    )
                    
                    # ADC -> Volt
                    adc_a = np.frombuffer(self.buf_a, dtype=np.int16, count=n.value).astype(np.float64)
                    adc_b = np.frombuffer(self.buf_b, dtype=np.int16, count=n.value).astype(np.float64)
                    
                    # Spannung: ADC -> Volt -> DUT (mit Tastkopf-Dämpfung)
                    vfs_a = range_fullscale_volts(self.range_a)
                    vfs_b = range_fullscale_volts(self.range_b)
                    u = adc_a * (vfs_a / self.max_adc.value) * self.u_probe_attenuation
                    
                    # Strom: ADC -> Volt -> Ampere (mit Rogowski-Kalibrierung)
                    i_v = adc_b * (vfs_b / self.max_adc.value)
                    if self.rogowski_v_per_a and self.rogowski_v_per_a > 0:
                        i = i_v / self.rogowski_v_per_a
                    else:
                        i = i_v
                    
                    # Callback aufrufen (für Live-Updates)
                    if self.on_pulse_callback:
                        try:
                            self.on_pulse_callback(self.pulse_id, t, u, i)
                        except Exception as e:
                            print(f"[Warnung] Callback-Fehler: {e}")
                    
                    # Speicherung
                    if save_csv:
                        append_pulse_to_csv(self.csv_path, t, u, i, i_unit, self.pulse_id)
                    
                    if save_npz:
                        append_pulse_npz(self.npz_path, self.pulse_id, t, u, i)
                    
                    # Zähler aktualisieren
                    self.pulse_count += 1
                    self.pulse_id += 1
                    
                    # Pause zwischen Pulsen
                    if inter_pulse_delay_s > 0:
                        time.sleep(inter_pulse_delay_s)
                
            finally:
                # Gerät stoppen und schließen
                try:
                    ps.ps3000aStop(self.handle)
                except Exception:
                    pass
        
        finally:
            self.is_running = False
            self.close()
    
    def _run_mock_measurement(self, n_pulses: int, inter_pulse_delay_s: float, save_csv: bool, save_npz: bool):
        """
        Führt eine Mock-Messung durch (wenn SDK nicht verfügbar).
        
        Erzeugt synthetische Testdaten für GUI-Tests ohne echtes Picoscope-Gerät.
        """
        print(f"[Mock] Starte Mock-Messung mit {n_pulses} Pulsen")
        self.is_running = True
        
        try:
            # Zeitvektor erstellen
            pre_samples = int(self.pretrig_ratio * self.n_samples)
            post_samples = self.n_samples - pre_samples
            self.dt = 1.0 / self.target_fs  # Geschätztes dt
            self.fs = self.target_fs
            t = np.arange(self.n_samples) * self.dt
            
            # Speicherung vorbereiten
            i_unit = "A" if (self.rogowski_v_per_a and self.rogowski_v_per_a > 0) else "V"
            
            if save_csv:
                from pico_pulse_lab.storage.csv_writer import ensure_csv, append_pulse_to_csv, write_meta
                ensure_csv(self.csv_path, self.run_name, i_unit)
            
            if save_npz:
                from pico_pulse_lab.storage.npz_writer import append_pulse_npz, save_pulse_npz
            
            # Meta-Daten
            vfs_a = range_fullscale_volts(self.range_a)
            vfs_b = range_fullscale_volts(self.range_b)
            
            self.meta = {
                'run_name': self.run_name,
                'fs': self.fs,
                'dt_s': self.dt,
                'pretrigger_samples': pre_samples,
                'posttrigger_samples': post_samples,
                'ch_a': {'coupling': getattr(self, 'coupling_a_str', 'AC'), 'v_range': vfs_a},
                'ch_b': {'coupling': getattr(self, 'coupling_b_str', 'AC'), 'v_range': vfs_b,
                        'rogowski_v_per_a': self.rogowski_v_per_a},
                'trigger_level_v': self.trigger_level_v,
                'csv_path': self.csv_path if save_csv else None,
                'npz_path': self.npz_path if save_npz else None,
                'mock_mode': True  # Markierung für Mock-Modus
            }
            
            if save_csv:
                write_meta(self.meta_path, self.meta)
            
            if save_npz:
                save_pulse_npz(self.npz_path, 0, np.array([0.0]), np.array([0.0]), np.array([0.0]), meta=self.meta)
            
            # Pulse-ID ermitteln
            if save_csv:
                from pico_pulse_lab.storage.csv_writer import scan_next_pulse_id
                self.pulse_id = scan_next_pulse_id(self.csv_path)
            elif save_npz:
                from pico_pulse_lab.storage.npz_writer import get_all_pulse_ids
                ids = get_all_pulse_ids(self.npz_path)
                self.pulse_id = max(ids) + 1 if ids else 1
            else:
                self.pulse_id = 1
            
            # Mock-Messung: Synthetische Pulse
            for k in range(n_pulses):
                # Synthetische Daten erzeugen: Exponential-Fall mit Rauschen
                u = 10.0 * np.exp(-t * 1000) * np.sin(2 * np.pi * 1000 * t) + np.random.normal(0, 0.1, len(t))
                i = -0.1 * np.exp(-t * 1000) * np.cos(2 * np.pi * 1000 * t) + np.random.normal(0, 0.01, len(t))
                
                # Callback aufrufen
                if self.on_pulse_callback:
                    try:
                        self.on_pulse_callback(self.pulse_id, t, u, i)
                    except Exception as e:
                        print(f"[Warnung] Callback-Fehler: {e}")
                
                # Speicherung
                if save_csv:
                    append_pulse_to_csv(self.csv_path, t, u, i, i_unit, self.pulse_id)
                
                if save_npz:
                    append_pulse_npz(self.npz_path, self.pulse_id, t, u, i)
                
                self.pulse_count += 1
                self.pulse_id += 1
                
                print(f"[Mock] Puls {k+1}/{n_pulses} erfasst")
                
                if inter_pulse_delay_s > 0:
                    time.sleep(inter_pulse_delay_s)
            
            print("[Mock] Mock-Messung abgeschlossen")
        
        finally:
            self.is_running = False
    
    def stop(self) -> None:
        """
        Stoppt eine laufende Messung.
        
        Diese Funktion sollte aus einem anderen Thread aufgerufen werden,
        wenn `start_measurement()` in einem Thread läuft.
        
        Returns
        -------
        None
        
        Notes
        -----
        - Aktuell unterstützt die Implementierung keinen Abbruch während
          der Messung. Diese Funktion bereitet die Implementierung vor.
        """
        self.is_running = False
    
    def close(self) -> None:
        """
        Schließt das Picoscope-Gerät.
        
        Diese Funktion wird automatisch von `start_measurement()` aufgerufen.
        Kann auch manuell aufgerufen werden für explizites Cleanup.
        
        Returns
        -------
        None
        """
        if self.handle is not None and PICO_SDK_AVAILABLE:
            try:
                ps.ps3000aCloseUnit(self.handle)
            except Exception:
                pass
            self.handle = None
        elif not PICO_SDK_AVAILABLE:
            # Mock-Modus: Nichts zu schließen
            self.handle = None
    
    def get_latest_pulse(self) -> tuple:
        """
        Gibt die Daten des letzten erfassten Pulses zurück.
        
        Returns
        -------
        tuple
            (pulse_id, t, u, i) oder None falls noch kein Puls erfasst wurde.
        
        Notes
        -----
        - Diese Funktion liest die Daten aus der letzten Messung.
        - Für Live-Zugriff während Messung verwende Callbacks.
        """
        # Diese Funktion würde aus Speicher laden
        # Aktuell wird sie durch Callbacks ersetzt
        # Kann erweitert werden für Speicher-basierte Implementierung
        return None
    
    def get_status(self) -> dict:
        """
        Gibt den aktuellen Status des Readers zurück.
        
        Returns
        -------
        dict
            Dictionary mit Status-Informationen:
            - is_running: bool - Läuft eine Messung?
            - is_configured: bool - Ist der Reader konfiguriert?
            - pulse_count: int - Anzahl erfasster Pulse in aktueller Session
            - pulse_id: int - Nächste freie Pulse-ID
            - run_name: str - Name des aktuellen Messlaufs
        """
        return {
            'is_running': self.is_running,
            'is_configured': self.is_configured,
            'pulse_count': self.pulse_count,
            'pulse_id': self.pulse_id,
            'run_name': self.run_name
        }
