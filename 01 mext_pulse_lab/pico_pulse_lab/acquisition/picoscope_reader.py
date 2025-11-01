"""
PS3000A Acquisition – U/I synchron, Append-CSV pro Messlauf (Mehrfach-Pulse)
- Kanal A: Spannung (AC, kleiner Bereich)
- Kanal B: Rogowski (AC, optional Volt->Ampere)
- Jede Erfassung wird als neuer 'pulse_id' in EINE CSV angehängt
  Spalten: pulse_id, sample_idx, time_s, u_V, i_{V|A}
"""

import os
import time
import json
import ctypes as ct # C-Typen für Picoscope SDK
import numpy as np  
from datetime import datetime
from picosdk.ps3000a import ps3000a as ps    # Picoscope PS3000A SDK 
from picosdk.functions import assert_pico_ok # Fehlerprüfung SDK-Aufrufe

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
CH_A                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_A"]
COUPLING_A          = ps.PS3000A_COUPLING["PS3000A_AC"]
RANGE_A             = ps.PS3000A_RANGE["PS3000A_50MV"]   # ±2 V -> eigentlich sollte beim Spannungsmessung hier bei 1:100 kleinere Werte besser klappen
DC_OFFSET_A         = 0.0  # Gleichanteil-Offset
U_PROBE_ATTENUATION = 50.0 # 1:50 Tastkopf, heißt bei 10V => 0,2V an PicoScope
# u_real = u_measured * u_probe_attenuation

# Kanal B: Rogowski (Strom)
CH_B                = ps.PS3000A_CHANNEL["PS3000A_CHANNEL_B"]
COUPLING_B          = ps.PS3000A_COUPLING["PS3000A_AC"]
RANGE_B             = ps.PS3000A_RANGE["PS3000A_10V"]   # anpassen
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
    """Mapping Pico-Range-Enum -> realer Messbereich in Volt.
    
    Quantisierungsauflösung
    -------------------------
    - ADC des 3205A hat 8 Bit Auflösung = 256 Stufen
    - zB delta_U = 100mV/256 = 0.39mV
    - zB delta_U = 40V/256 = 156mV

    Hinweise
    -------------------------
    - Effektive Auflösung (ENOB): 7.6 Bit
    - (durch Rauschen und Nichtlinearitäten etwas geringer als 8 Bit)
    """
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
    return table[v_range_enum]


def pick_timebase(handle, target_fs: float, n_samples: int):
    """Sucht eine Timebase, deren reale Abtastrate möglichst nah an target_fs liegt.
    Das ist nötig, weil der Pico nicht jede beliebige fs direkt unterstützt.

    Parameters
    ----------
    handle : 
        handle des PicoScopes
    target_fs : float
        gewünschte Abtastrate
        zB 20e6 für 20 MS/S 
        maximal möglich bei 
    n_samples : int
        Anzahl der Samples in einem "Messblock"

    Returns
    -------
        tb: Timebase
        dt: Zeitdauer [ns]
        fs: Abtastrate [Hz]

    Raises
    ------
    RuntimeError
        Keine gültige Zeitbasis gefunden

    Zeitauflösung
    -------------
    - 2 Kanäle aktv = max. Abtastrate 250MS/s = 4ns pro Sample
    - kontinuierliche Messung = > 10MS/s = 100ns pro Sample
    - Wiederholte Abtastung (ETS-Mode) max. Abtastrate 5GS/s = 0.2 ns (nur für repetitive Signale)
    
    Hinweise
    -------------------------
    - Jitter der Zeitbasis: < 5 ps RMS
    - Zeitbasis-Genauigkeit: ± 50 ppm
    """
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

# ---------- CSV-Helfer ----------
def _csv_header(i_unit: str) -> str:
    """Schreibt den CSV Header bei einer neuen Messreihe EINMALIG in die ersten Zeilen.
    """
    return (
        f"# RUN_NAME={RUN_NAME}\n"
        f"# created={datetime.now().isoformat()}\n"
        f"# columns: pulse_id,sample_idx,time_s,u_V,i_{i_unit}\n"
    )


def _ensure_csv(i_unit: str) -> None:
    """legt CSV an, falls sie noch nicht existiert."""
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", encoding="utf-8") as f:
            f.write(_csv_header(i_unit))

def _csv_exists_write_header(i_unit):
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", encoding="utf-8") as f:
            f.write(_csv_header(i_unit))

def _next_pulse_id_scan() -> int:
    """Liest die höchste pulse_id aus der CSV (nur einmal zu Beginn nötig)."""
    if not os.path.exists(CSV_PATH):
        return 1
    last_id = 0
    with open(CSV_PATH, "r", encoding="utf-8") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            try:
                pid = int(line.split(",")[0])
                if pid > last_id:
                    last_id = pid
            except Exception:
                pass
    return last_id + 1

def append_csv_with_id(t: np.ndarray, 
                       u: np.ndarray, 
                       i: np.ndarray, 
                       i_unit: str, 
                       pulse_id: int) -> None:
    """Hängt einen Puls mit gegebener pulse_id an die CSV an (ohne erneuten Scan)."""
    _csv_exists_write_header(i_unit)
    n = len(t)
    data = np.column_stack([
        np.full(n, pulse_id, dtype=np.int64),
        np.arange(n, dtype=np.int64),
        t.astype(np.float64, copy=False),
        u.astype(np.float64, copy=False),
        i.astype(np.float64, copy=False),
    ])
    with open(CSV_PATH, "a", encoding="utf-8") as f:
        np.savetxt(f, data, delimiter=",",
                   fmt=["%d", "%d", "%.9e", "%.9e", "%.9e"])

def write_meta_once(meta):
    """Schreibt/aktualisiert eine Meta-JSON zum Run (Sampling, Bereiche etc.)."""
    meta_out = dict(meta)
    meta_out["run_name"] = RUN_NAME
    meta_out["csv_path"] = CSV_PATH
    meta_out["created_or_updated"] = datetime.now().isoformat()
    with open(META_PATH, "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2)


# ============================================================
# 3) HAUPTFUNKTION
# ============================================================
def acquire_n_pulses(n_pulses: int = N_PULSES, 
                     inter_pulse_delay_s : float =  INTER_PULSE_DELAY_S) -> None:
    """
    Erfasst n_pulses synchron auf CH A/B und hängt sie an die Run-CSV an.
    Gerät wird nur einmal geöffnet/konfiguriert.
    """
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
        _csv_exists_write_header(i_unit)
        write_meta_once(dict(
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
        pulse_id = _next_pulse_id_scan()

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
            append_csv_with_id(t, u, i, i_unit, pulse_id)
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
