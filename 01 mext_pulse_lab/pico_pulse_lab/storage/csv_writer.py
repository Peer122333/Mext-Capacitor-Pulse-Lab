"""
CSV-Schreibfunktionen für Puls-Messdaten.

Dieses Modul stellt Funktionen zum Speichern von Puls-Messdaten im
CSV-Format bereit. Das Format ist menschenlesbar und universell
kompatibel, aber langsamer als binäre Formate wie .npz.

Struktur der CSV:
- Header-Zeilen beginnen mit '#'
- Datenzeilen: pulse_id,sample_idx,time_s,u_V,i_{A|V}
"""

import os
import json
import numpy as np
from datetime import datetime
from typing import Dict


# ---------- CSV-Helfer ----------
def _csv_header(run_name: str, i_unit: str) -> str:
    """
    Erzeugt den Header für eine neue CSV-Datei.
    
    Der Header enthält Metadaten wie Run-Name, Erstellungsdatum und
    Spaltenbeschreibung. Wird nur beim ersten Anlegen der Datei geschrieben.
    
    Parameters
    ----------
    run_name : str
        Name des Messlaufs (z.B. "31-10_01").
    i_unit : str
        Einheit des Stroms: "A" für Ampere oder "V" für Volt
        (falls Rogowski noch nicht umgerechnet wurde).
    
    Returns
    -------
    str
        Header-String mit Kommentarzeilen.
    
    Examples
    --------
    >>> header = _csv_header("test_01", "A")
    >>> print(header)
    # RUN_NAME=test_01
    # created=2024-01-01T12:00:00
    # columns: pulse_id,sample_idx,time_s,u_V,i_A
    """
    return (
        f"# RUN_NAME={run_name}\n"
        f"# created={datetime.now().isoformat()}\n"
        f"# columns: pulse_id,sample_idx,time_s,u_V,i_{i_unit}\n"
    )


def ensure_csv(csv_path: str, run_name: str, i_unit: str) -> None:
    """
    Legt CSV-Datei mit Header an, falls sie noch nicht existiert.
    
    Diese Funktion sollte einmal vor dem ersten Schreiben aufgerufen werden,
    um sicherzustellen, dass die Datei existiert und den korrekten Header hat.
    
    Parameters
    ----------
    csv_path : str
        Vollständiger Pfad zur CSV-Datei (z.B. "runs/test_01/test_01.csv").
        Verzeichnis muss nicht existieren (wird erstellt).
    run_name : str
        Name des Messlaufs.
    i_unit : str
        Einheit des Stroms ("A" oder "V").
    
    Returns
    -------
    None
    
    Examples
    --------
    >>> ensure_csv("runs/test_01.csv", "test_01", "A")
    >>> # Datei ist jetzt bereit zum Anhängen von Daten
    """
    # Verzeichnis erstellen falls nötig
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    
    # Header schreiben wenn Datei nicht existiert
    if not os.path.exists(csv_path):
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write(_csv_header(run_name, i_unit))


def scan_next_pulse_id(csv_path: str) -> int:
    """
    Liest aus bestehender CSV die nächste freie pulse_id.
    
    Durchsucht die CSV nach der höchsten vorhandenen pulse_id und gibt
    die nächste zurück. Sollte nur einmal pro Mess-Session am Anfang
    aufgerufen werden. Danach wird pulse_id manuell hochgezählt.
    
    Parameters
    ----------
    csv_path : str
        Pfad zur CSV-Datei.
    
    Returns
    -------
    int
        Nächste freie pulse_id (1 wenn Datei leer/nicht existiert).
    
    Examples
    --------
    >>> next_id = scan_next_pulse_id("runs/test_01.csv")
    >>> print(f"Nächste ID: {next_id}")
    """
    if not os.path.exists(csv_path):
        return 1  # Erste ID wenn Datei nicht existiert
    
    last_id = 0
    
    # Datei durchsuchen nach höchster pulse_id
    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            # Kommentarzeilen überspringen
            if not line or line[0] == "#":
                continue
            
            try:
                # Erste Spalte ist pulse_id
                pid = int(line.split(",")[0])
                if pid > last_id:
                    last_id = pid
            except Exception:
                # Fehlerhafte Zeilen ignorieren
                pass

    return last_id + 1


def append_pulse_to_csv(
    csv_path: str,
    t: np.ndarray,
    u: np.ndarray,
    i: np.ndarray,
    i_unit: str,
    pulse_id: int
) -> None:
    """
    Hängt einen Puls mit gegebener pulse_id an die CSV an.
    
    Diese Funktion ist der Alias für `append_csv_with_id()` und wird
    von `picoscope_reader.py` verwendet. Sie ist thread-sicher wenn
    auf Datei-Ebene (gleiche Datei von mehreren Threads).
    
    Parameters
    ----------
    csv_path : str
        Pfad zur CSV-Datei. MUSS bereits existieren (mit Header).
        Verwende `ensure_csv()` vorher falls nötig.
    t : np.ndarray
        Zeitvektor in Sekunden (1D-Array).
    u : np.ndarray
        Spannungswerte in Volt (1D-Array, gleiche Länge wie t).
    i : np.ndarray
        Stromwerte in Ampere oder Volt (1D-Array, gleiche Länge wie t).
    i_unit : str
        Einheit des Stroms ("A" oder "V").
        Muss mit dem Header übereinstimmen.
    pulse_id : int
        Eindeutige ID des Pulses (positive Ganzzahl).
    
    Returns
    -------
    None
    
    Raises
    ------
    FileNotFoundError
        Wenn die CSV-Datei nicht existiert.
    ValueError
        Wenn die Arrays unterschiedliche Längen haben.
    
    Examples
    --------
    >>> import numpy as np
    >>> t = np.linspace(0, 1e-3, 1000)
    >>> u = np.sin(2 * np.pi * 1000 * t) * 10
    >>> i = np.cos(2 * np.pi * 1000 * t) * 0.1
    >>> append_pulse_to_csv("runs/test_01.csv", t, u, i, "A", pulse_id=1)
    """
    # Delegiere an append_csv_with_id (synonym)
    append_csv_with_id(csv_path, t, u, i, i_unit, pulse_id)


def append_csv_with_id(
    csv_path: str,
    t: np.ndarray, 
    u: np.ndarray, 
    i: np.ndarray, 
    i_unit: str, 
    pulse_id: int
) -> None:
    """
    Hängt einen Puls mit gegebener pulse_id an die CSV an (ohne erneuten Scan).
    
    Diese Funktion ist die Hauptimplementierung für das Schreiben von
    Puls-Daten in CSV-Format. Die Daten werden als Zeilen im Format
    "pulse_id,sample_idx,time_s,u_V,i_{A|V}" geschrieben.
    
    Parameters
    ----------
    csv_path : str
        Pfad zur CSV-Datei. MUSS bereits existieren.
        Verwende `ensure_csv()` vorher falls nötig.
    t : np.ndarray
        Zeitvektor in Sekunden.
    u : np.ndarray
        Spannungswerte in Volt.
    i : np.ndarray
        Stromwerte (in Einheit i_unit).
    i_unit : str
        Einheit des Stroms ("A" oder "V").
    pulse_id : int
        Eindeutige ID des Pulses.
    
    Returns
    -------
    None
    
    Notes
    -----
    - Datei wird im Append-Modus geöffnet (an vorhandene Daten anhängen).
    - Format: wissenschaftliche Notation mit 9 Dezimalstellen für Zeit/Spannung/Strom.
    - Integer-Format für pulse_id und sample_idx.
    """
    # Länge prüfen
    n = len(t)
    if len(u) != n or len(i) != n:
        raise ValueError("Arrays t, u, i müssen gleiche Länge haben")
    
    # Datenmatrix zusammenbauen: [pulse_id, sample_idx, t, u, i]
    data = np.column_stack([
        np.full(n, pulse_id, dtype=np.int64),  # pulse_id (konstant)
        np.arange(n, dtype=np.int64),          # sample_idx (0, 1, 2, ...)
        t.astype(np.float64, copy=False),      # Zeit in Sekunden
        u.astype(np.float64, copy=False),      # Spannung in Volt
        i.astype(np.float64, copy=False),      # Strom (Einheit: i_unit)
    ])
    
    # In Datei schreiben (append mode)
    with open(csv_path, "a", encoding="utf-8") as f:
        np.savetxt(
            f, 
            data, 
            delimiter=",",  # Komma-getrennt
            fmt=["%d", "%d", "%.9e", "%.9e", "%.9e"]  # Format: int, int, float, float, float
        )


def write_meta(meta_path: str, meta: Dict) -> None:
    """
    Schreibt/aktualisiert eine Meta-JSON zum Run.
    
    Diese Funktion wird von `picoscope_reader.py` verwendet und ist
    ein Wrapper um `write_meta_once()`. Sie fügt automatisch Timestamp hinzu.
    
    Parameters
    ----------
    meta_path : str
        Pfad zur Meta-JSON-Datei (z.B. "runs/test_01/test_01.meta.json").
    meta : dict
        Dictionary mit Metadaten (z.B. fs, dt_s, trigger_level_v, etc.).
        Die Werte 'run_name' und 'csv_path' werden automatisch ergänzt.
    
    Returns
    -------
    None
    
    Examples
    --------
    >>> meta = {
    ...     'fs': 20e6,
    ...     'dt_s': 5e-8,
    ...     'trigger_level_v': -0.2
    ... }
    >>> write_meta("runs/test_01.meta.json", meta)
    """
    # Extrahiere run_name und csv_path aus meta falls vorhanden
    run_name = meta.get('run_name', 'unknown')
    csv_path = meta.get('csv_path', meta_path.replace('.meta.json', '.csv'))
    
    # Delegiere an write_meta_once
    write_meta_once(meta_path, run_name, csv_path, meta)


def write_meta_once(meta_path: str, run_name: str, csv_path: str, meta: Dict) -> None:
    """
    Schreibt/aktualisiert eine Meta-JSON zum Run (Sampling, Bereiche etc.).
    
    Diese Funktion speichert alle Messparameter und Konfiguration in einer
    JSON-Datei. Nützlich für spätere Analyse und Reproduzierbarkeit.
    
    Parameters
    ----------
    meta_path : str
        Pfad zur Meta-JSON-Datei.
    run_name : str
        Name des Messlaufs.
    csv_path : str
        Pfad zur zugehörigen CSV-Datei.
    meta : dict
        Dictionary mit Metadaten. Typische Schlüssel:
        - fs: Abtastfrequenz in Hz
        - dt_s: Zeitabstand zwischen Samples in Sekunden
        - trigger_level_v: Trigger-Pegel in Volt
        - ch_a, ch_b: Kanal-Konfigurationen
        - pretrigger_samples, posttrigger_samples: Trigger-Position
    
    Returns
    -------
    None
    
    Notes
    -----
    - Datei wird überschrieben (nicht angehängt).
    - Timestamp wird automatisch hinzugefügt.
    - JSON wird mit 2 Zeilen Einrückung formatiert (lesbar).
    
    Examples
    --------
    >>> meta = {
    ...     'fs': 20e6,
    ...     'dt_s': 5e-8,
    ...     'ch_a': {'coupling': 'AC', 'v_range': 0.05}
    ... }
    >>> write_meta_once("runs/test_01.meta.json", "test_01", "runs/test_01.csv", meta)
    """
    # Verzeichnis erstellen falls nötig
    os.makedirs(os.path.dirname(meta_path), exist_ok=True)
    
    # Metadaten kopieren und ergänzen
    meta_out = dict(meta)
    meta_out["run_name"] = run_name
    meta_out["csv_path"] = csv_path
    meta_out["created_or_updated"] = datetime.now().isoformat()
    
    # JSON schreiben (überschreibt alte Datei)
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2)  # Schön formatiert

