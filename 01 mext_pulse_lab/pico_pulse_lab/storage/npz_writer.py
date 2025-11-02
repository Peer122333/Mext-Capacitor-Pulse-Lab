"""
NumPy .npz Speicherung für Puls-Messdaten.

Dieses Modul stellt Funktionen zum schnellen, kompakten Speichern und Laden
von Puls-Messdaten im NumPy .npz Format bereit. Das Format ist deutlich
schneller als CSV und behält die volle numerische Präzision.

Struktur:
- {'pulses': {pulse_id: {'t': ..., 'u': ..., 'i': ...}}, 'meta': {...}}
"""

import os
import json
import numpy as np
from typing import Dict, Optional, Tuple
from datetime import datetime


def save_pulse_npz(
    path: str,
    pulse_id: int,
    t: np.ndarray,
    u: np.ndarray,
    i: np.ndarray,
    meta: Optional[Dict] = None
) -> None:
    """
    Speichert einen einzelnen Puls in eine neue .npz Datei.
    
    Erstellt eine neue Datei oder überschreibt eine existierende.
    Für das Anhängen weiterer Pulse verwende `append_pulse_npz()`.
    
    Parameters
    ----------
    path : str
        Vollständiger Pfad zur .npz Datei (z.B. "runs/test/run_01.npz").
        Verzeichnis muss nicht existieren (wird erstellt).
    pulse_id : int
        Eindeutige ID des Pulses (positive Ganzzahl).
        Wird als Schlüssel im 'pulses' Dictionary verwendet.
    t : np.ndarray
        Zeitvektor in Sekunden (1D-Array).
    u : np.ndarray
        Spannungswerte in Volt (1D-Array, gleiche Länge wie t).
    i : np.ndarray
        Stromwerte in Ampere (1D-Array, gleiche Länge wie t).
    meta : dict, optional
        Dictionary mit Metadaten (z.B. Abtastfrequenz, Bereiche, etc.).
        Wird unter dem Schlüssel 'meta' gespeichert.
        Standard: {'created': timestamp, 'pulse_count': 1}
    
    Returns
    -------
    None
    
    Examples
    --------
    >>> import numpy as np
    >>> t = np.linspace(0, 1e-3, 1000)
    >>> u = np.sin(2 * np.pi * 1000 * t) * 10
    >>> i = np.cos(2 * np.pi * 1000 * t) * 0.1
    >>> meta = {'fs': 1e6, 'run_name': 'test_01'}
    >>> save_pulse_npz('runs/test_01.npz', pulse_id=1, t=t, u=u, i=i, meta=meta)
    """
    # Verzeichnis erstellen falls nötig
    os.makedirs(os.path.dirname(path), exist_ok=True)
    
    # Datenstruktur aufbauen
    data = {
        'pulses': {
            pulse_id: {
                't': np.asarray(t, dtype=np.float64),
                'u': np.asarray(u, dtype=np.float64),
                'i': np.asarray(i, dtype=np.float64)
            }
        }
    }
    
    # Metadaten hinzufügen
    if meta is None:
        meta = {}
    
    # Standard-Metadaten ergänzen
    meta_out = dict(meta)
    meta_out['created'] = datetime.now().isoformat()
    meta_out['pulse_count'] = 1
    data['meta'] = meta_out
    
    # Speichern
    np.savez_compressed(path, **data)


def load_pulse_npz(path: str, pulse_id: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Lädt einen einzelnen Puls aus einer .npz Datei.
    
    Parameters
    ----------
    path : str
        Pfad zur .npz Datei.
    pulse_id : int
        ID des zu ladenden Pulses.
    
    Returns
    -------
    t : np.ndarray
        Zeitvektor in Sekunden.
    u : np.ndarray
        Spannungswerte in Volt.
    i : np.ndarray
        Stromwerte in Ampere.
    
    Raises
    ------
    FileNotFoundError
        Wenn die Datei nicht existiert.
    KeyError
        Wenn die pulse_id nicht in der Datei vorhanden ist.
    
    Examples
    --------
    >>> t, u, i = load_pulse_npz('runs/test_01.npz', pulse_id=1)
    >>> print(f"Geladen: {len(t)} Samples")
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Datei nicht gefunden: {path}")
    
    # Laden
    loaded = np.load(path, allow_pickle=True)
    
    # Struktur prüfen
    if 'pulses' not in loaded:
        raise ValueError("Ungültige .npz Struktur: 'pulses' fehlt")
    
    pulses = loaded['pulses'].item() if isinstance(loaded['pulses'], np.ndarray) else loaded['pulses']
    
    if pulse_id not in pulses:
        raise KeyError(f"Pulse-ID {pulse_id} nicht in Datei gefunden")
    
    pulse_data = pulses[pulse_id]
    t = pulse_data['t']
    u = pulse_data['u']
    i = pulse_data['i']
    
    return t, u, i


def append_pulse_npz(
    path: str,
    pulse_id: int,
    t: np.ndarray,
    u: np.ndarray,
    i: np.ndarray
) -> None:
    """
    Hängt einen neuen Puls an eine bestehende .npz Datei an.
    
    Lädt die bestehende Datei, fügt den neuen Puls hinzu und speichert
    alles wieder. Die Metadaten werden aktualisiert (pulse_count, updated).
    
    Parameters
    ----------
    path : str
        Pfad zur .npz Datei. Muss bereits existieren.
        Für erste Pulse verwende `save_pulse_npz()`.
    pulse_id : int
        Eindeutige ID des neuen Pulses.
        Falls bereits vorhanden, wird der alte Puls überschrieben.
    t : np.ndarray
        Zeitvektor in Sekunden.
    u : np.ndarray
        Spannungswerte in Volt.
    i : np.ndarray
        Stromwerte in Ampere.
    
    Returns
    -------
    None
    
    Raises
    ------
    FileNotFoundError
        Wenn die Datei nicht existiert.
    
    Examples
    --------
    >>> # Ersten Puls speichern
    >>> save_pulse_npz('runs/test_01.npz', 1, t1, u1, i1, meta={'fs': 1e6})
    >>> # Weitere Pulse anhängen
    >>> append_pulse_npz('runs/test_01.npz', 2, t2, u2, i2)
    >>> append_pulse_npz('runs/test_01.npz', 3, t3, u3, i3)
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Datei nicht gefunden: {path}. Verwende save_pulse_npz() für erste Pulse.")
    
    # Bestehende Datei laden
    loaded = np.load(path, allow_pickle=True)
    
    # Struktur extrahieren
    if 'pulses' in loaded:
        # NumPy-Array zu dict konvertieren falls nötig
        pulses = loaded['pulses'].item() if isinstance(loaded['pulses'], np.ndarray) else loaded['pulses']
    else:
        pulses = {}
    
    if 'meta' in loaded:
        meta = loaded['meta'].item() if isinstance(loaded['meta'], np.ndarray) else loaded['meta']
    else:
        meta = {}
    
    # Neuen Puls hinzufügen
    pulses[pulse_id] = {
        't': np.asarray(t, dtype=np.float64),
        'u': np.asarray(u, dtype=np.float64),
        'i': np.asarray(i, dtype=np.float64)
    }
    
    # Metadaten aktualisieren
    meta['updated'] = datetime.now().isoformat()
    meta['pulse_count'] = len(pulses)
    
    # Neue Struktur aufbauen
    data = {
        'pulses': pulses,
        'meta': meta
    }
    
    # Speichern (überschreibt alte Datei)
    np.savez_compressed(path, **data)


def get_all_pulse_ids(path: str) -> list:
    """
    Gibt eine Liste aller gespeicherten Pulse-IDs aus einer .npz Datei zurück.
    
    Parameters
    ----------
    path : str
        Pfad zur .npz Datei.
    
    Returns
    -------
    list of int
        Sortierte Liste aller Pulse-IDs in der Datei.
    
    Examples
    --------
    >>> ids = get_all_pulse_ids('runs/test_01.npz')
    >>> print(f"Gefunden: {len(ids)} Pulse")
    >>> print(f"IDs: {ids}")
    """
    if not os.path.exists(path):
        return []
    
    loaded = np.load(path, allow_pickle=True)
    
    if 'pulses' not in loaded:
        return []
    
    pulses = loaded['pulses'].item() if isinstance(loaded['pulses'], np.ndarray) else loaded['pulses']
    ids = sorted(list(pulses.keys()))
    
    return ids


def load_meta_npz(path: str) -> Dict:
    """
    Lädt nur die Metadaten aus einer .npz Datei.
    
    Parameters
    ----------
    path : str
        Pfad zur .npz Datei.
    
    Returns
    -------
    dict
        Dictionary mit Metadaten.
    
    Examples
    --------
    >>> meta = load_meta_npz('runs/test_01.npz')
    >>> print(f"Abtastfrequenz: {meta.get('fs', 'N/A')} Hz")
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Datei nicht gefunden: {path}")
    
    loaded = np.load(path, allow_pickle=True)
    
    if 'meta' in loaded:
        meta = loaded['meta'].item() if isinstance(loaded['meta'], np.ndarray) else loaded['meta']
        return dict(meta)
    else:
        return {}

