"""
Test-Funktionen für .npz Speicherung.

Diese Tests überprüfen das Speichern, Laden und Anhängen von
Puls-Daten im .npz Format.
"""

import numpy as np
import os
import tempfile
import sys

# Pfad für Import hinzufügen
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from pico_pulse_lab.storage.npz_writer import (
    save_pulse_npz,
    load_pulse_npz,
    append_pulse_npz,
    get_all_pulse_ids,
    load_meta_npz
)


def test_save_load_pulse_npz():
    """
    Test: Speichern und Laden eines einzelnen Pulses.
    
    Returns
    -------
    bool
        True wenn Test erfolgreich, False sonst.
    """
    print("\n=== Test: save/load_pulse_npz ===")
    
    # Temporäres Verzeichnis
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = os.path.join(tmpdir, "test_pulse.npz")
        
        # Synthetische Daten erstellen
        t = np.linspace(0, 1e-3, 1000)
        u = np.sin(2 * np.pi * 1000 * t) * 10  # 10 V Sinus, 1 kHz
        i = np.cos(2 * np.pi * 1000 * t) * 0.1  # 0.1 A Cosinus
        
        # Meta-Daten
        meta = {
            'fs': 1e6,
            'dt_s': 1e-6,
            'run_name': 'test_01'
        }
        
        try:
            # Speichern
            save_pulse_npz(npz_path, pulse_id=1, t=t, u=u, i=i, meta=meta)
            print(f"✓ Speichern erfolgreich: {npz_path}")
            
            # Laden
            t_loaded, u_loaded, i_loaded = load_pulse_npz(npz_path, pulse_id=1)
            print(f"✓ Laden erfolgreich: {len(t_loaded)} Samples")
            
            # Prüfen ob Daten identisch
            assert len(t_loaded) == len(t), "Zeitvektor-Länge stimmt nicht"
            assert len(u_loaded) == len(u), "Spannungsvektor-Länge stimmt nicht"
            assert len(i_loaded) == len(i), "Stromvektor-Länge stimmt nicht"
            
            assert np.allclose(t_loaded, t, rtol=1e-9), "Zeitvektor stimmt nicht überein"
            assert np.allclose(u_loaded, u, rtol=1e-9), "Spannungsvektor stimmt nicht überein"
            assert np.allclose(i_loaded, i, rtol=1e-9), "Stromvektor stimmt nicht überein"
            
            # Meta-Daten prüfen
            meta_loaded = load_meta_npz(npz_path)
            assert meta_loaded.get('run_name') == meta['run_name'], "Meta-Daten stimmen nicht"
            
            print("✓ Test erfolgreich (Daten identisch)")
            return True
        
        except Exception as e:
            print(f"✗ Test fehlgeschlagen: {e}")
            import traceback
            traceback.print_exc()
            return False


def test_append_pulse_npz():
    """
    Test: Anhängen mehrerer Pulse an bestehende Datei.
    
    Returns
    -------
    bool
        True wenn Test erfolgreich, False sonst.
    """
    print("\n=== Test: append_pulse_npz ===")
    
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = os.path.join(tmpdir, "test_pulses.npz")
        
        # Ersten Puls speichern
        t1 = np.linspace(0, 1e-3, 1000)
        u1 = np.sin(2 * np.pi * 1000 * t1) * 10
        i1 = np.cos(2 * np.pi * 1000 * t1) * 0.1
        
        meta = {'run_name': 'test_multi', 'fs': 1e6}
        save_pulse_npz(npz_path, pulse_id=1, t=t1, u=u1, i=i1, meta=meta)
        
        # Weitere Pulse anhängen
        for pulse_id in [2, 3, 4]:
            t = np.linspace(0, 1e-3, 1000)
            u = np.sin(2 * np.pi * 1000 * t + pulse_id) * 10
            i = np.cos(2 * np.pi * 1000 * t + pulse_id) * 0.1
            
            append_pulse_npz(npz_path, pulse_id, t, u, i)
        
        try:
            # Alle Pulse-IDs prüfen
            ids = get_all_pulse_ids(npz_path)
            print(f"✓ Gefundene Pulse-IDs: {ids}")
            
            assert len(ids) == 4, f"Erwartet 4 Pulse, gefunden {len(ids)}"
            assert ids == [1, 2, 3, 4], f"Pulse-IDs stimmen nicht: {ids}"
            
            # Jeden Puls laden und prüfen
            for pulse_id in ids:
                t, u, i = load_pulse_npz(npz_path, pulse_id)
                assert len(t) == 1000, f"Puls {pulse_id}: Falsche Länge"
                print(f"✓ Puls {pulse_id} geladen: {len(t)} Samples")
            
            print("✓ Test erfolgreich (alle Pulse korrekt)")
            return True
        
        except Exception as e:
            print(f"✗ Test fehlgeschlagen: {e}")
            import traceback
            traceback.print_exc()
            return False


def test_npz_with_meta():
    """
    Test: Meta-Daten in .npz prüfen.
    
    Returns
    -------
    bool
        True wenn Test erfolgreich, False sonst.
    """
    print("\n=== Test: npz_with_meta ===")
    
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = os.path.join(tmpdir, "test_meta.npz")
        
        # Meta-Daten definieren
        meta = {
            'run_name': 'meta_test',
            'fs': 20e6,
            'dt_s': 5e-8,
            'trigger_level_v': -0.2,
            'ch_a': {'coupling': 'AC', 'v_range': 0.05},
            'ch_b': {'coupling': 'AC', 'v_range': 10.0}
        }
        
        # Dummy-Daten speichern
        t = np.array([0.0])
        u = np.array([0.0])
        i = np.array([0.0])
        
        save_pulse_npz(npz_path, pulse_id=1, t=t, u=u, i=i, meta=meta)
        
        try:
            # Meta-Daten laden
            meta_loaded = load_meta_npz(npz_path)
            
            # Prüfen
            assert meta_loaded.get('run_name') == meta['run_name'], "run_name stimmt nicht"
            assert meta_loaded.get('fs') == meta['fs'], "fs stimmt nicht"
            assert meta_loaded.get('trigger_level_v') == meta['trigger_level_v'], "trigger_level_v stimmt nicht"
            
            print("✓ Test erfolgreich (Meta-Daten korrekt)")
            return True
        
        except Exception as e:
            print(f"✗ Test fehlgeschlagen: {e}")
            import traceback
            traceback.print_exc()
            return False


def run_all_tests():
    """
    Führt alle Tests aus.
    
    Returns
    -------
    bool
        True wenn alle Tests erfolgreich, False sonst.
    """
    results = []
    
    results.append(test_save_load_pulse_npz())
    results.append(test_append_pulse_npz())
    results.append(test_npz_with_meta())
    
    print("\n=== Test-Zusammenfassung ===")
    passed = sum(results)
    total = len(results)
    print(f"Bestanden: {passed}/{total}")
    
    return all(results)


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)

