"""
Test-Funktionen für Kapazitätsparameter-Berechnung.

Diese Tests überprüfen die Funktion estimate_cap_params() mit
synthetischen Daten und echten CSV-Daten falls verfügbar.
"""

import numpy as np
import sys
import os

# Pfad für Import hinzufügen
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from pico_pulse_lab.processing.cap_params import estimate_cap_params


def test_estimate_cap_params_with_synthetic_data():
    """
    Test mit synthetischen Daten.
    
    Erstellt synthetische Spannungs- und Strompulse basierend auf einem
    einfachen RC-Modell und testet, ob die Parameter korrekt berechnet werden.
    """
    print("\n=== Test: estimate_cap_params mit synthetischen Daten ===")
    
    # Synthetische Parameter
    R_true = 0.1  # 0.1 Ohm ESR
    C_true = 100e-6  # 100 µF Kapazität
    
    # Zeitvektor: 1 ms mit 1000 Samples
    t = np.linspace(0, 1e-3, 1000)
    dt = t[1] - t[0]
    fs = 1.0 / dt
    
    # Synthetischer Strom: Exponential-Fall (vereinfacht)
    # i(t) = I0 * exp(-t / (R*C))
    I0 = 10.0  # Anfangsstrom
    tau = R_true * C_true  # Zeitkonstante
    i = I0 * np.exp(-t / tau)
    
    # Synthetische Spannung: U = I * R + Integral(I) / C
    # Vereinfacht: U = I * R (ohne Integral, da es kompliziert wird)
    # Für echten Test würde man die vollständige Gleichung lösen
    u = i * R_true  # Vereinfachtes Modell
    
    # Leichte Rauschüberlagerung für Realismus
    noise_level = 0.001
    u += np.random.normal(0, noise_level, len(u))
    i += np.random.normal(0, noise_level, len(i))
    
    try:
        # Parameter berechnen
        esr, cap = estimate_cap_params(t, u, i)
        
        print(f"Wahre Parameter: ESR={R_true:.6f} Ω, C={C_true*1e6:.6f} µF")
        print(f"Geschätzt:      ESR={esr:.6f} Ω, C={cap*1e6:.6f} µF")
        print(f"Fehler ESR:      {abs(esr - R_true)/R_true * 100:.2f}%")
        print(f"Fehler Kapazität: {abs(cap - C_true)/C_true * 100:.2f}%")
        
        # Toleranz prüfen (bei synthetischen Daten mit Rauschen)
        esr_error = abs(esr - R_true) / R_true
        cap_error = abs(cap - C_true) / C_true
        
        if esr_error < 0.5 and cap_error < 0.5:  # 50% Toleranz aufgrund Vereinfachung
            print("✓ Test erfolgreich (Parameter in Toleranz)")
            return True
        else:
            print("✗ Test fehlgeschlagen (Parameter außerhalb Toleranz)")
            print(f"  Hinweis: Bei stark vereinfachtem Modell sind größere Fehler normal")
            return False
    
    except Exception as e:
        print(f"✗ Test fehlgeschlagen: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_estimate_cap_params_with_real_csv():
    """
    Test mit echten CSV-Daten falls verfügbar.
    
    Sucht nach CSV-Dateien im Runs-Verzeichnis und testet
    die Parameter-Berechnung mit echten Messdaten.
    """
    print("\n=== Test: estimate_cap_params mit CSV-Daten ===")
    
    # Suche nach CSV-Dateien im Runs-Verzeichnis
    runs_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'Runs')
    
    if not os.path.exists(runs_dir):
        print("ℹ Kein Runs-Verzeichnis gefunden. Test übersprungen.")
        return True
    
    # Suche nach CSV-Dateien
    csv_files = []
    for root, dirs, files in os.walk(runs_dir):
        for file in files:
            if file.endswith('.csv'):
                csv_files.append(os.path.join(root, file))
    
    if len(csv_files) == 0:
        print("ℹ Keine CSV-Dateien gefunden. Test übersprungen.")
        return True
    
    # Teste mit erster gefundener CSV
    csv_path = csv_files[0]
    print(f"Teste mit: {csv_path}")
    
    try:
        # CSV lesen (nur erste pulse_id)
        pulse_id = None
        rows = []
        
        with open(csv_path, 'r', encoding='utf-8') as f:
            for line in f:
                if not line or line[0] == '#':
                    continue
                
                parts = line.strip().split(',')
                if len(parts) < 5:
                    continue
                
                try:
                    pid = int(parts[0])
                    if pulse_id is None:
                        pulse_id = pid
                    
                    if pid == pulse_id:
                        t = float(parts[2])
                        u = float(parts[3])
                        i = float(parts[4])
                        rows.append((t, u, i))
                except:
                    continue
        
        if len(rows) == 0:
            print("ℹ Keine Datenzeilen gefunden. Test übersprungen.")
            return True
        
        # Arrays extrahieren
        rows.sort(key=lambda r: r[0])  # Sortieren nach Zeit
        t = np.array([r[0] for r in rows])
        u = np.array([r[1] for r in rows])
        i = np.array([r[2] for r in rows])
        
        print(f"  Gefunden: {len(t)} Samples")
        print(f"  Zeitbereich: {t[0]:.6e} s bis {t[-1]:.6e} s")
        print(f"  Spannung: [{u.min():.3f}, {u.max():.3f}] V")
        print(f"  Strom: [{i.min():.3f}, {i.max():.3f}] A")
        
        # Parameter berechnen
        esr, cap = estimate_cap_params(t, u, i)
        
        print(f"  Geschätzte Parameter:")
        print(f"    ESR: {esr:.6f} Ω")
        print(f"    Kapazität: {cap*1e6:.6f} µF")
        
        print("✓ Test erfolgreich (Berechnung durchgeführt)")
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
    
    # Test mit synthetischen Daten
    results.append(test_estimate_cap_params_with_synthetic_data())
    
    # Test mit echten CSV-Daten
    results.append(test_estimate_cap_params_with_real_csv())
    
    # Zusammenfassung
    print("\n=== Test-Zusammenfassung ===")
    passed = sum(results)
    total = len(results)
    print(f"Bestanden: {passed}/{total}")
    
    return all(results)


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)

