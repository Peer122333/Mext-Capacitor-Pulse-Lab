from pico_pulse_lab.acquisition.picoscope_reader import Pico3205AReader, acquire_n_pulses
from pico_pulse_lab.acquisition.temp_logger import TempLogger, test_temp_logger
from pico_pulse_lab.control.pulse_controller import PulseController
from pico_pulse_lab.processing.fft import processing_worker  # wenn du es so nennst
from pico_pulse_lab.storage.csv_writer import storage_worker
from pico_pulse_lab.gui.app import run_app
import time


def example_with_temp_logger():
    """
    Beispiel: Puls-Messung mit paralleler Temperaturmessung.
    
    Zeigt, wie man den TempLogger während der Puls-Messung verwendet.
    """
    print("\n=== Puls-Messung mit Temperaturmessung ===")
    
    # Temperaturlogger starten
    temp_logger = TempLogger(update_interval_s=0.5)
    temp_logger.start()
    
    try:
        print("Temperaturmessung gestartet")
        time.sleep(0.5)  # Warte auf erste Messwerte
        
        # Zeige aktuelle Temperatur
        temp = temp_logger.get_current_temp(channel=1)
        print(f"Starttemperatur: {temp:.2f} °C" if temp is not None else "Keine Temperatur verfügbar")
        
        # Starte Puls-Messung (kann parallel laufen)
        print("\nStarte Puls-Messung...")
        # acquire_n_pulses(n_pulses=3)  # Deaktiviert für Demo
        
        # Während der Messung: Temperatur abfragen
        for i in range(6):
            temp = temp_logger.get_current_temp(channel=1)
            print(f"  [Messung {i+1}] Temp: {temp:.2f} °C" if temp is not None else f"  [Messung {i+1}] Keine Messung")
            time.sleep(0.5)
        
        print("\nPuls-Messung abgeschlossen")
        
        # Endtemperatur
        temp = temp_logger.get_current_temp(channel=1)
        print(f"Endtemperatur: {temp:.2f} °C" if temp is not None else "Keine Temperatur verfügbar")
        
    finally:
        # Temperaturlogger stoppen
        temp_logger.stop()
    
    print("=== Beispiel beendet ===\n")


if __name__ == "__main__":
    # Option 1: Test-Funktion
    # test_temp_logger()
    
    # Option 2: Beispiel mit Puls-Messung
    # example_with_temp_logger()
    
    # Option 3: Normale Puls-Messung (aus picoscope_reader.py)
    # acquire_n_pulses(n_pulses=3)

    