"""
Einfacher USB TC-08 Temperaturlogger
- Live-Temperaturmessung während der Puls-Messung
- Läuft in separatem Thread im Hintergrund
"""

import ctypes as ct
import threading
import time
from typing import Optional

# USB TC-08 SDK (Pico Technology Python SDK)
try:
    from picosdk.usbtc08 import usbtc08 as tc08
    from picosdk.functions import assert_pico2000_ok
    TC08_SDK_AVAILABLE = True
except (ImportError, OSError) as e:
    # SDK nicht verfügbar (z.B. nicht installiert oder DLL nicht gefunden)
    print(f"[Warnung] USB TC-08 SDK nicht verfügbar: {e}")
    print("[Warnung] Temperaturlogger wird im Mock-Modus laufen")
    tc08 = None
    assert_pico2000_ok = lambda x: None  # Dummy-Funktion
    TC08_SDK_AVAILABLE = False


class TempLogger:
    """
    Einfacher Temperaturlogger für USB TC-08.
    
    Beispiel:
        logger = TempLogger()
        logger.start()
        # ... während Puls-Messung ...
        temp = logger.get_current_temp(channel=1)  # Kanal 1
        logger.stop()
    """
    
    def __init__(self, update_interval_s: float = 0.5):
        """
        Initialisiert den Temperaturlogger.
        
        Args:
            update_interval_s: Intervall für Temperaturabfrage in Sekunden
        """
        self.update_interval_s = update_interval_s
        self.handle = None
        self.is_running = False
        self._lock = threading.Lock()
        self._temp_values = {}  # {channel: (temp_celsius, timestamp)}
        self._temp_history = []  # Liste von (timestamp, {channel: temp}) für Zeitverlauf
        self._max_history = 1000  # Maximale Anzahl Einträge in Historie
        self._thread = None
        
        # Callback für Live-Updates (wird bei jedem neuen Messwert aufgerufen)
        self.on_update_callback = None  # Callback: (channel, temp_celsius, timestamp) -> None
        
    def open(self) -> bool:
        """
        Öffnet die Verbindung zum USB TC-08 Gerät.
        
        Returns:
            True wenn erfolgreich, False sonst
        """
        if not TC08_SDK_AVAILABLE:
            print("[TempLogger] Mock-Modus: SDK nicht verfügbar")
            return False
            
        try:
            # usb_tc08_open_unit gibt direkt einen Handle zurück (int16)
            status = tc08.usb_tc08_open_unit()
            assert_pico2000_ok(status)
            self.handle = status
            print(f"[TempLogger] Gerät geöffnet, Handle: {self.handle}")
            
            # Setze Mains-Rejection auf 50 Hz (0 = 50 Hz, 1 = 60 Hz)
            status_mains = tc08.usb_tc08_set_mains(self.handle, 0)
            assert_pico2000_ok(status_mains)
            
            return True
        except Exception as e:
            print(f"[TempLogger] Fehler beim Öffnen: {e}")
            return False
    
    def close(self) -> None:
        """Schließt die Verbindung zum Gerät."""
        if self.handle is not None and TC08_SDK_AVAILABLE:
            try:
                status = tc08.usb_tc08_close_unit(self.handle)
                assert_pico2000_ok(status)
                print("[TempLogger] Gerät geschlossen")
            except Exception as e:
                print(f"[TempLogger] Fehler beim Schließen: {e}")
            self.handle = None
    
    def start(self) -> bool:
        """
        Startet die kontinuierliche Temperaturmessung im Hintergrund.
        
        Returns:
            True wenn erfolgreich, False sonst
        """
        if self.is_running:
            print("[TempLogger] Läuft bereits")
            return True
            
        if not self.open():
            # Mock-Modus: Starte trotzdem für Tests
            print("[TempLogger] Starte im Mock-Modus")
        
        self.is_running = True
        self._thread = threading.Thread(target=self._measurement_loop, daemon=True)
        self._thread.start()
        print("[TempLogger] Messung gestartet")
        return True
    
    def stop(self) -> None:
        """Stoppt die Temperaturmessung."""
        self.is_running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.close()
        print("[TempLogger] Messung gestoppt")
    
    def get_current_temp(self, channel: int = 1) -> Optional[float]:
        """
        Gibt die aktuelle Temperatur eines Kanals zurück.
        
        Args:
            channel: Kanal-Nummer (1-8)
            
        Returns:
            Temperatur in °C oder None falls nicht verfügbar
        """
        with self._lock:
            if channel in self._temp_values:
                temp, _ = self._temp_values[channel]
                return temp
        return None
    
    def get_all_temps(self) -> dict:
        """
        Gibt alle aktuellen Temperaturwerte zurück.
        
        Returns
        -------
        dict
            Dictionary mit {channel: temp_celsius}
            Nur Kanäle mit aktuellen Werten werden zurückgegeben.
        
        Examples
        --------
        >>> temps = logger.get_all_temps()
        >>> print(f"Aktuelle Temperaturen: {temps}")
        """
        with self._lock:
            return {ch: temp for ch, (temp, _) in self._temp_values.items()}
    
    def get_temperature_history(self, channel: int = 1, max_points: int = None) -> tuple:
        """
        Gibt den Temperaturverlauf eines Kanals zurück.
        
        Diese Funktion ist nützlich für Live-Plots in der GUI, da sie
        eine Zeitreihe der Temperaturwerte liefert.
        
        Parameters
        ----------
        channel : int, optional
            Kanal-Nummer (Standard: 1).
        max_points : int, optional
            Maximale Anzahl zurückzugebender Punkte (Standard: alle).
            Falls angegeben, werden die neuesten N Punkte zurückgegeben.
        
        Returns
        -------
        tuple
            (timestamps, temperatures) - Beide als numpy-Arrays
            - timestamps: Zeitpunkte in Sekunden (relativ zum Start)
            - temperatures: Temperaturwerte in °C
        
        Examples
        --------
        >>> ts, temps = logger.get_temperature_history(channel=1, max_points=100)
        >>> # Plotten: plt.plot(ts, temps)
        """
        import numpy as np
        
        with self._lock:
            # Historie filtern nach Kanal
            channel_data = []
            for ts, temps_dict in self._temp_history:
                if channel in temps_dict:
                    channel_data.append((ts, temps_dict[channel]))
            
            if not channel_data:
                # Keine Daten: leere Arrays zurückgeben
                return np.array([]), np.array([])
            
            # Sortieren nach Zeit
            channel_data.sort(key=lambda x: x[0])
            
            # Limitiere auf max_points (neueste Punkte)
            if max_points is not None and len(channel_data) > max_points:
                channel_data = channel_data[-max_points:]
            
            # Arrays extrahieren
            timestamps = np.array([ts for ts, _ in channel_data])
            temperatures = np.array([temp for _, temp in channel_data])
            
            # Relative Zeit (erster Eintrag = 0)
            if len(timestamps) > 0:
                timestamps = timestamps - timestamps[0]
            
            return timestamps, temperatures
    
    def set_callback(self, callback):
        """
        Setzt einen Callback für Live-Temperatur-Updates.
        
        Der Callback wird bei jedem neuen Messwert aufgerufen (asynchron
        im Hintergrund-Thread). Nützlich für Live-Updates in der GUI.
        
        Parameters
        ----------
        callback : callable, optional
            Funktion mit Signatur: (channel, temp_celsius, timestamp) -> None
            - channel: int - Kanal-Nummer (1-8)
            - temp_celsius: float - Temperatur in °C
            - timestamp: float - Zeitstempel (time.time())
            Falls None: Callback wird entfernt.
        
        Examples
        --------
        >>> def on_temp_update(channel, temp, ts):
        ...     print(f"Kanal {channel}: {temp:.2f} °C")
        >>> logger.set_callback(on_temp_update)
        
        Notes
        -----
        - Der Callback wird im Hintergrund-Thread aufgerufen.
        - Für GUI-Updates sollte der Callback thread-sicher sein (z.B. Queue verwenden).
        """
        self.on_update_callback = callback
    
    def _measurement_loop(self) -> None:
        """Hintergrund-Thread: Liest kontinuierlich Temperaturwerte."""
        if not TC08_SDK_AVAILABLE:
            # Mock-Modus für Tests
            self._mock_measurement_loop()
            return
            
        # Konfiguriere Kanal 1 mit Thermoelement-Typ K
        try:
            # Thermoelement-Typen: B=66, E=69, J=74, K=75, N=78, R=82, S=83, T=84, ' '=32, X=88
            typeK = ct.c_int8(75)
            status_set = tc08.usb_tc08_set_channel(self.handle, 1, typeK)
            assert_pico2000_ok(status_set)
            
            # Hole Minimum-Intervall (in ms)
            min_interval_ms = tc08.usb_tc08_get_minimum_interval_ms(self.handle)
            assert_pico2000_ok(min_interval_ms)
            min_interval_s = min_interval_ms / 1000.0
            
            # Stelle sicher, dass update_interval_s >= min_interval_s
            if self.update_interval_s < min_interval_s:
                print(f"[TempLogger] Update-Intervall zu klein ({self.update_interval_s}s), "
                      f"verwende Minimum: {min_interval_s}s")
                self.update_interval_s = min_interval_s
                
        except Exception as e:
            print(f"[TempLogger] Fehler bei Konfiguration: {e}")
            return
        
        # Einheiten: Celsius
        units = tc08.USBTC08_UNITS["USBTC08_UNITS_CENTIGRADE"]
        
        while self.is_running:
            try:
                # usb_tc08_get_single liest eine Einzelmessung
                # temp ist ein Array von 9 floats: [Cold Junction, Channel 1-8]
                temp = (ct.c_float * 9)()
                overflow = ct.c_int16(0)
                
                status_get = tc08.usb_tc08_get_single(
                    self.handle,
                    ct.byref(temp),
                    ct.byref(overflow),
                    units
                )
                assert_pico2000_ok(status_get)
                
                # Kanal 1 ist temp[1] (temp[0] ist Cold Junction)
                temp_celsius = temp[1]
                timestamp = time.time()
                
                with self._lock:
                    # Aktuellen Wert speichern
                    self._temp_values[1] = (temp_celsius, timestamp)
                    
                    # Historie aktualisieren
                    self._temp_history.append((timestamp, {1: temp_celsius}))
                    
                    # Historie begrenzen (älteste Einträge entfernen)
                    if len(self._temp_history) > self._max_history:
                        self._temp_history = self._temp_history[-self._max_history:]
                
                # Callback aufrufen (außerhalb des Locks für Thread-Safety)
                if self.on_update_callback:
                    try:
                        self.on_update_callback(1, temp_celsius, timestamp)
                    except Exception as e:
                        print(f"[TempLogger] Callback-Fehler: {e}")
                        
            except Exception as e:
                print(f"[TempLogger] Fehler in Messschleife: {e}")
            
            time.sleep(self.update_interval_s)
    
    def _mock_measurement_loop(self) -> None:
        """Mock-Modus: Simuliert Temperaturwerte für Tests."""
        import random
        base_temp = 25.0  # Raumtemperatur
        
        while self.is_running:
            # Simuliere kleine Schwankungen
            temp = base_temp + random.uniform(-0.5, 0.5)
            timestamp = time.time()
            
            with self._lock:
                # Aktuellen Wert speichern
                self._temp_values[1] = (temp, timestamp)
                
                # Historie aktualisieren
                self._temp_history.append((timestamp, {1: temp}))
                
                # Historie begrenzen
                if len(self._temp_history) > self._max_history:
                    self._temp_history = self._temp_history[-self._max_history:]
            
            # Callback aufrufen (außerhalb des Locks)
            if self.on_update_callback:
                try:
                    self.on_update_callback(1, temp, timestamp)
                except Exception as e:
                    print(f"[TempLogger] Callback-Fehler (Mock): {e}")
            
            time.sleep(self.update_interval_s)


def test_temp_logger():
    """
    Einfache Test-Funktion für den Temperaturlogger.
    """
    print("\n=== TempLogger Test ===")
    
    logger = TempLogger(update_interval_s=0.2)
    
    try:
        # Starte Messung
        logger.start()
        time.sleep(1.0)  # Warte auf erste Messwerte
        
        # Lese Temperatur
        temp = logger.get_current_temp(channel=1)
        print(f"Temperatur Kanal 1: {temp:.2f} °C" if temp is not None else "Keine Temperatur verfügbar")
        
        # Lese alle Werte
        all_temps = logger.get_all_temps()
        print(f"Alle Temperaturen: {all_temps}")
        
        # Kontinuierliche Ausgabe für 3 Sekunden
        print("\nLive-Messung (3 Sekunden):")
        for i in range(6):
            temp = logger.get_current_temp(channel=1)
            print(f"  [{i*0.5:.1f}s] Temp: {temp:.2f} °C" if temp is not None else f"  [{i*0.5:.1f}s] Keine Messung")
            time.sleep(0.5)
        
    finally:
        logger.stop()
    
    print("=== Test beendet ===\n")


if __name__ == "__main__":
    test_temp_logger()

