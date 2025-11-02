import os
import json
import numpy as np
from datetime import datetime

# ---------- CSV-Helfer ----------
def _csv_header(run_name: str, i_unit: str) -> str:
    """
    Erzeugt den Header für neue CSV-Datei.
    Wird nur geschrieben, wenn Datei noch nicht existiert. 
    """
    return (
        f"# RUN_NAME={run_name}\n"
        f"# created={datetime.now().isoformat()}\n"
        f"# columns: pulse_id,sample_idx,time_s,u_V,i_{i_unit}\n"
    )

def ensure_csv(csv_path: str, run_name: str, i_unit: str) -> str:
    """
    Legt CSV mit Header an, falls sie noch nicht existiert.
    """
    if not os.path.exists(csv_path):
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write(_csv_header(run_name, i_unit))

def scan_next_pulse_id(csv_path: str) -> int:
    """
    Liest aus bestehender CSV die nächste freie pulse_id aus der CSV zurück.
    Wird nur einmal pro Mess-Session am Anfang aufgerufen. Da danach automatisch fortlaufend. 
    """
    if not os.path.exists(csv_path):
        return 1
    
    last_id = 0
    with open(csv_path, "r", encoding="utf-8") as f:
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
    csv_path MUSS existieren oder vorher mit ensure_csv() angelegt worden sein.
    """
    n = len(t)
    data = np.column_stack([
        np.full(n, pulse_id, dtype=np.int64),
        np.arange(n, dtype=np.int64),
        t.astype(np.float64, copy=False),
        u.astype(np.float64, copy=False),
        i.astype(np.float64, copy=False),
    ])
    with open(csv_path, "a", encoding="utf-8") as f:
        np.savetxt(
            f, 
            data, 
            delimiter=",",
            fmt=["%d", "%d", "%.9e", "%.9e", "%.9e"]
        )

def write_meta_once(meta_path: str, run_name: str, csv_path: str, meta: dict) -> None:
    """
    Schreibt/aktualisiert eine Meta-JSON zum Run (Sampling, Bereiche etc.).
    runame, csv_path werden automatisch ergänzt.
    """
    meta_out = dict(meta)
    meta_out["run_name"] = run_name
    meta_out["csv_path"] = csv_path
    meta_out["created_or_updated"] = datetime.now().isoformat()
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2)

