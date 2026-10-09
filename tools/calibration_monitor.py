"""Kart raporunu gösterir ve calibration.txt dosyasını son değerlerle günceller.

ESP-IDF terminali: python tools/calibration_monitor.py --port COM17
Ctrl+C çıkış. --once: ilk tam raporu kaydet ve çık.
"""
import argparse
import os
from pathlib import Path
import time

import serial
from esptool.reset import HardReset


def save_report(path: Path, lines: list[str]) -> None:
    text = "\n".join(lines) + "\n"
    if not text.startswith("#") or "format = 1\n" not in text or "sensor = " not in text:
        raise ValueError("Eksik kalibrasyon raporu; dosya degistirilmedi.")
    # Tam raporu atomik yaz; önceki kullanıcı dosyası geri alınabilsin.
    if path.exists():
        previous = path.read_text(encoding="utf-8")
        if previous == text:
            return
        path.with_suffix(".previous.txt").write_text(previous, encoding="utf-8")
    temporary = path.with_suffix(".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)
    print(f"\nKALIBRASYON DOSYASI GUNCELLENDI: {path}\n", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "calibration.txt")
    args = parser.parse_args()
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, 115200, 0.3
    port.dtr = port.rts = False
    port.open()
    report = None
    pending = b""
    deadline = time.monotonic() + args.timeout
    try:
        HardReset(port)()
        while not args.once or time.monotonic() < deadline:
            pending += port.read(port.in_waiting or 1)
            while b"\n" in pending:
                raw, pending = pending.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                print(line, flush=True)
                if line in ("CALIBRATION_BEGIN", "CALIBRATION_UPDATE_BEGIN"):
                    report = []
                elif line in ("CALIBRATION_END", "CALIBRATION_UPDATE_END") and report is not None:
                    save_report(args.output, report)
                    report = None
                    if args.once and line == "CALIBRATION_END":
                        return
                elif report is not None:
                    report.append(line)
        raise TimeoutError("Tam rapor alinmadi; mevcut dosya korundu.")
    except KeyboardInterrupt:
        print("\nMonitor kapandi. Son tam rapor dosyada korundu.")
    finally:
        port.close()


if __name__ == "__main__":
    main()
