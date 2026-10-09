"""Salt okunur panel önizlemesi. Gerçek kart yok; demo kullanıcı düğmesiyle açılır.

Çalıştır: python tools/preview_drone_web.py
Yalnız localhost: http://127.0.0.1:8765
"""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json

WEB = Path(__file__).resolve().parents[1] / "components" / "drone_web" / "web"


class PreviewHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB), **kwargs)

    def do_GET(self):
        if self.path.split("?")[0] == "/api/telemetry":
            payload = json.dumps({"valid": False, "age_ms": -1}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        else:
            super().do_GET()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    print("Flight Deck preview: http://127.0.0.1:8765 (kart verisi yok)", flush=True)
    ThreadingHTTPServer(("127.0.0.1", 8765), PreviewHandler).serve_forever()
