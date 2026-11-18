"""Serve the DUK simulation dashboard on the local machine."""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from simulation_adapter import DashboardSimulation

HERE = Path(__file__).resolve().parent
STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/plots.js": ("plots.js", "text/javascript; charset=utf-8"),
}


class DashboardServer(ThreadingHTTPServer):
    def __init__(self, port: int) -> None:
        self.simulation = DashboardSimulation(HERE.parents[1])
        super().__init__(("127.0.0.1", port), DashboardHandler)


class DashboardHandler(BaseHTTPRequestHandler):
    server: DashboardServer

    def _send(self, status: int, content_type: str, data: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, status: int, value: dict) -> None:
        data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self._send(status, "application/json; charset=utf-8", data)

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/api/config":
            self._json(200, self.server.simulation.config)
        elif path in STATIC:
            filename, content_type = STATIC[path]
            self._send(200, content_type, (HERE / "static" / filename).read_bytes())
        else:
            self._json(404, {"error": "Không tìm thấy đường dẫn."})

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        endpoints = {
            "/api/kinematics": self.server.simulation.kinematics,
            "/api/balance": self.server.simulation.balance,
        }
        if path not in endpoints:
            self._json(404, {"error": "Không tìm thấy phép mô phỏng."})
            return
        if self.headers.get_content_type() != "application/json":
            self._json(415, {"error": "Dữ liệu đầu vào phải dùng JSON."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("Kích thước dữ liệu đầu vào không hợp lệ.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("Cấu hình phải là một đối tượng JSON.")
            self._json(200, endpoints[path](payload))
        except (UnicodeError, json.JSONDecodeError):
            self._json(400, {"error": "Dữ liệu JSON không hợp lệ."})
        except (ValueError, OverflowError, RecursionError) as error:
            self._json(400, {"error": str(error)})
        except RuntimeError as error:
            self._json(422, {"error": f"Bộ mô phỏng dừng: {error}"})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=8765, type=int, help="Loopback port")
    args = parser.parse_args()
    with DashboardServer(args.port) as server:
        print(f"DUK Simulation Lab: http://127.0.0.1:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
