#!/usr/bin/env python3
"""
Web dashboard for a Solis hybrid inverter.

Polls the inverter over Modbus RTU (using solis_modbus.py) in a background
thread and serves a live dashboard plus a JSON API.

  python webapp.py --port /dev/serial0              # real inverter
  python webapp.py --demo                           # simulated data, no hardware

Then open http://<pi-address>:8080 in a browser.

Endpoints:
  /              dashboard
  /api/data      latest reading (JSON)
  /api/history   power/SOC history since start (JSON)
"""

from __future__ import annotations

import argparse
import json
import math
import random
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import solis_modbus as sm

WEB_DIR = Path(__file__).resolve().parent / "web"
HISTORY_POINTS = 8640  # 24 h at 10 s


# --------------------------------------------------------------------------
# Demo inverter (simulated registers)
# --------------------------------------------------------------------------

class DemoClient:
    """Produces plausible hybrid inverter registers following the time of day."""

    def __init__(self):
        self.soc = 55.0
        self.last = time.time()
        self.e = {"pv": 0.0, "chg": 0.0, "dis": 0.0, "imp": 0.0, "exp": 0.0, "load": 0.0}

    def close(self) -> None:
        pass

    def _regs(self) -> dict[int, int]:
        now = time.time()
        dt_h = min(now - self.last, 60) / 3600
        self.last = now
        t = datetime.now()
        hour = t.hour + t.minute / 60

        pv = max(0.0, math.sin(math.pi * (hour - 6.5) / 12.5)) * 4600 * random.uniform(0.85, 1.0)
        load = 350 + 250 * random.random() + (1800 if 18 <= hour < 21 else 0) + (900 if random.random() < 0.1 else 0)
        backup = 120 + 30 * random.random()
        home = load + backup
        surplus = pv - home
        if surplus > 0 and self.soc < 100:
            batt = min(surplus, 3000)          # + charge
        elif surplus < 0 and self.soc > 20:
            batt = max(surplus, -3000)         # - discharge
        else:
            batt = 0.0
        grid = pv - home - batt                # + export
        self.soc = max(0.0, min(100.0, self.soc + batt * dt_h / 100))

        for k, v in (("pv", pv), ("load", home), ("chg", max(batt, 0)), ("dis", max(-batt, 0)),
                     ("exp", max(grid, 0)), ("imp", max(-grid, 0))):
            self.e[k] += v * dt_h / 1000

        bv = 48 + self.soc * 0.06
        r: dict[int, int] = {}

        def u32(a, v):
            v = int(v) & 0xFFFFFFFF
            r[a], r[a + 1] = v >> 16, v & 0xFFFF

        def s16(a, v):
            r[a] = int(v) & 0xFFFF

        r[35000] = 0x2190
        for i, ch in enumerate(struct.unpack(">8H", b"DEMO000000000001")):
            r[33004 + i] = ch
        r.update({33001: 0x0D, 33021: 0x05, 33002: 0x1C, 33069: 0x06, 33067: 600, 33042: 261, 33041: 1250,
                  33048: 1, 33092: 0x13, 33095: 0x0003, 33121: 0x0001, 33094: 5000 + random.randint(-4, 4)})
        u32(33057, pv)
        for i, share in ((0, 0.55), (1, 0.45)):
            v = 360 + 20 * random.random() if pv > 0 else 0
            r[33049 + 2 * i] = int(v * 10)
            r[33050 + 2 * i] = int(pv * share / v * 10) if v else 0
        r[33035] = int(self.e["pv"] * 10)
        u32(33029, 18234 + self.e["pv"])
        r[33139], r[33140], r[33133] = int(self.soc), 98, int(bv * 10)
        r[33134], r[33135] = int(abs(batt) / bv * 10), 0 if batt >= 0 else 1
        u32(33149, abs(batt))
        s16(33043, 255)
        s16(33046, 341)
        r[34348], r[34349] = 3280, 3310
        r[33163], r[33167] = int(self.e["chg"] * 10), int(self.e["dis"] * 10)
        u32(33161, 4120 + self.e["chg"])
        u32(33165, 3890 + self.e["dis"])
        inv = pv - batt
        u32(33151, inv)
        u32(33079, inv)
        u32(33083, abs(inv))
        r[33073] = 2300 + random.randint(-30, 30)
        r[33076] = int(abs(inv) / 230 * 10)
        s16(33093, 380 + random.randint(-5, 5))
        r[33147] = int(load)
        s16(33148, backup)
        r[33137], r[33138] = 2300, int(backup / 230 * 10)
        r[33179] = int(self.e["load"] * 10)
        u32(33177, 21300 + self.e["load"])
        r[33171], r[33175] = int(self.e["imp"] * 10), int(self.e["exp"] * 10)
        u32(33169, 9100 + self.e["imp"])
        u32(33173, 7600 + self.e["exp"])
        u32(33263, grid)
        r[33251], r[33252] = r[33073], int(abs(grid) / 230 * 100)
        r[33282] = r[33094]
        r.update({43000: t.year % 100, 43001: t.month, 43002: t.day, 43003: t.hour, 43004: t.minute,
                  43005: t.second, 43006: 1, 43007: 0xBE, 43009: 0x0001, 43010: 100, 43011: 20, 43018: 10,
                  43024: 80, 43110: 0b100001, 43117: 1000, 43118: 1000, 43111: 1, 43112: 2300})
        return r

    def read(self, fc: int, addr: int, count: int) -> list[int]:
        if not hasattr(self, "_cache") or time.monotonic() - self._cache[0] > 1:
            self._cache = (time.monotonic(), self._regs())
        regs = self._cache[1]
        return [regs.get(a, 0) for a in range(addr, addr + count)]


# --------------------------------------------------------------------------
# Background poller
# --------------------------------------------------------------------------

def _val(results: dict, key: str, default=None):
    r = results.get(key)
    return r.value if r is not None and r.raw is not None and r.value is not None else default


class Poller(threading.Thread):
    def __init__(self, args):
        super().__init__(daemon=True)
        self.args = args
        self.lock = threading.Lock()
        self.latest: dict | None = None
        self.error: str | None = "starting"
        self.last_ok: float | None = None
        self.history: deque = deque(maxlen=HISTORY_POINTS)
        self.client = None

    def _connect(self):
        if self.args.demo:
            return DemoClient()
        return sm.RtuClient(self.args.port, self.args.unit, self.args.baud, self.args.timeout, self.args.retries)

    def run(self) -> None:
        groups = sm.ALL_GROUPS if self.args.groups == "all" else self.args.groups.split(",")
        while True:
            t0 = time.monotonic()
            try:
                if self.client is None:
                    self.client = self._connect()
                results = sm.poll(self.client, groups, lambda m: print(m, file=sys.stderr))
                data = sm.to_dict(results)
                grid_load = _val(results, "grid_load_power", 0)
                backup = _val(results, "backup_power", 0)
                point = {
                    "t": int(time.time() * 1000),
                    "pv": _val(results, "pv_power"),
                    "battery": _val(results, "battery1_power_signed"),
                    "grid": _val(results, "meter_active_power"),
                    "load": grid_load + max(backup, 0),
                    "soc": _val(results, "battery_soc"),
                }
                with self.lock:
                    self.latest, self.error, self.last_ok = data, None, time.time()
                    self.history.append(point)
            except sm.NoResponse:
                with self.lock:
                    self.error = "Inverter not responding (check wiring, slave address, baud rate)"
            except Exception as e:  # serial port missing, unplugged, permissions...
                with self.lock:
                    self.error = f"{type(e).__name__}: {e}"
                if self.client is not None:
                    try:
                        self.client.close()
                    except Exception:
                        pass
                    self.client = None
            time.sleep(max(1.0, self.args.interval - (time.monotonic() - t0)))

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "ok": self.error is None,
                "error": self.error,
                "last_ok": self.last_ok,
                "interval": self.args.interval,
                "demo": self.args.demo,
                "titles": sm.GROUPS,
                "data": self.latest,
            }

    def history_list(self) -> list:
        with self.lock:
            return list(self.history)


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

def make_handler(poller: Poller):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/api/data":
                self._send(200, json.dumps(poller.snapshot()).encode(), "application/json")
            elif path == "/api/history":
                self._send(200, json.dumps(poller.history_list()).encode(), "application/json")
            elif path in ("/", "/index.html"):
                self._send(200, (WEB_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
            else:
                self._send(404, b"not found", "text/plain")

        def log_message(self, fmt, *args):
            pass  # keep the console quiet

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description="Web dashboard for a Solis hybrid inverter.")
    ap.add_argument("--port", help="serial port, e.g. /dev/serial0, /dev/ttyUSB0, COM5")
    ap.add_argument("--unit", type=int, default=1, help="inverter slave address (default 1)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--retries", type=int, default=1)
    ap.add_argument("--interval", type=float, default=10, help="seconds between polls (default 10)")
    ap.add_argument("--groups", default=",".join(sm.DEFAULT_GROUPS), help="register groups to read, or 'all'")
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default all interfaces)")
    ap.add_argument("--http-port", type=int, default=8080, help="web server port (default 8080)")
    ap.add_argument("--demo", action="store_true", help="use simulated data instead of a real inverter")
    args = ap.parse_args()
    if not args.demo and not args.port:
        ap.error("--port is required (or use --demo)")

    poller = Poller(args)
    poller.start()
    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(poller))
    src = "demo data" if args.demo else f"{args.port} unit {args.unit}"
    print(f"Solis dashboard on http://{args.host}:{args.http_port}  ({src}, every {args.interval:g}s)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
