#!/usr/bin/env python3
"""
Solis hybrid (energy storage) inverter reader - Modbus RTU over RS485.

Implements the read side of "RS485_MODBUS (ESINV-33000ID) Hybrid Inverter",
Communication Protocol for Energy Storage Inverters Ver3.5.

Protocol notes (from the PDF):
  * 9600 bps, 8N1, default slave address 1
  * >= 300 ms between query frames
  * Register addresses are sent exactly as listed (no -1 offset):
    33001 is requested as 0x80E9 (protocol example 7.1)
  * FC04 input registers: 33000-36999, 25000-25999 (live data)
    FC03 holding registers: 43000-44999 (settings)
  * Unimplemented registers return 0x0000 instead of an exception
  * U32/S32: high word first. 0x80000000 marks an invalid S32 value

Only dependency: pyserial  (pip install pyserial)

Examples:
  python solis_modbus.py --list-ports
  python solis_modbus.py --port /dev/serial0
  python solis_modbus.py --port COM5 --groups battery,pv,grid,load
  python solis_modbus.py --port COM5 --groups all
  python solis_modbus.py --port COM5 --json
  python solis_modbus.py --port COM5 --interval 60 --csv solis_log.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable

# --------------------------------------------------------------------------
# Modbus RTU transport
# --------------------------------------------------------------------------

MAX_REGS_PER_READ = 50
INTER_FRAME_DELAY = 0.30  # seconds, protocol section 3.2
NO_REPLY_ABORT = 2        # give up a poll after this many timeouts with no reply at all

EXCEPTION_TEXT = {
    0x01: "illegal function code",
    0x02: "illegal data address",
    0x03: "illegal data value",
    0x04: "service fault",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", crc16(frame))  # CRC is sent low byte first


class ModbusError(Exception):
    pass


class ModbusException(ModbusError):
    """Inverter answered with an exception response."""

    def __init__(self, code: int):
        self.code = code
        super().__init__(f"exception 0x{code:02X} ({EXCEPTION_TEXT.get(code, 'unknown')})")


class ModbusTimeout(ModbusError):
    pass


class NoResponse(ModbusError):
    pass


class RtuClient:
    def __init__(self, port: str, unit: int, baud: int = 9600, timeout: float = 1.0, retries: int = 1):
        import serial  # imported here so --help works without pyserial

        self.unit = unit
        self.retries = retries
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self._last = 0.0

    def close(self) -> None:
        self.ser.close()

    def _wait_gap(self) -> None:
        dt = time.monotonic() - self._last
        if dt < INTER_FRAME_DELAY:
            time.sleep(INTER_FRAME_DELAY - dt)

    def _transact(self, fc: int, addr: int, count: int) -> list[int]:
        req = with_crc(struct.pack(">BBHH", self.unit, fc, addr, count))
        self._wait_gap()
        self.ser.reset_input_buffer()
        self.ser.write(req)
        self.ser.flush()
        try:
            head = self.ser.read(3)
            if len(head) < 3:
                raise ModbusTimeout(f"no response (got {len(head)} bytes)")
            rid, rfc, n = head
            if rfc == fc | 0x80:
                tail = self.ser.read(2)
                frame = head + tail
                if len(tail) < 2 or crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                    raise ModbusError("bad CRC on exception response")
                raise ModbusException(n)
            if rid != self.unit or rfc != fc:
                raise ModbusError(f"unexpected header {head.hex(' ')}")
            if n != count * 2:
                raise ModbusError(f"byte count {n}, expected {count * 2}")
            body = self.ser.read(n + 2)
            if len(body) < n + 2:
                raise ModbusTimeout("truncated response")
            frame = head + body
            if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                raise ModbusError("CRC mismatch")
            return list(struct.unpack(f">{count}H", body[:n]))
        finally:
            self._last = time.monotonic()

    def read(self, fc: int, addr: int, count: int) -> list[int]:
        last: Exception | None = None
        for _ in range(self.retries + 1):
            try:
                return self._transact(fc, addr, count)
            except ModbusException:
                raise  # deterministic answer, retrying won't help
            except ModbusError as e:
                last = e
        raise last  # type: ignore[misc]


# --------------------------------------------------------------------------
# Lookup tables (appendices)
# --------------------------------------------------------------------------

# Appendix 1 (M35000)
INVERTER_TYPE = {
    0x0000: "Undefined",
    0x2030: "5G/S5 single-phase LV battery hybrid",
    0x2031: "5G/S5 single-phase AC-coupled",
    0x2060: "5G three-phase HV battery hybrid",
    0x2070: "S6 three-phase HV battery 5-12kW",
    0x2071: "S6 three-phase HV battery 12-20kW", 0x2171: "S6 three-phase HV battery 12-20kW",
    0x2172: "S6 three-phase LV battery 10-15kW",
    0x2073: "S6 three-phase HV battery 29.9-150kW", 0x2173: "S6 three-phase HV battery 29.9-150kW",
    0x2273: "S6 three-phase HV battery 29.9-150kW",
    0x2074: "S6 three-phase HV 5-10kW AC-coupled Pro",
    0x2080: "S6 single-phase HV battery US",
    0x2181: "S6 single-phase LV battery 12-16kW US",
    0x2183: "S6 LV battery 5-8kW Brazil split-phase",
    0x2090: "S6 single-phase LV battery 3-18kW", 0x2190: "S6 single-phase LV battery 3-18kW",
    0x2091: "S6 single-phase LV battery AC-coupled",
    0x3010: "S5 single-phase LV battery off-grid",
    0x3020: "S6 single-phase LV battery off-grid", 0x3120: "S6 single-phase LV battery off-grid",
}

# Appendix 3 (M33095); 33070 holds the sub-code
STATUS = {
    0x0000: "Waiting", 0x0001: "OpenRun", 0x0002: "SoftRun", 0x0003: "Generating", 0x0004: "Off-Grid",
    0xF010: "Grid surge (Surge Alarm)", 0xF011: "Internal fan fault", 0xF013: "Grid SPD fault (VgSpdFail)",
    0xF014: "DC SPD fault (DcSpdFail)", 0xF015: "External fan fault", 0xF016: "Fuse fault",
    0xF017: "L&PE fail", 0xF019: "Boost fail", 0xF01A: "Generator start fail", 0xF01B: "Generator stop fail",
    0xF01C: "IGBT temperature difference", 0xF01D: "N-PE voltage fault", 0xF01E: "PV/battery power mismatch",
    0xF01F: "Generator phase error",
    **{0xF020 + i: f"BMS alarm (warning) {i + 1:02d}" for i in range(14)},
    0xF035: "AC Coupling lock", 0xF036: "AC Coupling stop", 0xF037: "Backup breaker overload",
    0xF040: "CT direction fail", 0xF041: "Meter phase sequence warning", 0xF042: "Slave phase sequence warning",
    0xF050: "Grid voltage unbalance warning", 0xF051: "DC switch abnormal warning",
    0xF080: "Grid-tied inverter comm fail", 0xF081: "Meter 2 fail", 0xF082: "Parallel version mismatch",
    0xF083: "Load cut-off (grid import over limit)", 0xF084: "Load cut-off lock",
    0x1010: "Grid over voltage (OV-G-V)", 0x1011: "Grid under voltage (UN-G-V)",
    0x1012: "Grid over frequency (OV-G-F)", 0x1013: "Grid under frequency (UN-G-F)",
    0x1014: "Grid reverse (Backfeed_Iac)", 0x1015: "No grid (NO-Grid)", 0x1016: "Grid unbalance (G-PHASE)",
    0x1017: "Grid frequency fluctuation (G-F-FLU)", 0x1018: "Grid over current (OV-G-I)",
    0x1019: "Grid current tracking fault (IGFOL-F)", 0x101A: "Grid phase fault",
    0x101C: "Overcurrent screening fault (IGBTSift-Pro)", 0x101D: "G100 voltage protection",
    0x101E: "Grid fluctuation abnormal (GV-F)",
    0x1020: "DC over voltage (OV-DC)", 0x1021: "DC bus over voltage (OV-BUS)",
    0x1022: "DC bus unbalance (UNB-BUS)", 0x1023: "DC bus under voltage (UN-BUS)",
    0x1024: "DC bus unbalance 2 (UNB2-BUS)", 0x1025: "DC channel A over current", 0x1026: "DC channel B over current",
    0x1027: "DC interference (DC-INTF)", 0x1028: "DC reverse (Reve-DC)", 0x1029: "PV mid-point grounding",
    0x102A: "Bus voltage inconsistent (Vbus-Sam)", 0x102B: "PV port surge", 0x102C: "DC over temperature (OV-TEM05)",
    0x102D: "MPPT over voltage", 0x102E: "MPPT over current",
    0x1030: "Grid interference (GRID-INTF)", 0x1031: "DSP init protection (INI-FAULT)",
    0x1032: "Over temperature (OV-TEM01)", 0x1033: "PV insulation fault (PV ISO-PRO)",
    0x1034: "Leakage current protection (ILeak-PRO)", 0x1035: "Relay check fail",
    0x1036: "DSP_B communication fault", 0x1037: "DC injection fault (DCInj-FAULT)",
    0x1038: "12V under voltage (12Power-FAULT)", 0x1039: "Leakage current self-check (ILeak-Check)",
    0x103A: "Under temperature (UN-TEM)", 0x103B: "IGBT temperature difference", 0x103C: "Grid disturbance 02",
    0x103D: "L&PE fault", 0x103E: "NTC over temperature (OV-TEM02)", 0x103F: "AC terminal over temperature (OV-TEM03)",
    0x1040: "AFCI self-check fault", 0x1041: "Arc fault (ARC-FAULT)", 0x1046: "Grid interference 02",
    0x1047: "Grid current sampling error (IG-AD)", 0x1048: "Grid level-3 over voltage (OV03-G-V)",
    0x1050: "Grid side instantaneous over current (OV-IgTr)", 0x1051: "DC bus hardware over voltage (OV-BUS-H)",
    0x1052: "LLC hardware over current (OV-ILLC)", 0x1053: "EPM hard limit", 0x1057: "Load overload (Over-Load)",
    0x1058: "DSP self-check fault", 0x1059: "Vg sampling fault", 0x105A: "DSP hardware mismatch",
    0x105D: "DC bus hardware over voltage (storage)",
    0x1800: "DC average over current (OV-DC-I)", 0x1801: "DC input over voltage (OV-DC)",
    0x2010: "Fail Safe", 0x2011: "Fail Safe", 0x2012: "Battery communication fail", 0x2014: "DSP communication fail",
    0x2018: "DRM link fail", 0x201A: "CT fault", 0x201B: "DRM control off grid",
    0x2040: "EPM hard limit protection", 0x2041: "AFCI board comm fail", 0x2042: "AFCI board CT module fail",
    0x2043: "G100 overcurrent protection", 0x2044: "Multiple masters configured", 0x2045: "SPH communication fail",
    0x2046: "Parallel slave comm error", 0x2047: "Panel over current", 0x2048: "Low battery (off-grid)",
    0x204A: "Inverter bridge hard limit", 0x204B: "G100 meter comm fail", 0x204C: "G100 CT fault",
    0x204D: "AFCI board type mismatch", 0x2804: "Battery 1 wiring error", 0x2805: "Battery 2 wiring error",
}
STATUS_SUB = {
    (0x0000, 1): "Grid Off (controlled)", (0x0000, 2): "Waiting01",
    (0x1020, 3): "PV ground fault", (0x1020, 4): "DC boost fault",
    (0x1036, 1): "DSP_B sampling fault",
    (0x101D, 1): "G100 static overvoltage 1", (0x101D, 2): "G100 static overvoltage 2",
    (0x101D, 3): "G100 dynamic overvoltage", (0x101D, 4): "G100 static undervoltage 1",
    (0x101D, 5): "G100 static undervoltage 2", (0x101D, 6): "G100 dynamic undervoltage",
}
GENERATING_LIMITS = {
    0x0001: "LimByTemp", 0x0002: "LimByFreq", 0x0004: "LimByVg", 0x0008: "LimByVar", 0x0010: "LimByUnFr",
    0x0020: "Ramp-up", 0x0040: "Overload", 0x0080: "LimByACTemp", 0x0100: "LimByDRM", 0x0200: "LimByEPM",
}

# Appendix 4
BITS_33121 = {
    0: "Normal operation", 1: "Initial standby", 2: "Shutdown", 3: "Fault", 4: "Standby",
    5: "Derated (temp/freq)", 6: "Limited power (external)", 7: "Bypass overload", 8: "Load fault",
    9: "Grid fault", 10: "Battery fault", 12: "Grid surge", 13: "Fan fault", 14: "External fan fault",
    15: "Fault shutdown",
}
BITS_33339 = {
    0: "Generator start failure", 1: "Generator stop failure", 2: "PV/battery power mismatch",
    3: "Generator voltage/phase abnormal", 4: "SolisHub-DSP comm fault", 5: "AC-coupling locked",
    6: "AC-coupling suspended", 7: "Backup breaker overload", 8: "Grid-tied inverter comm failure",
    9: "Meter 2 abnormal", 10: "Master/slave software mismatch", 11: "Load unloaded (grid draw over limit)",
    12: "Load unload locked",
}
BITS_33123 = {0: "Volt-watt", 1: "Volt-var", 2: "Fixed PF", 3: "Fixed reactive power", 4: "Power-PF", 5: "Power-Q"}
# Appendix 7 (43110)
BITS_43110 = {
    0: "Self-consumption mode", 1: "Optimized revenue V1", 2: "Off-grid mode", 3: "Battery wake-up",
    4: "Backup battery (reserve SOC)", 5: "Grid charging allowed", 6: "Feed-in priority mode",
    7: "Night over-discharge retention", 10: "Battery conditioning", 11: "Peak-shaving mode",
}
BITS_LG = {0: "Fully charged", 1: "Fully discharged", 2: "CB closed", 3: "Power-saving unavailable"}
BITS_25003 = {
    0: "Ready for remote control", 1: "Limited by BMS", 2: "Limited by inverter", 3: "Limited by grid frequency",
    4: "Limited by grid voltage", 5: "Limited by ramp-up", 6: "Limited by DRM", 7: "Limited by EPM",
    8: "Limited by grid peak power", 9: "Limited by reactive power",
}
BITS_ONLINE = {i: f"ID{i}" for i in range(1, 32)}

# Appendix 5 / 6 (43009 low byte = LV battery, high byte = HV battery)
LV_BATTERY = {
    0x00: "No battery", 0x01: "Pylontech LV", 0x03: "BYD B-Box LV", 0x04: "Dyness LV", 0x05: "PureDrive LV",
    0x06: "LG Chem LV", 0x07: "AoBo LV", 0x08: "JiaWei LV", 0x09: "WECO LV", 0x0A: "FreedomWon LV",
    0x0B: "Soluna LV", 0x0C: "GSL LV", 0x0D: "Rita LV", 0x0E: "GS Energy Risen LV", 0x0F: "Alpha LV",
    0x10: "UZ Energy", 0x11: "ATL LV", 0x12: "Zeta LV", 0x13: "Highstar LV", 0x14: "KODAK LV", 0x15: "FOX LV",
    0x16: "EXIDE LV", 0x17: "HD Energy LV", 0x18: "Pytes LV", 0x19: "ARVIO LV", 0x1A: "PAND LV", 0x1B: "WANKE LV",
    0x1C: "Dowell LV", 0x1D: "ROSEN ESS", 0x1E: "ZRGP LV", 0x1F: "Narada", 0x20: "BST LV", 0x21: "Cegasa LV",
    0x22: "Meterboost", 0x23: "MOURA", 0x24: "Tecloman LV", 0x25: "Upower UE-H", 0x26: "Upower UE-I",
    0x27: "SUNWODA LV", 0x28: "CFE LV", 0x29: "DMEGC LV", 0x2A: "JA Solar J-Pack LV", 0x2B: "HY4850",
    0x3E: "Greenrich", 0x3F: "PYTES_LV", 0x54: "JA Solar JA-L", 0x55: "CLENERGY LV", 0x56: "Solis-Alpha LV",
    0x61: "SolisStorage RE-L", 0x62: "Lithium LV (RS485)", 0x63: "Generic lithium LV", 0x64: "Lead acid",
    0x65: "48V lithium", 0x66: "51.2V lithium",
}
HV_BATTERY = {
    0x00: "No battery", 0x01: "Pylontech HV", 0x03: "BYD B-Box HV", 0x04: "LG HV", 0x05: "Soluna/DLG HV",
    0x06: "Dyness HV", 0x07: "Aoboet HV", 0x08: "WECO HV", 0x09: "Alpha HV", 0x0A: "GS Energy",
    0x0B: "BYD HVS/HVM/HVL", 0x0C: "Jinko HV", 0x0D: "FOX HV", 0x0E: "LG 16H HV", 0x0F: "PureDrive HV",
    0x10: "UZ Energy HV", 0x13: "Fortress HV", 0x14: "AMPACE HV", 0x15: "WTS HV (BYD)", 0x16: "J-PACK HV",
    0x17: "SUNWODA HV", 0x26: "LG Enblock S", 0x27: "JA-H HV", 0x28: "BMZ HV", 0x29: "HomeGrid HV",
    0x2A: "Greenrich HV", 0x2B: "BAT HV", 0x2C: "Gotion HV", 0x2D: "Pylon M7 HV", 0x61: "SolisStorage RE-H",
    0x62: "SolisStorage CI", 0x63: "Generic lithium HV",
}

CHARGE_DIR = {0: "Charging", 1: "Discharging"}
ON_OFF_CMD = {0xBE: "On", 0xDE: "Off"}
ENABLED = {0: "Disabled", 1: "Enabled"}
OFF_ON = {0: "Off", 1: "On"}
VALID_AA55 = {0xAA55: "Valid"}
LG_DIAG = {0: "No abnormality", 1: "Warning 1 (power limited)", 2: "Warning 2 (standby)", 3: "Fault"}
LG_MODE = {0: "Initial", 1: "Standby", 3: "Operation", 4: "Update", 5: "Fault", 6: "Self update", 10: "Power saving"}
GEN_STATE = {1: "Stopped", 2: "Supplying power", 3: "Preheating", 4: "Cooling"}
AMBIENT = {1: "Normal temperature", 2: "Low temperature", 3: "High temperature"}
ITALY_FULL_TEST = {1: "Single self-test", 2: "Full self-test"}

# --------------------------------------------------------------------------
# Register map
# --------------------------------------------------------------------------

TYPE_SIZE = {"U16": 1, "S16": 1, "U32": 2, "S32": 2}


@dataclass
class Reg:
    addr: int
    key: str
    name: str
    type: str = "U16"   # U16 S16 U32 S32 HEX ENUM BITS ASCII TIME BATTMODEL BMSREQ DISPATCH
    scale: float = 1
    unit: str = ""
    group: str = ""
    size: int = 0
    table: dict | None = None
    note: str = ""
    fc: int = 4

    def __post_init__(self):
        if not self.size:
            self.size = TYPE_SIZE.get(self.type, 1)


REGS: list[Reg] = []
GROUPS: dict[str, str] = {}


def group(gkey: str, title: str, rows: list[tuple], fc: int = 4):
    GROUPS[gkey] = title
    for r in rows:
        addr, key, name, *rest = r
        kw = rest[0] if rest else {}
        REGS.append(Reg(addr, key, name, group=gkey, fc=fc, **kw))


def U(scale=1, unit="", **kw):   return dict(type="U16", scale=scale, unit=unit, **kw)
def S(scale=1, unit="", **kw):   return dict(type="S16", scale=scale, unit=unit, **kw)
def U32(scale=1, unit="", **kw): return dict(type="U32", scale=scale, unit=unit, **kw)
def S32(scale=1, unit="", **kw): return dict(type="S32", scale=scale, unit=unit, **kw)
def E(table, **kw):  return dict(type="ENUM", table=table, **kw)
def B(table, **kw):  return dict(type="BITS", table=table, **kw)
def H(**kw):         return dict(type="HEX", **kw)


group("info", "Device info", [
    (35000, "inverter_type", "Inverter type (35000)", E(INVERTER_TYPE)),
    (33000, "product_model", "Product model", H()),
    (33004, "serial_number", "Serial number", dict(type="ASCII", size=16)),
    (33001, "dsp_major", "DSP major version", H()),
    (33021, "dsp_sub", "DSP sub version", H()),
    (33002, "hmi_major", "HMI major version", H()),
    (33069, "hmi_sub", "HMI sub version", H()),
    (34367, "hmi_lcd_version", "HMI-LCD firmware", H(note="hi=major, lo=minor")),
    (34240, "afci_version", "AFCI board firmware", H()),
    (33068, "safety_version", "Safety regulation version", U()),
    (33092, "grid_standard", "National standard code", H(note="see Appendix 2 (M33092)")),
    (33067, "rated_apparent_power", "Rated apparent power", U(10, "VA")),
    (33042, "rated_ac_current", "Rated AC grid port current", U(0.1, "A")),
    (33041, "battery_max_current", "Battery max charge/discharge current", U(0.1, "A")),
    (33048, "dc_input_type", "DC input type", U(note="value+1 = number of PV inputs")),
])

group("status", "Status & alarms", [
    (33095, "inverter_status", "Inverter status", H()),
    (33070, "alarm_code", "Alarm sub-code", H()),
    (33121, "operating_status", "Operating status bits", B(BITS_33121)),
    (33339, "inverter_warning", "Inverter warning bits", B(BITS_33339)),
    (33098, "drm_status", "DRM status", U()),
    (33123, "std_mode_status", "Standard mode startup bits", B(BITS_33123)),
    (33093, "inverter_temperature", "Inverter module temperature (IGBT)", S(0.1, "C")),
])

group("pv", "PV", [
    (33057, "pv_power", "Total PV input power", U32(1, "W")),
    *[row for i, base in [(1, 33049), (2, 33051), (3, 33053), (4, 33055), (5, 33059), (6, 33061),
                          (7, 33063), (8, 33065), (9, 34498), (10, 34500)]
      for row in [(base, f"pv{i}_voltage", f"PV{i} voltage", U(0.1, "V")),
                  (base + 1, f"pv{i}_current", f"PV{i} current", U(0.1, "A"))]],
    (33035, "pv_energy_today", "PV energy today", U(0.1, "kWh")),
    (33029, "pv_energy_total", "PV energy total", U32(1, "kWh")),
])

group("battery", "Battery 1", [
    (33139, "battery_soc", "State of charge", U(1, "%")),
    (33140, "battery_soh", "State of health", U(1, "%")),
    (33133, "battery_voltage", "Voltage", U(0.1, "V")),
    (33134, "battery_current", "Current (magnitude)", S(0.1, "A")),
    (33135, "battery_direction", "Direction", E(CHARGE_DIR)),
    (33149, "battery_power", "Power (magnitude)", S32(1, "W")),
    (33043, "battery_temperature", "Battery temperature (BMS)", S(0.1, "C")),
    (33046, "battery_mosfet_temp", "Inverter battery MOSFET temp", S(0.1, "C")),
    (34348, "battery_cell_v_min", "Min cell voltage", U(0.001, "V")),
    (34349, "battery_cell_v_max", "Max cell voltage", U(0.001, "V")),
    (34350, "battery_cell_t_min", "Min cell temperature", S(0.1, "C")),
    (34351, "battery_cell_t_max", "Max cell temperature", S(0.1, "C")),
    (33295, "battery_bms_requests", "BMS requests", dict(type="BMSREQ")),
    (33163, "battery_charge_today", "Charge energy today", U(0.1, "kWh")),
    (33167, "battery_discharge_today", "Discharge energy today", U(0.1, "kWh")),
    (33161, "battery_charge_total", "Charge energy total", U32(1, "kWh")),
    (33165, "battery_discharge_total", "Discharge energy total", U32(1, "kWh")),
])

group("grid", "AC grid port (inverter)", [
    (33151, "grid_port_power", "Grid port total active power", S32(1, "W", note="+ out of inverter, - into inverter")),
    (33079, "active_power", "Inverter active power", S32(1, "W")),
    (33081, "reactive_power", "Inverter reactive power", S32(1, "var")),
    (33083, "apparent_power", "Inverter apparent power", S32(1, "VA")),
    (33073, "ac_voltage_a", "AB line / A phase voltage", U(0.1, "V")),
    (33074, "ac_voltage_b", "BC line / B phase voltage", U(0.1, "V")),
    (33075, "ac_voltage_c", "CA line / C phase voltage", U(0.1, "V")),
    (33076, "ac_current_a", "A phase current", U(0.1, "A")),
    (33077, "ac_current_b", "B phase current", U(0.1, "A")),
    (33078, "ac_current_c", "C phase current", U(0.1, "A")),
    (33094, "grid_frequency", "Grid frequency", U(0.01, "Hz")),
    (33512, "grid_port_p_a", "Phase A active power", S(10, "W")),
    (33513, "grid_port_q_a", "Phase A reactive power", S(10, "var")),
    (33514, "grid_port_s_a", "Phase A apparent power", S(10, "VA")),
    (33515, "grid_port_p_b", "Phase B active power", S(10, "W")),
    (33516, "grid_port_q_b", "Phase B reactive power", S(10, "var")),
    (33517, "grid_port_s_b", "Phase B apparent power", S(10, "VA")),
    (33518, "grid_port_p_c", "Phase C active power", S(10, "W")),
    (33519, "grid_port_q_c", "Phase C reactive power", S(10, "var")),
    (33520, "grid_port_s_c", "Phase C apparent power", S(10, "VA")),
])

group("backup", "Backup port (EPS)", [
    (33148, "backup_power_lo", "Backup load power (low)", U()),
    (34344, "backup_power_hi", "Backup load power (high)", U()),
    (33137, "backup_voltage_a", "Backup voltage A", U(0.1, "V")),
    (33138, "backup_current_a", "Backup current A", U(0.1, "A")),
    (33153, "backup_voltage_b", "Backup voltage B", U(0.1, "V")),
    (33154, "backup_current_b", "Backup current B", U(0.1, "A")),
    (33155, "backup_voltage_c", "Backup voltage C", U(0.1, "V")),
    (33156, "backup_current_c", "Backup current C", U(0.1, "A")),
    (33521, "backup_p_a", "Phase A active power", S(10, "W")),
    (33522, "backup_q_a", "Phase A reactive power", S(10, "var")),
    (33523, "backup_s_a", "Phase A apparent power", S(10, "VA")),
    (33524, "backup_p_b", "Phase B active power", S(10, "W")),
    (33525, "backup_q_b", "Phase B reactive power", S(10, "var")),
    (33526, "backup_s_b", "Phase B apparent power", S(10, "VA")),
    (33527, "backup_p_c", "Phase C active power", S(10, "W")),
    (33528, "backup_q_c", "Phase C reactive power", S(10, "var")),
    (33529, "backup_s_c", "Phase C apparent power", S(10, "VA")),
    (33596, "backup_energy_today", "Backup load energy today", U(0.1, "kWh")),
    (33590, "backup_energy_total", "Backup load energy total", U32(1, "kWh")),
])

group("load", "Household load", [
    (33147, "grid_load_power_lo", "Grid load power (low)", U()),
    (34343, "grid_load_power_hi", "Grid load power (high)", U()),
    (34424, "grid_load_p_a", "Grid load phase A power", U(10, "W")),
    (34425, "grid_load_p_b", "Grid load phase B power", U(10, "W")),
    (34426, "grid_load_p_c", "Grid load phase C power", U(10, "W")),
    (33179, "load_energy_today", "Total load energy today (grid+backup)", U(0.1, "kWh")),
    (33177, "load_energy_total", "Total load energy (grid+backup)", U32(1, "kWh")),
    (33586, "grid_load_energy_today", "Grid load energy today", U(0.1, "kWh")),
    (33580, "grid_load_energy_total", "Grid load energy total", U32(1, "kWh")),
])

group("energy", "Grid energy", [
    (33171, "grid_import_today", "Imported from grid today", U(0.1, "kWh")),
    (33175, "grid_export_today", "Exported to grid today", U(0.1, "kWh")),
    (33169, "grid_import_total", "Imported from grid total", U32(1, "kWh")),
    (33173, "grid_export_total", "Exported to grid total", U32(1, "kWh")),
    (33577, "port_export_today", "Inverter AC port export today", U(0.1, "kWh")),
    (33578, "port_import_today", "Inverter AC port import today", U(0.1, "kWh")),
    (33186, "port_export_total", "Inverter AC port export total", U32(1, "kWh")),
    (33188, "port_import_total", "Inverter AC port import total", U32(1, "kWh")),
])

group("meter", "Meter / CT", [
    (33263, "meter_active_power", "Meter total active power", S32(1, "W", note="+ export, - import")),
    (33257, "meter_active_power_a", "Meter active power A", S32(1, "W")),
    (33259, "meter_active_power_b", "Meter active power B", S32(1, "W")),
    (33261, "meter_active_power_c", "Meter active power C", S32(1, "W")),
    (33251, "meter_voltage_a", "Meter voltage A", U(0.1, "V")),
    (33252, "meter_current_a", "Meter current A", U(0.01, "A")),
    (33253, "meter_voltage_b", "Meter voltage B", U(0.1, "V")),
    (33254, "meter_current_b", "Meter current B", U(0.01, "A")),
    (33255, "meter_voltage_c", "Meter voltage C", U(0.1, "V")),
    (33256, "meter_current_c", "Meter current C", U(0.01, "A")),
    (33271, "meter_reactive_power", "Meter total reactive power", S32(1, "var")),
    (33265, "meter_reactive_power_a", "Meter reactive power A", S32(1, "var")),
    (33267, "meter_reactive_power_b", "Meter reactive power B", S32(1, "var")),
    (33269, "meter_reactive_power_c", "Meter reactive power C", S32(1, "var")),
    (33279, "meter_apparent_power", "Meter total apparent power", S32(1, "VA")),
    (33273, "meter_apparent_power_a", "Meter apparent power A", S32(1, "VA")),
    (33275, "meter_apparent_power_b", "Meter apparent power B", S32(1, "VA")),
    (33277, "meter_apparent_power_c", "Meter apparent power C", S32(1, "VA")),
    (33281, "meter_power_factor", "Meter power factor", S(0.01)),
    (33282, "meter_frequency", "Meter frequency", U(0.01, "Hz")),
    (33283, "meter_import_energy", "Meter energy imported", U32(0.01, "kWh")),
    (33285, "meter_export_energy", "Meter energy exported", U32(0.01, "kWh")),
    (33314, "meter2_active_power", "Meter 2 total active power", S32(1, "W")),
])

group("settings", "Battery & mode settings (FC03)", [
    (43000, "rtc", "Inverter clock", dict(type="TIME", size=6)),
    (43006, "slave_address", "Slave address", U()),
    (43007, "power_on_off", "Power on/off", E(ON_OFF_CMD)),
    (43009, "battery_model", "Battery model", dict(type="BATTMODEL")),
    (43110, "storage_control", "Energy storage control switch", B(BITS_43110)),
    (43010, "max_charge_soc", "Max charging SOC", U(1, "%")),
    (43011, "over_discharge_soc", "Over-discharge SOC", U(1, "%")),
    (43018, "force_charge_soc", "Forced charge SOC", U(1, "%")),
    (43024, "reserve_soc", "Battery reserved SOC (backup)", U(1, "%")),
    (43117, "max_charge_current", "Battery 1 max charge current", U(0.1, "A")),
    (43118, "max_discharge_current", "Battery 1 max discharge current", U(0.1, "A")),
    (43019, "rated_capacity", "Rated capacity (lead-acid / non-comm)", U(1, "Ah")),
    (43020, "over_discharge_voltage", "Over-discharge voltage", U(0.1, "V")),
    (43021, "force_charge_voltage", "Forced charge voltage", U(0.1, "V")),
    (43022, "temp_comp", "Lead-acid temp compensation", U(1, "mV/C")),
    (43023, "ambient_status", "Lead-acid ambient selection", E(AMBIENT)),
    (43111, "backup_enabled", "Backup load function", E(ENABLED)),
    (43112, "backup_voltage_set", "Backup output voltage", U(0.1, "V")),
], fc=3)

group("battery2", "Battery 2 (dual-battery models)", [
    (34278, "bms2_soc", "State of charge", U(1, "%")),
    (34279, "bms2_soh", "State of health", U(1, "%")),
    (34289, "battery2_voltage", "Voltage", U(0.1, "V")),
    (34290, "battery2_current", "Current (magnitude)", S(0.1, "A")),
    (34291, "battery2_direction", "Direction", E(CHARGE_DIR)),
    (34368, "battery2_power", "Power (magnitude)", S32(1, "W")),
    (34277, "bms2_temperature", "Temperature", S(0.1, "C")),
    (34280, "bms2_charge_v_limit", "Charge voltage limit", U(0.1, "V")),
    (34371, "bms2_discharge_v_limit", "Discharge voltage limit", U(0.1, "V")),
    (34281, "bms2_charge_i_limit", "Charge current limit", U(0.1, "A")),
    (34282, "bms2_discharge_i_limit", "Discharge current limit", U(0.1, "A")),
    (34373, "bms2_cell_v_min", "Min cell voltage", U(0.001, "V")),
    (34374, "bms2_cell_v_max", "Max cell voltage", U(0.001, "V")),
    (34375, "bms2_cell_t_min", "Min cell temperature", U(0.1, "C")),
    (34376, "bms2_cell_t_max", "Max cell temperature", U(0.1, "C")),
    (34578, "bms2_requests", "BMS requests", dict(type="BMSREQ")),
])


def _lg_rows(base: int, who: str) -> list[tuple]:
    k = who.lower()
    return [
        (base + 0, f"lg_{k}_soc", f"{who} SOC", U(0.1, "%")),
        (base + 1, f"lg_{k}_soh", f"{who} SOH", U(0.1, "%")),
        (base + 2, f"lg_{k}_voltage", f"{who} voltage", U(0.1, "V")),
        (base + 3, f"lg_{k}_current", f"{who} current (+chg)", S(0.1, "A")),
        (base + 4, f"lg_{k}_max_charge_p", f"{who} max charge power", U(1, "W")),
        (base + 5, f"lg_{k}_max_discharge_p", f"{who} max discharge power", U(1, "W")),
        (base + 6, f"lg_{k}_dcdc_fw_main", f"{who} DCDC firmware (main)", H()),
        (base + 7, f"lg_{k}_dcdc_fw_test", f"{who} DCDC firmware (test)", H()),
        (base + 8, f"lg_{k}_bms_fw_hi", f"{who} BMS firmware high", H()),
        (base + 9, f"lg_{k}_bms_fw_lo", f"{who} BMS firmware low", H()),
        (base + 10, f"lg_{k}_fault_id", f"{who} fault ID", H()),
        (base + 11, f"lg_{k}_diagnostic", f"{who} diagnostic", E(LG_DIAG)),
        (base + 12, f"lg_{k}_mode", f"{who} status/mode", E(LG_MODE)),
        (base + 13, f"lg_{k}_power", f"{who} power (+chg)", S(1, "W")),
        (base + 14, f"lg_{k}_max_temp", f"{who} max temperature", S(0.1, "C")),
        (base + 15, f"lg_{k}_flags", f"{who} status flags", B(BITS_LG)),
    ]


group("lg", "LG RESU 10H/16H battery", [*_lg_rows(34200, "Master"), *_lg_rows(34216, "Secondary")])

group("smart", "Smart port / generator / AC coupling", [
    (34328, "smart_voltage_a", "Smart port voltage A", U(0.1, "V")),
    (34329, "smart_voltage_b", "Smart port voltage B", U(0.1, "V")),
    (34330, "smart_voltage_c", "Smart port voltage C", U(0.1, "V")),
    (34331, "smart_current_a", "Smart port current A", U(0.1, "A")),
    (34332, "smart_current_b", "Smart port current B", U(0.1, "A")),
    (34333, "smart_current_c", "Smart port current C", U(0.1, "A")),
    (34391, "smart_power_a", "Smart port active power A", S(10, "W")),
    (34392, "smart_power_b", "Smart port active power B", S(10, "W")),
    (34393, "smart_power_c", "Smart port active power C", S(10, "W")),
    (34411, "smart_load_energy_total", "Smart load energy total", U32(1, "kWh")),
    (34413, "smart_load_energy_today", "Smart load energy today", U32(1, "kWh")),
    (33530, "gen_power_a", "Generator active power A", S(10, "W")),
    (33534, "gen_power_b", "Generator active power B", S(10, "W")),
    (33535, "gen_power_c", "Generator active power C", S(10, "W")),
    (34410, "gen_frequency", "Generator frequency", U(0.01, "Hz")),
    (34590, "gen_state", "Generator state", E(GEN_STATE)),
    (33531, "gen_energy_today", "Generator energy today", U(0.1, "kWh")),
    (33532, "gen_energy_total", "Generator energy total", U32(1, "kWh")),
    (34451, "ac_couple_energy_today", "AC coupling energy today", U(0.1, "kWh")),
    (34445, "ac_couple_energy_total", "AC coupling energy total", U32(1, "kWh")),
    (33245, "parallel_inverter_power", "AC-coupled PV inverter power", U(0.1, "kW")),
    (33246, "parallel_inverter_ct", "AC-coupled PV inverter CT switch", E(OFF_ON)),
])

group("ports", "Port power summary (module 1)", [
    (34600, "module1_id", "Module ID", H()),
    (34601, "module1_count", "Register count", U()),
    (34602, "module1_valid", "Valid flag", E(VALID_AA55)),
    (34603, "port_pv_power", "PV power", S32(1, "W")),
    (34605, "port_battery1_power", "Battery 1 power (+chg)", S32(1, "W")),
    (34607, "port_battery2_power", "Battery 2 power (+chg)", S32(1, "W")),
    (34609, "port_inverter_power", "Inverting bridge active power", S32(1, "W")),
    (34611, "port_backup_power", "Backup load active power", S32(1, "W")),
    (34613, "port_backup_apparent", "Backup load apparent power", S32(1, "VA")),
    (34615, "port_grid_port_power", "AC grid port active power", S32(1, "W")),
    (34617, "port_generator_power", "Generator active power", S32(1, "W")),
    (34619, "port_smart_load_power", "Smart load active power", S32(1, "W")),
    (34621, "port_gridtie_inv_power", "Grid-tied inverter output", S32(1, "W")),
    (34623, "port_grid_load_power", "Grid load active power", S32(1, "W")),
    (34625, "port_public_grid_power", "Public grid active power (+export)", S32(1, "W")),
])

group("parallel", "Parallel system", [
    (33388, "parallel_online", "Inverters online", B(BITS_ONLINE)),
    (34427, "parallel_valid", "Parallel data valid", E(VALID_AA55)),
    (34428, "par_pv_power", "Total PV power", S(100, "W")),
    (34429, "par_gen_power", "Total generator power", S(100, "W")),
    (34430, "par_battery_power", "Total battery power (+chg)", S(100, "W")),
    (34431, "par_backup_power", "Total backup load power", S(100, "W")),
    (34432, "par_inverter_power", "Total inverter power (+export)", S(100, "W")),
    (34433, "par_smart_power", "Total smart load power", S(100, "W")),
    (34434, "par_ac_couple_power", "Total AC couple power", S(100, "W")),
    (34490, "par_grid_load_power", "Total grid load power", S(100, "W")),
    (34491, "par_public_grid_power", "Public grid power (+export)", S(100, "W")),
    (34537, "par_grid_port_power", "Total AC grid port power", S(100, "W")),
    (34779, "par_pv_power_w", "PV power", S32(1, "W")),
    (34781, "par_battery_power_w", "Battery power (+chg)", S32(1, "W")),
    (34785, "par_grid_port_power_w", "AC grid port power", S32(1, "W")),
    (34795, "par_pcc_power", "PCC active power", S32(1, "W")),
    (34797, "par_max_soc", "Max battery SOC", U(1, "%")),
    (34798, "par_min_soc", "Min battery SOC", U(1, "%")),
])

group("system", "PV + storage system (module 6, C&I)", [
    (34900, "module6_id", "Module ID", H()),
    (34901, "module6_count", "Register count", U()),
    (34902, "module6_valid", "Valid flag", E(VALID_AA55)),
    (34903, "sys_pv_power", "Hybrid inverters PV power", S32(1, "W")),
    (34905, "sys_battery_power", "Hybrid inverters battery power (+chg)", S32(1, "W")),
    (34907, "sys_inverter_power", "Hybrid inverters active power", S32(1, "W")),
    (34909, "sys_backup_power", "System backup load power", S32(1, "W")),
    (34911, "sys_grid_port_power", "Hybrid inverters AC grid port power", S32(1, "W")),
    (34913, "sys_generator_power", "Generator power", S32(1, "W")),
    (34915, "sys_smart_load_power", "Smart load power", S32(1, "W")),
    (34917, "sys_gridtie_power", "Grid-tied inverters power", S32(1, "W")),
    (34919, "sys_grid_load_power", "System grid load power", S32(1, "W")),
    (34921, "sys_public_grid_power", "Public grid power (+export)", S32(1, "W")),
    (34923, "sys_gridtie_online", "Grid-tied inverters online", U()),
])

group("remote", "Remote dispatch / FCAS", [
    (33289, "optimal_revenue_v2", "Optimal revenue V2", E(VALID_AA55)),
    (34502, "dispatch_valid", "Remote dispatch available", E(VALID_AA55)),
    (34503, "dispatch_version", "Remote dispatch version", H()),
    (34504, "dispatch_status", "Remote dispatch status", dict(type="DISPATCH")),
    (34799, "remote_protocol_version", "Remote control protocol version", H()),
    (34753, "local_pv_power", "Local PV power", S32(1, "W")),
    (34755, "local_battery_power", "Local battery power (+chg)", S32(1, "W")),
    (34765, "local_grid_port_power", "Local AC grid port power", S32(1, "W")),
    (25000, "module7_id", "Module ID", H()),
    (25001, "module7_count", "Register count", U()),
    (25002, "module7_valid", "Valid flag", E(VALID_AA55)),
    (25003, "remote_status", "Remote control status", B(BITS_25003)),
    (25004, "avail_charge_p", "Battery available charge power", U32(1, "W")),
    (25006, "avail_discharge_p", "Battery available discharge power", U32(1, "W")),
    (25008, "avail_import_p", "AC port available import power", U32(1, "W")),
    (25010, "avail_export_p", "AC port available export power", U32(1, "W")),
    (25012, "par_avail_charge_p", "Parallel available charge power", U32(1, "W")),
    (25014, "par_avail_discharge_p", "Parallel available discharge power", U32(1, "W")),
    (25016, "par_avail_import_p", "Parallel available import power", U32(1, "W")),
    (25018, "par_avail_export_p", "Parallel available export power", U32(1, "W")),
    (25020, "par_online_ids", "Parallel inverters online", dict(type="U32")),
])

_ITALY = [("59.S1", "V"), ("59.S2", "V"), ("27.S1", "V"), ("27.S2", "V"),
          ("81>.S1", "F"), ("81<.S1", "F"), ("81>.S2F", "F"), ("81<.S2F", "F"),
          ("81>.S2S", "F"), ("81<.S2S", "F")]


def _italy_rows(base: int, phase: str) -> list[tuple]:
    rows = []
    for i, (code, kind) in enumerate(_ITALY):
        slug = code.replace(">", "gt").replace("<", "lt").replace(".", "_").lower()
        val = U(0.1, "V") if kind == "V" else U(0.01, "Hz")
        label = "voltage" if kind == "V" else "frequency"
        sfx = f"_{phase.lower()}" if phase else ""
        ph = f" ({phase} phase)" if phase else ""
        rows.append((base + 2 * i, f"it_{slug}_{label}{sfx}", f"{code} {label}{ph}", val))
        rows.append((base + 2 * i + 1, f"it_{slug}_time{sfx}", f"{code} time{ph}", U(1, "ms")))
    return rows


group("italy", "Italy CEI 0-21 self-test", [
    (33221, "italy_single_selftest", "Single self-test item", U()),
    (33222, "italy_full_selftest", "Full self-test", E(ITALY_FULL_TEST)),
    *_italy_rows(33223, ""),
    *_italy_rows(33470, "B"),
    *_italy_rows(33490, "C"),
])

DEFAULT_GROUPS = ["info", "status", "pv", "battery", "grid", "backup", "load", "energy", "meter", "settings"]
ALL_GROUPS = list(GROUPS)

_keys = [r.key for r in REGS]
assert len(_keys) == len(set(_keys)), "duplicate register keys"


# --------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------

@dataclass
class Span:
    fc: int
    start: int
    count: int
    regs: list[Reg] = field(default_factory=list)


def build_spans(regs: list[Reg], max_gap: int = 10) -> list[Span]:
    spans: list[Span] = []
    for r in sorted(regs, key=lambda x: (x.fc, x.addr)):
        end = r.addr + r.size
        cur = spans[-1] if spans else None
        if (cur and cur.fc == r.fc
                and r.addr - (cur.start + cur.count) <= max_gap
                and end - cur.start <= MAX_REGS_PER_READ):
            cur.count = max(cur.count, end - cur.start)
            cur.regs.append(r)
        else:
            spans.append(Span(r.fc, r.addr, r.size, [r]))
    return spans


def read_regs(client: RtuClient, regs: list[Reg], log: Callable[[str], None]) -> tuple[dict[int, list[int]], dict[int, str]]:
    """Returns ({id(reg): [words]}, {id(reg): error})."""
    raw: dict[int, list[int]] = {}
    errors: dict[int, str] = {}
    state = {"ok": False, "timeouts": 0}

    def do_span(span: Span) -> None:
        if not state["ok"] and state["timeouts"] >= NO_REPLY_ABORT:
            for r in span.regs:
                errors[id(r)] = "skipped (inverter not responding)"
            return
        try:
            words = client.read(span.fc, span.start, span.count)
        except ModbusException as e:
            if len(span.regs) > 1:
                mid = len(span.regs) // 2
                for half in (span.regs[:mid], span.regs[mid:]):
                    for sub in build_spans(half):
                        do_span(sub)
                return
            for r in span.regs:
                errors[id(r)] = str(e)
            return
        except ModbusError as e:
            if isinstance(e, ModbusTimeout):
                state["timeouts"] += 1
            log(f"  ! FC{span.fc:02d} {span.start}-{span.start + span.count - 1}: {e}")
            for r in span.regs:
                errors[id(r)] = str(e)
            return
        state["ok"] = True
        for r in span.regs:
            off = r.addr - span.start
            raw[id(r)] = words[off:off + r.size]

    for span in build_spans(regs):
        do_span(span)
    if not state["ok"] and state["timeouts"] >= NO_REPLY_ABORT:
        raise NoResponse("inverter did not reply")
    return raw, errors


# --------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------

def _decimals(scale: float) -> int:
    if scale >= 1:
        return 0
    return max(0, len(f"{scale:.6f}".rstrip("0").split(".")[1]))


def _bits(v: int, table: dict, width: int = 16) -> list[str]:
    return [table.get(b, f"bit{b}") for b in range(width) if v >> b & 1]


def _s16(w: int) -> int:
    return w - 0x10000 if w & 0x8000 else w


def _bms_req(v: int) -> str:
    names = {0: "invalid", 1: "no request", 2: "REQUEST", 3: "invalid"}
    return f"force charge: {names[v & 3]}, heating: {names[v >> 2 & 3]}"


def decode(r: Reg, words: list[int]) -> tuple[object, str]:
    """Returns (value, human text)."""
    t = r.type
    w = words[0]
    if t in ("U16", "S16", "U32", "S32"):
        if t == "U16":
            v = w
        elif t == "S16":
            v = _s16(w)
        else:
            v = (words[0] << 16) | words[1]
            if t == "S32":
                if v == 0x80000000:
                    return None, "invalid / not present"
                if v & 0x80000000:
                    v -= 0x100000000
        val = round(v * r.scale, _decimals(r.scale)) if r.scale != 1 else v
        return val, f"{val} {r.unit}".strip()
    if t == "HEX":
        return w, f"0x{w:04X}"
    if t == "ENUM":
        return w, r.table.get(w, f"unknown (0x{w:04X})")
    if t == "BITS":
        active = _bits(w, r.table)
        return w, (", ".join(active) if active else "none") + f"  [0x{w:04X}]"
    if t == "ASCII":
        b = b"".join(struct.pack(">H", x) for x in words)
        s = b.replace(b"\x00", b"").decode("ascii", "replace").strip()
        return s, s
    if t == "TIME":
        y, mo, d, h, mi, s = words
        ts = f"20{y:02d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"
        return ts, ts
    if t == "BATTMODEL":
        lv, hv = w & 0xFF, w >> 8
        parts = []
        if hv:
            parts.append(f"HV: {HV_BATTERY.get(hv, f'code 0x{hv:02X}')}")
        if lv or not hv:
            parts.append(f"LV: {LV_BATTERY.get(lv, f'code 0x{lv:02X}')}")
        return w, ", ".join(parts) + f"  [0x{w:04X}]"
    if t == "BMSREQ":
        return w, _bms_req(w)
    if t == "DISPATCH":
        st = {0: "Not running", 1: "Default control (standby)", 2: "Real-time control", 3: "Timing control"}
        md = {0: "-", 1: "Battery standby", 2: "Battery charge/discharge", 3: "Grid point import/export",
              4: "AC port import/export", 5: "Self-use priority", 6: "Feed-in priority"}
        return w, f"{st.get(w & 0xFF, hex(w & 0xFF))}, mode: {md.get(w >> 8, hex(w >> 8))}"
    return w, str(w)


def status_text(status: int, sub: int) -> str:
    base = STATUS.get(status, f"Unknown status 0x{status:04X}")
    if status == 0x0003 and sub:
        flags = [n for m, n in GENERATING_LIMITS.items() if sub & m]
        return f"{base} - derating: {', '.join(flags) or f'0x{sub:04X}'}"
    if (status, sub) in STATUS_SUB:
        return STATUS_SUB[(status, sub)]
    if status in (0x1800, 0x1801) and sub:
        return f"{base} - DC input {sub}"
    if sub:
        return f"{base} (sub-code {sub:02d})"
    return base


@dataclass
class Result:
    reg: Reg
    raw: list[int] | None
    value: object = None
    text: str = ""
    error: str = ""


def _calc(results: dict[str, Result], key: str, name: str, group_: str, value, text: str, unit: str = "") -> None:
    results[key] = Result(Reg(0, key, name, unit=unit, group=group_), [], value, text)


def _ok(results: dict[str, Result], *keys: str) -> bool:
    return all(k in results and results[k].raw for k in keys)


def poll(client: RtuClient, groups: list[str], log: Callable[[str], None]) -> dict[str, Result]:
    results: dict[str, Result] = {}
    regs = [r for r in REGS if r.group in groups]
    raw, errors = read_regs(client, regs, log)
    for r in regs:
        if id(r) in raw:
            v, txt = decode(r, raw[id(r)])
            results[r.key] = Result(r, raw[id(r)], v, txt)
        else:
            results[r.key] = Result(r, None, error=errors.get(id(r), "not read"))

    # ---- derived values ----
    if _ok(results, "inverter_status"):
        st = results["inverter_status"]
        sub = results["alarm_code"].raw[0] if _ok(results, "alarm_code") else 0
        if st.raw[0] not in STATUS and _ok(results, "operating_status") and results["operating_status"].raw[0]:
            # Code not listed in Appendix 3: describe it with the 33121 operating status bits instead
            bits = ", ".join(_bits(results["operating_status"].raw[0], BITS_33121))
            st.text = f"{bits} (status code not in protocol)  [0x{st.raw[0]:04X}]"
        else:
            st.text = f"{status_text(st.raw[0], sub)}  [0x{st.raw[0]:04X}]"

    if _ok(results, "dsp_major", "dsp_sub"):
        v = f"{results['dsp_major'].raw[0] & 0xFF:02X}{results['dsp_sub'].raw[0] & 0xFF:02X}"
        _calc(results, "dsp_version", "DSP firmware version", "info", v, v)
    if _ok(results, "hmi_major", "hmi_sub"):
        v = f"{results['hmi_major'].raw[0] & 0xFF:02X}{results['hmi_sub'].raw[0] & 0xFF:02X}"
        _calc(results, "hmi_version", "HMI firmware version", "info", v, v)
    n_pv = 10
    if _ok(results, "dc_input_type"):
        d = results["dc_input_type"]
        d.value = d.raw[0] + 1
        d.text = f"{d.value} PV input(s)"
        if 1 <= d.value <= 10:
            n_pv = d.value
            for i in range(n_pv + 1, 11):
                results.pop(f"pv{i}_voltage", None)
                results.pop(f"pv{i}_current", None)

    for i in range(1, n_pv + 1):
        if _ok(results, f"pv{i}_voltage", f"pv{i}_current"):
            v, c = results[f"pv{i}_voltage"].value, results[f"pv{i}_current"].value
            p = round(v * c, 1)
            _calc(results, f"pv{i}_power", f"PV{i} power (V x I)", "pv", p, f"{p} W", "W")

    # Battery: magnitude + direction register -> signed (+ charge, - discharge)
    for n, (cur, pw, dr) in {"1": ("battery_current", "battery_power", "battery_direction"),
                             "2": ("battery2_current", "battery2_power", "battery2_direction")}.items():
        grp = "battery" if n == "1" else "battery2"
        if _ok(results, pw, dr) and results[pw].value is not None:
            sign = -1 if results[dr].raw[0] == 1 else 1
            p = sign * abs(results[pw].value)
            state = "idle" if p == 0 else ("charging" if p > 0 else "discharging")
            _calc(results, f"battery{n}_power_signed", "Power (+charge / -discharge)", grp, p, f"{p} W ({state})", "W")
        if _ok(results, cur, dr):
            sign = -1 if results[dr].raw[0] == 1 else 1
            c = round(sign * abs(results[cur].value), 1)
            _calc(results, f"battery{n}_current_signed", "Current (+charge / -discharge)", grp, c, f"{c} A", "A")

    # Grid load power: 33147 low + 34343 high (U32)
    if _ok(results, "grid_load_power_lo"):
        hi = results["grid_load_power_hi"].raw[0] if _ok(results, "grid_load_power_hi") else 0
        p = (hi << 16) | results["grid_load_power_lo"].raw[0]
        _calc(results, "grid_load_power", "Grid load power (excl. backup)", "load", p, f"{p} W", "W")
    for k in ("grid_load_power_lo", "grid_load_power_hi"):
        results.pop(k, None)

    # Backup load power: 33148 alone (S16) up to 20 kW, combined with 34344 (S32) above
    if _ok(results, "backup_power_lo"):
        lo = results["backup_power_lo"].raw[0]
        hi = results["backup_power_hi"].raw[0] if _ok(results, "backup_power_hi") else 0
        rated = results["rated_apparent_power"].value if _ok(results, "rated_apparent_power") else 0
        if rated > 20000 or hi not in (0, 0xFFFF):
            v = (hi << 16) | lo
            p = v - 0x100000000 if v & 0x80000000 else v
        else:
            p = _s16(lo)
        _calc(results, "backup_power", "Backup load power (+out of port)", "backup", p, f"{p} W", "W")
    for k in ("backup_power_lo", "backup_power_hi"):
        results.pop(k, None)

    return results


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------

def print_table(results: dict[str, Result], show_raw: bool, show_errors: bool) -> None:
    print(f"\n=== Solis hybrid inverter  {datetime.now():%Y-%m-%d %H:%M:%S} ===")
    for gkey, title in GROUPS.items():
        rows = [r for r in results.values() if r.reg.group == gkey and (r.raw is not None or show_errors)]
        if not rows:
            continue
        print(f"\n[{title}]")
        for res in rows:
            addr = f"{res.reg.addr:>5}" if res.reg.addr else " calc"
            if res.raw is None:
                text = f"-- {res.error}"
            else:
                text = res.text
                if show_raw and res.raw:
                    text += "   raw=" + " ".join(f"{x:04X}" for x in res.raw)
            print(f"  {addr}  {res.reg.name:<40} {text}")


def to_dict(results: dict[str, Result]) -> dict:
    out: dict = {"timestamp": datetime.now().isoformat(timespec="seconds")}
    for gkey in GROUPS:
        g = {k: {"value": r.value, "unit": r.reg.unit, "text": r.text, "register": r.reg.addr or None}
             for k, r in results.items() if r.reg.group == gkey and r.raw is not None}
        if g:
            out[gkey] = g
    return out


def append_csv(path: str, results: dict[str, Result]) -> None:
    row = {"timestamp": datetime.now().isoformat(timespec="seconds")}
    for k, r in results.items():
        if r.raw is not None:
            row[k] = r.value
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    if not new:
        with open(path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f), [])
        # keep existing column order, append any new columns at the end
        cols = header + [k for k in row if k not in header]
        if cols != header:
            with open(path, newline="", encoding="utf-8") as f:
                old = list(csv.DictReader(f))
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=cols)
                w.writeheader()
                w.writerows(old)
    else:
        cols = list(row)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(row)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

NO_REPLY_HINT = """No reply from the inverter. Check:
  - RS485 A/B wiring (try swapping A and B), and GND if available
  - slave address (--unit) and baud rate (--baud) match the inverter settings
  - no datalogger stick is polling the same RS485 port
  - Raspberry Pi: serial console disabled, and the RS485 board handles DE/RE direction"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Read operating data from a Solis hybrid inverter over Modbus RTU (RS485).")
    ap.add_argument("--port", help="serial port, e.g. COM5, /dev/ttyUSB0 or /dev/serial0")
    ap.add_argument("--unit", type=int, default=1, help="inverter slave address (default 1)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--timeout", type=float, default=1.0, help="response timeout in seconds")
    ap.add_argument("--retries", type=int, default=1, help="retries on timeout/CRC error")
    ap.add_argument("--groups", default=",".join(DEFAULT_GROUPS),
                    help=f"comma separated groups, or 'all'. Default: {','.join(DEFAULT_GROUPS)}. "
                         f"Available: {','.join(ALL_GROUPS)}")
    ap.add_argument("--interval", type=float, default=0, help="poll repeatedly every N seconds")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    ap.add_argument("--csv", metavar="FILE", help="append each poll as a row to a CSV file")
    ap.add_argument("--raw", action="store_true", help="show raw register words")
    ap.add_argument("--show-unsupported", action="store_true", help="also list registers that could not be read")
    ap.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    args = ap.parse_args(argv)

    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:<14} {p.description}")
        return 0
    if not args.port:
        ap.error("--port is required (use --list-ports to find it)")

    groups = ALL_GROUPS if args.groups.strip() == "all" else [g.strip() for g in args.groups.split(",") if g.strip()]
    bad = [g for g in groups if g not in GROUPS]
    if bad:
        ap.error(f"unknown group(s): {', '.join(bad)}")

    log = (lambda m: print(m, file=sys.stderr))
    client = RtuClient(args.port, args.unit, args.baud, args.timeout, args.retries)
    rc = 0
    try:
        while True:
            t0 = time.monotonic()
            try:
                results = poll(client, groups, log)
            except NoResponse:
                log(NO_REPLY_HINT)
                rc = 1
                results = None
            if results:
                rc = 0
                if args.json:
                    print(json.dumps(to_dict(results), indent=2, ensure_ascii=False))
                else:
                    print_table(results, args.raw, args.show_unsupported)
                if args.csv:
                    append_csv(args.csv, results)
            if args.interval <= 0:
                break
            time.sleep(max(0.0, args.interval - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
