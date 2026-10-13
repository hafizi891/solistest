#!/usr/bin/env python3
"""
Solis PV grid-connected inverter reader - Modbus RTU over RS485.

Implements the read side (function code 0x04, input registers) of
"RS485_MODBUS Solis PV Grid-Connected Inverter" protocol V19.

Protocol notes (from the PDF):
  * 9600 bps, 8N1, slave address 1..247
  * >= 300 ms between frames, max ~50 registers per request
  * Registers in section 5.2 (3000-3999) are sent with address - 1
    (e.g. register 3000 is requested as 0x0BB7 = 2999)
  * Register 35000 (section 5.1) is sent as-is, no offset
  * U32/S32: high word first

Only dependency: pyserial  (pip install pyserial)

Examples:
  python solis_modbus.py --list-ports
  python solis_modbus.py --port COM5
  python solis_modbus.py --port COM5 --unit 1 --groups info,energy,ac,dc,status
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

EXCEPTION_TEXT = {
    0x01: "illegal function code",
    0x02: "illegal data address",
    0x03: "illegal data value",
    0x04: "service failure",
    0x05: "HMI/DSP communication failure",
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

    def _transact(self, fc: int, wire_addr: int, count: int) -> list[int]:
        req = with_crc(struct.pack(">BBHH", self.unit, fc, wire_addr, count))
        self._wait_gap()
        self.ser.reset_input_buffer()
        self.ser.write(req)
        self.ser.flush()
        try:
            head = self.ser.read(3)
            if len(head) < 3:
                raise ModbusTimeout(f"no response (got {len(head)} bytes)")
            addr, rfc, n = head
            if rfc == fc | 0x80:
                tail = self.ser.read(2)
                frame = head + tail
                if len(tail) < 2 or crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                    raise ModbusError("bad CRC on exception response")
                raise ModbusException(n)
            if addr != self.unit or rfc != fc:
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

    def read_input(self, wire_addr: int, count: int) -> list[int]:
        last: Exception | None = None
        for _ in range(self.retries + 1):
            try:
                return self._transact(0x04, wire_addr, count)
            except ModbusException:
                raise  # deterministic answer, retrying won't help
            except ModbusError as e:
                last = e
        raise last  # type: ignore[misc]


# --------------------------------------------------------------------------
# Lookup tables (appendices)
# --------------------------------------------------------------------------

INVERTER_TYPE = {
    0x1110: "Single phase inverter (0.7-8K1P / 7-10K1P / no-screen)",
    0x1111: "Single phase microinverter (0.6-0.8K1P)",
    0x1120: "Three phase inverter (5-25K3P / 5-30K3P(K2) no-screen)",
    0x1121: "Three phase inverter (25-136K / 3-30K3P(K3) no-screen)",
    0x1123: "Three phase inverter (MAX/PRO)",
    0x1124: "Three phase inverter (320K)",
}

AC_OUTPUT_TYPE = {0: "Single phase", 1: "Three-phase four-wire", 2: "Phase three-wire", 3: "Three-phase three-wire"}

WORKING_MODE = {
    0: "No response mode", 1: "Volt-watt default", 2: "Volt-var", 3: "Fixed power factor",
    4: "Fixed reactive power", 5: "Power-PF", 6: "Rule21 Volt-watt (US)",
}

POWER_LIMIT_SWITCH = {0xAA: "Enabled", 0x55: "Disabled (100%)"}
REACTIVE_SWITCH = {0x55: "Off (PF=1, Q=0)", 0xA1: "Reactive power ratio active", 0xA2: "Power factor setting active"}
INSTALL_METHOD = {0: "90 deg", 1: "15 deg"}
EPM_HARD_SWITCH = {0: "Null", 1: "ON", 2: "OFF"}
CT_LINK = {0: "Connected", 1: "Connection failed", 2: "Cannot judge"}
G100_STATE = {1: "Normal", 2: "Overrun", 3: "Fault", 4: "Off"}
ITALY_FULL_TEST = {1: "Single self-test", 2: "Full self-test"}
VRT_FLAG = {0xA5: "VRT disabled", 0x5A: "VRT enabled"}
AFCI_BOARD = {0: "Old AFCI", 1: "Lingshi detection board", 2: "TI detection board"}
STD_SELECT = {0: "Invalid", 1: "Waiting for selection", 2: "Setting standard", 3: "Standard set"}
IGBT_SELFCHECK = {0: "Not triggered", 1: "Triggering", 2: "Trigger success", 4: "Self-test ended"}
RESET_ENABLE = {0x00: "Disabled", 0x59: "Enabled"}
RESET_CMD = {0x00: "Disabled", 0x45: "Enabled"}
METER_485 = {0: "Meter communication", 1: "Dual 485"}
PV_COMBO = {0x0000: "1 voltage : 2 currents", 0x0003: "1 voltage : 3 currents"}
PERF_NORMAL = {1: "CAT_A", 2: "CAT_B"}
PERF_ABNORMAL = {1: "CAT_I", 2: "CAT_II", 3: "CAT_III"}
MANUFACTURER = {0: "Null", 1: "Ginlong"}
IV_TYPE = {0: "Undefined", 1: "60-point IV curve", 2: "128-point IV curve"}
IV_SCAN = {
    0x00: "Not configured", 0x10: "Request processing", 0x11: "Request failed", 0x12: "Request succeeded",
    0x20: "Turning on IV scan", 0x21: "Unable to start IV scan", 0x22: "IV scan on",
    0x30: "Scanning", 0x31: "Scan failed", 0x32: "Scan successful",
    0x40: "HMI getting DSP data", 0x41: "HMI failed to get DSP data", 0x42: "HMI got DSP data",
}
OFF_ON = {0: "Off", 1: "On"}
NO_YES = {0: "No", 1: "Yes"}

# Appendix 1 - inverter status (3044); 3030 holds the sub-code
STATUS = {
    0x0000: "Waiting", 0x0001: "OpenRun", 0x0002: "SoftRun", 0x0003: "Generating",
    0x1004: "Grid Off",
    0x1010: "Grid over voltage (OV-G-V)", 0x1011: "Grid under voltage (UN-G-V)",
    0x1012: "Grid over frequency (OV-G-F)", 0x1013: "Grid under frequency (UN-G-F)",
    0x1014: "Grid reverse (Backfeed_Iac)", 0x1015: "No grid (NO-Grid)", 0x1016: "Grid unbalance (G-PHASE)",
    0x1017: "Grid frequency fluctuation (G-F-FLU)", 0x1018: "Grid over current (OV-G-I)",
    0x1019: "Grid current tracking fault (IGFOL-F)", 0x101A: "Grid phase abnormal",
    0x101C: "IGBT shift fault", 0x101D: "G100 voltage protection",
    0x1020: "DC over voltage (OV-DC)", 0x1021: "DC bus over voltage (OV-BUS)",
    0x1022: "DC bus unbalance (UNB-BUS)", 0x1023: "DC bus under voltage (UN-BUS)",
    0x1024: "DC bus unbalance 2 (UNB2-BUS)", 0x1025: "DC channel A over current (OV-DCA-I)",
    0x1026: "DC channel B over current (OV-DCB-I)", 0x1027: "DC interference (DC-INTF)",
    0x1028: "DC reverse (Reve-DC)", 0x1029: "PV mid-point grounding (PvMidIso)",
    0x102A: "Master/slave bus voltage inconsistent (Vbus-Sam)",
    0x1030: "Grid interference protection (GRID-INTF)", 0x1031: "DSP initial protection (INI-FAULT)",
    0x1032: "Temperature protection (OV-TEM01)", 0x1033: "PV insulation fault (PV ISO-PRO)",
    0x1034: "Leakage current protection (ILeak-PRO)", 0x1035: "Relay protection (RelayChk-FAIL)",
    0x1036: "DSP_B communication fault", 0x1037: "DC injection protection (DCInj-FAULT)",
    0x1038: "12V under voltage (12Power-FAULT)", 0x1039: "Leakage current check (ILeak-Check)",
    0x103A: "Under temperature protection (UN-TEM)", 0x103D: "L&PE fault",
    0x103E: "NTC over temperature (OV-TEM02)",
    0x1040: "AFCI check fault", 0x1041: "AFCI arc fault (ARC-FAULT)", 0x1046: "Grid interference 02",
    0x1047: "Grid current sampling error (IG-AD)", 0x1048: "IGBT over current (IGBT-OV-I)",
    0x1050: "Grid side instantaneous over current (OV-IgTr)", 0x1051: "DC bus hardware over voltage (OV-BUS-H)",
    0x1052: "LLC hardware over current (OV-ILLC)", 0x1053: "EPM hard limit / battery over voltage",
    0x1054: "Battery under voltage (UN-Vbatt)", 0x1055: "Battery not connected",
    0x1056: "Bypass over voltage (OV-VBackup)", 0x1057: "Bypass overload (Over-Load)",
    0x1058: "DSP self-check exception", 0x1059: "Vg sampling exception", 0x105A: "DSP hardware mismatch",
    0x105B: "Battery over current (BAT-DOC)",
    0x2010: "Fail Safe", 0x2011: "Fail Safe", 0x2012: "Battery communication failure",
    0x2014: "DSP communication failure", 0x2015: "BMS alarm 1", 0x2016: "Inconsistent battery selection",
    0x2017: "BMS alarm 2", 0x2018: "DRM connection failed", 0x2019: "Abnormal meter selection",
    0x201A: "CT connect error", 0x201B: "DRM control off grid",
    0x2020: "Lead-acid high ambient temperature", 0x2021: "Lead-acid low ambient temperature",
    0x2030: "Backup overload (BKAC Overload)", 0x2040: "EPM hard limit protection",
    0x2041: "AFCI board communication fail", 0x2042: "AFCI board CT module fail",
    0x2043: "G100 overcurrent protection",
    0xF010: "Grid surge (warning)", 0xF011: "Fan fault (warning)", 0xF013: "AC SPD error (warning)",
    0xF014: "DC SPD error (warning)", 0xF015: "External fan fault (warning)", 0xF016: "Fuse alarm (warning)",
    0xF017: "AC phase ground fault (warning)", 0xF018: "DSP communication error (warning)",
    0xF019: "Boost error", 0xF01C: "IGBT temperature difference",
}
STATUS_SUB = {
    (0x0000, 1): "Grid Off (controlled)",
    (0x1020, 3): "PV ground fault", (0x1020, 4): "DC boost fault",
    (0x1036, 1): "DSP_B sampling fault",
    (0x101D, 1): "Static overvoltage 1", (0x101D, 2): "Static overvoltage 2",
    (0x101D, 3): "Dynamic overvoltage", (0x101D, 4): "Static undervoltage 1",
    (0x101D, 5): "Static undervoltage 2", (0x101D, 6): "Dynamic undervoltage",
}
GENERATING_LIMITS = {
    0x0001: "LimByTemp", 0x0002: "LimByFreq", 0x0004: "LimByVg", 0x0008: "LimByVar",
    0x0010: "LimByUnFr", 0x0020: "Ramp-up", 0x0040: "Overload", 0x0100: "LimByDRM", 0x0200: "LimByEPM",
}

# Appendix 2 - bit definitions
BITS_3095 = {0: "LimByTemp", 1: "LimByFreq", 2: "LimByVg", 3: "LimByVar", 4: "LimByUnFr",
             5: "Ramp-up", 6: "Overload", 8: "LimByDRM", 9: "LimByEPM"}
BITS_FAULT1 = {
    0: "Grid AB overvoltage", 1: "Grid BC overvoltage", 2: "Grid CA overvoltage",
    3: "Grid AB undervoltage", 4: "Grid BC undervoltage", 5: "Grid CA undervoltage",
    6: "Grid over frequency", 7: "Grid under frequency", 8: "Grid unbalance",
    10: "Grid frequency fluctuation", 11: "Grid phase abnormal", 12: "No grid", 13: "Grid reverse",
    14: "Grid AB transient overvoltage", 15: "Hardware overcurrent screening failure",
}
BITS_FAULT2 = {
    0: "DC over voltage 01", 1: "DC over voltage 02", 2: "DC reverse",
    3: "Master/slave bus voltage inconsistent", 4: "DC bus over voltage", 5: "DC bus under voltage",
    6: "DC bus unbalance", 7: "DC bus voltage detection abnormal",
    8: "Grid AB average overvoltage", 9: "Grid BC average overvoltage", 10: "Grid CA average overvoltage",
    11: "PV mid-point grounding", 12: "DC boost fault", 13: "DC hardware overcurrent",
    14: "Grid current tracking fault", 15: "Grid voltage instantaneous overvoltage",
}
BITS_FAULT3 = {
    0: "Phase A overcurrent", 1: "Phase B overcurrent", 2: "Phase C overcurrent",
    3: "DC1 average overcurrent", 4: "DC2 average overcurrent", 5: "AC hardware overcurrent",
    6: "DC component of current exceeds limit",
    7: "Grid AB overvoltage 02", 8: "Grid BC overvoltage 02", 9: "Grid CA overvoltage 02",
    10: "Grid AB undervoltage 02", 11: "Grid BC undervoltage 02", 12: "Grid CA undervoltage 02",
    13: "Grid overfrequency 02", 14: "Grid underfrequency 02", 15: "Grid overvoltage 03",
}
BITS_FAULT4 = {
    0: "Battery overcurrent", 1: "Module over temperature", 2: "System over temperature",
    3: "Relay failure", 4: "PV midpoint grounding", 5: "Low temperature protection",
    6: "Negative ground fault", 7: "Positive ground fault", 8: "12V undervoltage",
    9: "Leakage current 01 (30mA)", 10: "Leakage current 02 (60mA)", 11: "Leakage current 03 (150mA)",
    12: "Leakage current 04 (300mA)", 13: "Leakage current sensor failure", 14: "Grid disturbance 02",
    15: "Grid current sampling abnormal",
}
BITS_FAULT5 = {
    0: "Grid side instantaneous overcurrent", 1: "Battery/Vbus overvoltage hardware failure",
    2: "LLC hardware overcurrent", 3: "Battery overvoltage detection", 4: "Battery undervoltage detection",
    5: "AC level 2 hardware overcurrent", 6: "Master-slave DSP communication abnormal",
    7: "Anomaly detected from DSP", 8: "AFCI self-test failure", 9: "AFCI failure",
    10: "Battery not connected", 11: "DSP self-test abnormal", 12: "Grid voltage sampling abnormal",
    13: "DSP software/hardware mismatch", 14: "Inverter overvoltage", 15: "Load overload",
}
BITS_FAULT6 = {
    0: "Fail Safe", 1: "DRM controlled off-grid", 2: "CT connection abnormal", 3: "DRM not connected",
    8: "EPM hard limit protection", 9: "AFCI board communication abnormal",
    10: "AFCI board CT module abnormal", 11: "G100 overcurrent protection",
}
# Appendix 3 - control word (3049): bit set = protection NOT enabled
BITS_3049 = {
    0: "Relay protection disabled", 1: "Leakage current protection disabled", 2: "AFCI protection disabled",
    3: "Grounding protection disabled", 4: "Over-frequency load shedding disabled",
    5: "Grid disturbance 02 disabled", 6: "Multi-MPPT parallel mode", 7: "Constant voltage MPPT mode",
    8: "Grid current AD sampling protection disabled",
}
# Appendix 5 - initial setting state (3221)
BITS_3221 = {
    0: "Model set", 1: "National standard set", 2: "Power curve set", 3: "Module ID flag",
    4: "Fan detection supported", 5: "FCAS running", 6: "AFCI self-test ended",
    7: "AFCI self-test detected arc", 8: "DSP chip type bit8", 9: "DSP chip type bit9",
    10: "IGBT screening complete", 14: "DSP waveform ready",
}
BITS_3113 = {0: "FailSafe on", 1: "MET-CT FailSafe on"}
BITS_3147 = {0: "LVRT", 1: "US Rule21 VRT", 2: "US Rule21 FRT", 3: "Brazil LVRT", 4: "BDEW LVRT"}
BITS_3152 = {0: "Module 1 fault", 1: "Module 2 fault", 2: "Module 3 fault", 3: "Module 4 fault", 4: "Module 5 fault"}
BITS_3154 = {0: "Volt-watt", 1: "Volt-var", 2: "Fixed PF", 3: "Fixed reactive power", 4: "Power-PF"}
BITS_3157 = {0: "PID on"}
BITS_3160 = {0: "Fstart accuracy 0.001Hz"}
BITS_3250 = {0: "Meter on load side", 1: "Meter on grid side", 2: "24H consumption monitoring"}
BITS_3713 = {0: "Over-frequency derating", 1: "Volt-watt", 2: "Volt-var", 3: "Fixed reactive power",
             4: "Fixed PF", 5: "Active-reactive P-Q"}

# --------------------------------------------------------------------------
# Register map
# --------------------------------------------------------------------------

TYPE_SIZE = {"U16": 1, "S16": 1, "U32": 2, "S32": 2}


@dataclass
class Reg:
    addr: int
    key: str
    name: str
    type: str = "U16"           # U16 S16 U32 S32 HEX ENUM BITS SNHEX ASCII TIME
    scale: float = 1
    unit: str = ""
    group: str = ""
    size: int = 0
    table: dict | None = None
    note: str = ""

    def __post_init__(self):
        if not self.size:
            self.size = TYPE_SIZE.get(self.type, 1)


REGS: list[Reg] = []
GROUPS: dict[str, str] = {}


def group(gkey: str, title: str, rows: list[tuple]):
    GROUPS[gkey] = title
    for r in rows:
        addr, key, name, *rest = r
        kw = rest[0] if rest else {}
        REGS.append(Reg(addr, key, name, group=gkey, **kw))


def U(scale=1, unit="", **kw):  return dict(type="U16", scale=scale, unit=unit, **kw)
def S(scale=1, unit="", **kw):  return dict(type="S16", scale=scale, unit=unit, **kw)
def U32(scale=1, unit="", **kw): return dict(type="U32", scale=scale, unit=unit, **kw)
def S32(scale=1, unit="", **kw): return dict(type="S32", scale=scale, unit=unit, **kw)
def E(table, **kw):  return dict(type="ENUM", table=table, **kw)
def B(table, **kw):  return dict(type="BITS", table=table, **kw)
def H(**kw):         return dict(type="HEX", **kw)


group("info", "Device info", [
    (35000, "inverter_type", "Inverter type (35000)", E(INVERTER_TYPE)),
    (3000, "product_model", "Product model", H()),
    (3001, "dsp_version", "DSP software version", H(note="AA=slave DSP, BB=main DSP")),
    (3002, "hmi_major_version", "HMI major version", H()),
    (3021, "hmi_version", "HMI version", H(note="AA=minor, BB=major")),
    (3108, "master_dsp_sub_version", "Main DSP minor version", H()),
    (3003, "ac_output_type", "AC output type", E(AC_OUTPUT_TYPE)),
    (3004, "dc_input_type", "DC input type", U(note="value+1 = number of DC inputs")),
    (3061, "serial_number", "Serial number", dict(type="SNHEX", size=4)),
    (3230, "serial_number_ascii", "Serial number (ASCII)", dict(type="ASCII", size=18)),
    (3110, "rated_apparent_power", "Rated apparent power", U(10, "VA")),
    (3054, "grid_standard", "Grid standard code", H(note="see Appendix 4")),
    (3055, "power_curve_number", "Power curve number", U()),
    (3107, "grid_filter_number", "Grid filter number", U()),
    (3223, "safety_version", "Safety version", U()),
    (3073, "system_time", "System time", dict(type="TIME", size=6)),
    (3079, "drm_number", "DRM number", U()),
    (3094, "installation_method", "Installation method", E(INSTALL_METHOD)),
    (3220, "batch_upgrade_flag", "Batch upgrade support", H(note="hi=ARM, lo=DSP")),
    (3221, "initial_setting_state", "Initial setting state", B(BITS_3221)),
])

group("energy", "Energy", [
    (3009, "total_energy", "Total energy", U32(1, "kWh")),
    (3117, "total_energy_decimal", "Total energy decimal part", U(0.001, "kWh")),
    (3015, "energy_today", "Energy today", U(0.1, "kWh")),
    (3159, "dc_energy_today", "DC energy today", U(0.1, "kWh")),
    (3011, "energy_this_month", "Energy this month", U32(1, "kWh")),
    (3013, "energy_last_month", "Energy last month", U32(1, "kWh")),
    (3017, "energy_this_year", "Energy this year", U32(1, "kWh")),
    (3019, "energy_last_year", "Energy last year", U32(1, "kWh")),
])

group("dc", "DC / PV input", [
    (3007, "total_dc_power", "Total DC input power", U32(1, "W")),
    (3022, "dc_voltage_1", "DC voltage 1", U(0.1, "V")),
    (3023, "dc_current_1", "DC current 1", U(0.1, "A")),
    (3024, "dc_voltage_2", "DC voltage 2", U(0.1, "V")),
    (3025, "dc_current_2", "DC current 2", U(0.1, "A")),
    (3026, "dc_voltage_3", "DC voltage 3", U(0.1, "V")),
    (3027, "dc_current_3", "DC current 3", U(0.1, "A")),
    (3028, "dc_voltage_4", "DC voltage 4", U(0.1, "V")),
    (3029, "dc_current_4", "DC current 4", U(0.1, "A")),
    (3031, "initial_ground_voltage", "Initial ground voltage", U(0.1, "V")),
    (3032, "dc_bus_voltage", "DC bus voltage", U(0.1, "V")),
    (3033, "dc_half_bus_voltage", "DC half-bus voltage", U(0.1, "V")),
    (3092, "pv_insulation_resistance", "PV insulation resistance", U(1, "kOhm")),
    (3087, "insulation_protection_value", "Insulation resistance protection", U(1, "kOhm")),
    (3106, "leakage_current", "Leakage current", U(1, "mA")),
    (3086, "leakage_protection_value", "Leakage current protection", U(1, "mA")),
])

group("ac", "AC output", [
    (3005, "active_power", "Active power", S32(1, "W")),
    (3056, "reactive_power", "Reactive power", S32(1, "var")),
    (3058, "apparent_power", "Apparent power", S32(1, "VA")),
    (3060, "power_factor", "Power factor", S(0.001, note="50-70K/255K only")),
    (3109, "power_percentage", "Real-time power / rated", U(0.01, "%")),
    (3034, "ac_voltage_a", "AB line / A phase voltage", U(0.1, "V")),
    (3035, "ac_voltage_b", "BC line / B phase voltage", U(0.1, "V")),
    (3036, "ac_voltage_c", "CA line / C phase voltage", U(0.1, "V")),
    (3037, "ac_current_a", "A phase current", U(0.1, "A")),
    (3038, "ac_current_b", "B phase current", U(0.1, "A")),
    (3039, "ac_current_c", "C phase current", U(0.1, "A")),
    (3043, "grid_frequency", "Grid frequency", U(0.01, "Hz")),
    (3042, "inverter_temperature", "Inverter temperature (IGBT)", S(0.1, "C")),
    (3093, "inverter_temperature_2", "Inverter temperature 2", S(0.1, "C")),
])

group("status", "Status & faults", [
    (3044, "inverter_status", "Inverter status", H()),
    (3030, "alarm_code", "Alarm sub-code", H()),
    (3095, "limit_status", "Derating / limit status", B(BITS_3095)),
    (3096, "fault_code_1", "Fault code 1", B(BITS_FAULT1)),
    (3097, "fault_code_2", "Fault code 2", B(BITS_FAULT2)),
    (3098, "fault_code_3", "Fault code 3", B(BITS_FAULT3)),
    (3099, "fault_code_4", "Fault code 4", B(BITS_FAULT4)),
    (3100, "fault_code_5", "Fault code 5", B(BITS_FAULT5)),
    (3101, "fault_code_6", "Fault code 6", B(BITS_FAULT6)),
    (3049, "control_word", "Inverter control word", B(BITS_3049)),
    (3119, "g100_status", "G100 running status", E(G100_STATE)),
    (3225, "igbt_selfcheck", "IGBT self-check", E(IGBT_SELFCHECK)),
    (3228, "one_click_reset_enable", "One-click reset enable", E(RESET_ENABLE)),
    (3229, "one_click_reset_command", "One-click reset command", E(RESET_CMD)),
])

group("control", "Power control", [
    (3041, "working_mode", "Standard working mode", E(WORKING_MODE)),
    (3154, "working_mode_status", "Working mode running bits", B(BITS_3154)),
    (3090, "power_limit_switch", "Power limit switch", E(POWER_LIMIT_SWITCH)),
    (3050, "active_power_limit", "Active power limit", U(0.01, "%")),
    (3045, "active_power_limit_output", "Limited active power output", S32(1, "W")),
    (3091, "reactive_power_switch", "Reactive power switch", E(REACTIVE_SWITCH)),
    # Unit column says 0.01 but the remark says "set 1, write 1000" - 0.001 matches the example
    (3051, "pf_adjust_value", "Power factor adjust value", S(0.001)),
    (3052, "pf_adjust_actual", "Actual power factor adjustment", S(0.001)),
    (3053, "reactive_power_percent", "Reactive power setting", S(0.01, "%")),
    (3047, "reactive_power_limit_output", "Limited reactive power output", S32(1, "var")),
    (3158, "voltvar_vref", "Volt-Var Vref", U(0.1, "V")),
    (3155, "standard_select_status", "Power-on standard selection", E(STD_SELECT)),
    (3160, "accuracy_flag", "Accuracy flag", B(BITS_3160)),
    (3161, "meter_or_dual485", "Meter / dual 485 switch", E(METER_485)),
    (3147, "vrt_enable_bits", "Voltage ride-through enable", B(BITS_3147)),
    (3148, "vrt_enable_flag", "Voltage ride-through flag", E(VRT_FLAG)),
])

group("epm", "Internal EPM (export power manager)", [
    (3111, "epm_switch", "EPM soft switch", U(note="meaning depends on model, see PDF p.9")),
    (3112, "epm_backflow_limit", "EPM backflow power limit", S(100, "W")),
    (3113, "epm_failsafe", "EPM failsafe", B(BITS_3113)),
    (3114, "epm_realtime_backflow", "EPM real-time backflow power", U(10, "W")),
    (3115, "epm_hard_switch", "EPM hard switch", E(EPM_HARD_SWITCH)),
    (3116, "epm_hard_limit", "EPM hard limit", S(100, "W")),
    (3118, "ct_link_test", "CT link test", E(CT_LINK)),
    (3157, "external_pid_status", "External PID status", B(BITS_3157)),
])

group("meter", "Meter", [
    (3250, "meter_placement", "Meter placement", B(BITS_3250)),
    (3248, "household_load_energy", "Household load consumption", S32(0.01, "kWh")),
    (3251, "meter_voltage_a", "Meter voltage A", U(0.1, "V")),
    (3252, "meter_current_a", "Meter current A", U(0.01, "A")),
    (3253, "meter_voltage_b", "Meter voltage B", U(0.1, "V")),
    (3254, "meter_current_b", "Meter current B", U(0.01, "A")),
    (3255, "meter_voltage_c", "Meter voltage C", U(0.1, "V")),
    (3256, "meter_current_c", "Meter current C", U(0.01, "A")),
    (3257, "meter_active_power_a", "Meter active power A", S32(0.001, "kW")),
    (3259, "meter_active_power_b", "Meter active power B", S32(0.001, "kW")),
    (3261, "meter_active_power_c", "Meter active power C", S32(0.001, "kW")),
    (3263, "meter_active_power", "Meter total active power", S32(0.001, "kW", note="+ export, - import")),
    (3265, "meter_reactive_power_a", "Meter reactive power A", S32(1, "var")),
    (3267, "meter_reactive_power_b", "Meter reactive power B", S32(1, "var")),
    (3269, "meter_reactive_power_c", "Meter reactive power C", S32(1, "var")),
    (3271, "meter_reactive_power", "Meter total reactive power", S32(1, "var")),
    (3273, "meter_apparent_power_a", "Meter apparent power A", S32(1, "VA")),
    (3275, "meter_apparent_power_b", "Meter apparent power B", S32(1, "VA")),
    (3277, "meter_apparent_power_c", "Meter apparent power C", S32(1, "VA")),
    (3279, "meter_apparent_power", "Meter total apparent power", S32(1, "VA")),
    (3281, "meter_power_factor", "Meter power factor", S(0.001)),
    (3282, "meter_frequency", "Meter grid frequency", U(0.01, "Hz")),
    (3283, "meter_total_active_energy", "Meter grid total active energy", U32(0.01, "kWh")),
    (3285, "meter_transmission_energy", "Meter grid transmission total energy", U32(0.01, "kWh")),
])

group("strings", "PV strings (models with >= 4 DC inputs)", [
    (3287, "pv_combination", "PV voltage/current combination", E(PV_COMBO)),
    (3299, "pv_total_voltage", "Total PV voltage", U(0.1, "V")),
    (3300, "pv_total_current", "Total PV current", S(0.1, "A")),
    *[(3300 + i, f"pv{i}_current", f"PV{i} current", S(0.1, "A")) for i in range(1, 21)],
    *[(3288 + i - 20, f"pv{i}_current", f"PV{i} current", S(0.1, "A")) for i in range(21, 31)],
    (3337, "pv31_current", "PV31 current", U(0.1, "A")),
    (3338, "pv32_current", "PV32 current", U(0.1, "A")),
    *[(3320 + i, f"pv_string{i}_voltage", f"PV string {i} voltage", U(0.1, "V")) for i in range(1, 17)],
])

group("mppt", "MPPT (models with >= 8 DC inputs)", [
    *[(3499 + i, f"mppt{i}_voltage", f"MPPT {i} voltage", U(0.1, "V")) for i in range(1, 16)],
    *[(3529 + i, f"mppt{i}_current", f"MPPT {i} current", S(0.1, "A")) for i in range(1, 16)],
    (3545, "mppt16_voltage", "MPPT 16 voltage", S(0.1, "V")),
    (3546, "mppt16_current", "MPPT 16 current", S(0.1, "A")),
])

group("temps", "Module temperatures (250K series)", [
    (3570, "ntc_phase_a", "A phase module NTC", U(0.1, "C")),
    (3571, "ntc_phase_b", "B phase module NTC", U(0.1, "C")),
    (3572, "ntc_phase_c", "C phase module NTC", U(0.1, "C")),
    *[(3572 + i, f"dc_ntc{i}", f"DC NTC{i}", U(0.1, "C")) for i in range(1, 9)],
])

group("afci", "AFCI (arc fault)", [
    (3149, "afci_board_model", "AFCI board model", E(AFCI_BOARD)),
    (3150, "afci_board_version", "AFCI board version", H()),
    (3151, "afci_fault_data_ready", "AFCI fault data obtained", E(NO_YES)),
    (3152, "afci_ct_module_faults", "AFCI CT module faults", B(BITS_3152)),
    (3153, "afci_arc_fault_string", "AFCI arc fault string", U(note="1-30 string, 31-35 module")),
    (3120, "arc_fault_count", "Arc fault occurrences", U()),
    (3224, "afci_selfcheck_done", "AFCI self-check completed", U()),
])

group("dispatch", "Plant dispatch / adjustable capacity", [
    (3601, "pv_adjustable", "Adjustable now", E(NO_YES)),
    (3602, "p_adjust_up", "Active power adjustable up", S32(1, "W")),
    (3604, "p_adjust_down", "Active power adjustable down", S32(1, "W")),
    (3606, "q_cap_adjust_up", "Capacitive Q adjustable up", S32(1, "var")),
    (3608, "q_cap_adjust_down", "Capacitive Q adjustable down", S32(1, "var")),
    (3610, "q_ind_adjust_up", "Inductive Q adjustable up", S32(1, "var")),
    (3612, "q_ind_adjust_down", "Inductive Q adjustable down", S32(1, "var")),
    (3614, "v_upper_at_max_q", "Voltage upper limit at max Q", U(0.1, "V")),
    (3615, "v_lower_at_max_q", "Voltage lower limit at max Q", U(0.1, "V")),
    (3616, "pv_total_p_control", "Total active power control", E(OFF_ON)),
    (3617, "pv_total_q_control", "Total reactive power control", E(OFF_ON)),
    (3618, "pv_total_v_control", "Total voltage control", E(OFF_ON)),
    (3619, "init_completed", "Initialization completed", E(NO_YES)),
    (3620, "p_control_status", "Active power control", E(OFF_ON)),
    (3621, "q_control_status", "Reactive power control", E(OFF_ON)),
    (3622, "v_control_status", "Voltage control", E(OFF_ON)),
    (3623, "full_generation", "Fully generating", E(NO_YES)),
    (3624, "night_q_capable", "Night reactive power capable", E(NO_YES)),
    (3625, "p_control_done", "Active power control completed", E(NO_YES)),
    (3626, "q_control_done", "Reactive power control completed", E(NO_YES)),
    (3627, "v_control_done", "Voltage adjustment completed", E(NO_YES)),
    (3628, "remote_controllable", "Remotely controllable", E(NO_YES)),
    (3629, "dc_switch", "DC switch position", E(OFF_ON)),
    (3630, "ac_switch", "AC switch position", E(OFF_ON)),
    (3631, "emergency_shutdown", "Emergency shutdown", E({0: "Disabled", 1: "Enabled"})),
    (3632, "starting", "Starting", E({0: "Startup completed", 1: "Startup in progress"})),
    (3633, "alarm_running", "Alarm", E({0: "No alarm", 1: "Alarm running"})),
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
    (3121, "italy_single_selftest", "Single self-test item", U()),
    (3122, "italy_full_selftest", "Full self-test", E(ITALY_FULL_TEST)),
    *_italy_rows(3123, ""),
    *_italy_rows(3650, "B"),
    *_italy_rows(3670, "C"),
])

group("us", "Rated data / US grid code", [
    (3700, "rated_active_power", "Rated active power", U32(1, "W")),
    (3702, "p_at_pf_lead_08", "Active power at PF +0.8", U32(1, "W")),
    (3704, "leading_pf", "Leading power factor", U(0.01)),
    (3705, "p_at_pf_lag_08", "Active power at PF -0.8", U32(1, "W")),
    (3707, "lagging_pf", "Lagging power factor", U(0.01)),
    (3708, "perf_category_normal", "Normal performance category", E(PERF_NORMAL)),
    (3709, "perf_category_abnormal", "Abnormal performance category", E(PERF_ABNORMAL)),
    (3710, "ac_rated_voltage", "AC rated voltage", U(1, "V")),
    (3711, "ac_ov1_limit", "AC level 1 overvoltage limit", U(0.1, "V")),
    (3712, "ac_uv1_limit", "AC level 1 undervoltage limit", U(0.1, "V")),
    (3713, "control_mode", "Control mode", B(BITS_3713)),
    (3714, "admittance", "Admittance parameter (raw)", U()),
    (3715, "manufacturer", "Manufacturer", E(MANUFACTURER)),
    (3716, "hw_version_mainboard", "Mainboard hardware version", H()),
    (3717, "rt_reactive_power", "Real-time reactive power", U32(1, "var")),
    (3719, "q_cap_max", "Capacitive max reactive power", U32(1, "var")),
    (3721, "q_ind_max", "Inductive max reactive power", U32(1, "var")),
    (3733, "iv_curve_type", "IV curve type", E(IV_TYPE)),
    (3734, "iv_scan_flag", "IV scan state", E(IV_SCAN)),
    (3735, "iv_string_number", "IV curve string number", U()),
])

group("calib", "Multi-DSP (80-110K MAX/PRO)", [
    *[(3142 + i, f"dsp{i}_version", f"DSP{i} version", H()) for i in range(1, 5)],
    *[(3204 + 3 * (d - 1) + p, f"dsp{d}_voltage_{'abc'[p]}", f"DSP{d} {'ABC'[p]} phase voltage", U(0.1, "V"))
      for d in range(1, 5) for p in range(3)],
])

DEFAULT_GROUPS = ["info", "energy", "dc", "ac", "status", "control", "epm", "meter",
                  "strings", "mppt", "temps", "afci", "dispatch", "italy", "us", "calib"]

_keys = [r.key for r in REGS]
assert len(_keys) == len(set(_keys)), "duplicate register keys"


def wire_address(doc_addr: int) -> int:
    # Section 5.1 (35000) uses the address as-is; 5.2 (3000-3999) needs address - 1
    return doc_addr if doc_addr >= 30000 else doc_addr - 1


# --------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------

@dataclass
class Span:
    start: int
    count: int
    regs: list[Reg] = field(default_factory=list)


def build_spans(regs: list[Reg], max_gap: int = 10) -> list[Span]:
    spans: list[Span] = []
    for r in sorted(regs, key=lambda x: x.addr):
        end = r.addr + r.size
        cur = spans[-1] if spans else None
        if (cur and r.addr - (cur.start + cur.count) <= max_gap
                and end - cur.start <= MAX_REGS_PER_READ
                and (r.addr >= 30000) == (cur.start >= 30000)):
            cur.count = max(cur.count, end - cur.start)
            cur.regs.append(r)
        else:
            spans.append(Span(r.addr, r.size, [r]))
    return spans


def read_regs(client: RtuClient, regs: list[Reg], log: Callable[[str], None]) -> tuple[dict[int, list[int]], dict[int, str]]:
    """Returns ({addr: [words]}, {addr: error}) per register definition."""
    raw: dict[int, list[int]] = {}
    errors: dict[int, str] = {}

    def do_span(span: Span) -> None:
        try:
            words = client.read_input(wire_address(span.start), span.count)
        except ModbusException as e:
            # Part of the span may be unsupported on this model: split and retry
            if len(span.regs) > 1:
                mid = len(span.regs) // 2
                for half in (span.regs[:mid], span.regs[mid:]):
                    for sub in build_spans(half):
                        do_span(sub)
                return
            for r in span.regs:
                errors[r.addr] = str(e)
            return
        except ModbusError as e:
            log(f"  ! {span.start}-{span.start + span.count - 1}: {e}")
            for r in span.regs:
                errors[r.addr] = str(e)
            return
        for r in span.regs:
            off = r.addr - span.start
            raw[r.addr] = words[off:off + r.size]

    for span in build_spans(regs):
        do_span(span)
    return raw, errors


# --------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------

def _decimals(scale: float) -> int:
    if scale >= 1:
        return 0
    return max(0, len(f"{scale:.6f}".rstrip("0").split(".")[1]))


def _bits(v: int, table: dict) -> list[str]:
    return [table.get(b, f"bit{b}") for b in range(16) if v >> b & 1]


def decode(r: Reg, words: list[int]) -> tuple[object, str]:
    """Returns (value, human text)."""
    t = r.type
    w = words[0]
    if t in ("U16", "S16", "U32", "S32"):
        if t == "U16":
            v = w
        elif t == "S16":
            v = w - 0x10000 if w & 0x8000 else w
        else:
            v = (words[0] << 16) | words[1]
            if t == "S32" and v & 0x80000000:
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
    if t == "SNHEX":
        sn = "".join(f"{x:04X}"[::-1] for x in words)
        return sn, sn
    if t == "ASCII":
        b = b"".join(struct.pack(">H", x) for x in words)
        s = b.replace(b"\x00", b"").decode("ascii", "replace").strip()
        return s, s
    if t == "TIME":
        y, mo, d, h, mi, s = words
        ts = f"20{y:02d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"
        return ts, ts
    return w, str(w)


def status_text(status: int, sub: int) -> str:
    base = STATUS.get(status, f"Unknown status 0x{status:04X}")
    if status == 0x0003 and sub:
        flags = [n for m, n in GENERATING_LIMITS.items() if sub & m]
        return f"{base} - derating: {', '.join(flags) or f'0x{sub:04X}'}"
    if (status, sub) in STATUS_SUB:
        return STATUS_SUB[(status, sub)]
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


def poll(client: RtuClient, groups: list[str], respect_dc_type: bool, log: Callable[[str], None]) -> dict[str, Result]:
    results: dict[str, Result] = {}

    def run(regs: list[Reg]) -> None:
        raw, errors = read_regs(client, regs, log)
        for r in regs:
            if r.addr in raw:
                v, txt = decode(r, raw[r.addr])
                results[r.key] = Result(r, raw[r.addr], v, txt)
            else:
                results[r.key] = Result(r, None, error=errors.get(r.addr, "not read"))

    gated = {"strings", "mppt"}
    run([r for r in REGS if r.group in groups and r.group not in gated])

    want = [g for g in groups if g in gated]
    if want:
        n_inputs = None
        dct = results.get("dc_input_type")
        if dct is None:
            run([r for r in REGS if r.key == "dc_input_type"])
            dct = results.pop("dc_input_type")
        if dct.raw:
            n_inputs = dct.raw[0]
        if respect_dc_type and n_inputs is not None:
            # Section 5.2 / reg 3004: >=4 -> 3287-3338, >=8 -> 3500-3546
            want = [g for g in want if (g == "strings" and n_inputs >= 4) or (g == "mppt" and n_inputs >= 8)]
        if want:
            run([r for r in REGS if r.group in want])

    # Derived values
    st, al = results.get("inverter_status"), results.get("alarm_code")
    if st and st.raw:
        st.text = f"{status_text(st.raw[0], al.raw[0] if al and al.raw else 0)}  [0x{st.raw[0]:04X}]"
    dct = results.get("dc_input_type")
    if dct and dct.raw:
        dct.value = dct.raw[0] + 1
        dct.text = f"{dct.value} DC input(s)"
    te, td = results.get("total_energy"), results.get("total_energy_decimal")
    if te and te.raw and td and td.raw:
        te.value = round(te.value + td.value, 3)
        te.text = f"{te.value} kWh"
    for i in range(1, 5):
        v, c = results.get(f"dc_voltage_{i}"), results.get(f"dc_current_{i}")
        if v and c and v.raw and c.raw:
            key = f"dc_power_{i}"
            p = round(v.value * c.value, 1)
            reg = Reg(0, key, f"DC power {i} (V x I)", unit="W", group="dc")
            results[key] = Result(reg, [], p, f"{p} W")
    return results


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------

def print_table(results: dict[str, Result], show_raw: bool, show_errors: bool) -> None:
    print(f"\n=== Solis inverter  {datetime.now():%Y-%m-%d %H:%M:%S} ===")
    for gkey, title in GROUPS.items():
        rows = [r for r in results.values() if r.reg.group == gkey and (r.raw is not None or show_errors)]
        if not rows:
            continue
        print(f"\n[{title}]")
        for res in rows:
            addr = f"{res.reg.addr:>5}" if res.reg.addr else "  calc"
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
            row[k] = r.value if not isinstance(r.value, list) else ";".join(map(str, r.value))
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

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Read all operating data from a Solis grid-tied inverter over Modbus RTU (RS485).")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--unit", type=int, default=1, help="inverter slave address (default 1)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--timeout", type=float, default=1.0, help="response timeout in seconds")
    ap.add_argument("--retries", type=int, default=1, help="retries on timeout/CRC error")
    ap.add_argument("--groups", default=",".join(DEFAULT_GROUPS),
                    help=f"comma separated groups (default all): {','.join(DEFAULT_GROUPS)}")
    ap.add_argument("--force-all", action="store_true",
                    help="read PV string / MPPT groups even if register 3004 says the model doesn't have them")
    ap.add_argument("--interval", type=float, default=0, help="poll repeatedly every N seconds")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    ap.add_argument("--csv", metavar="FILE", help="append each poll as a row to a CSV file")
    ap.add_argument("--raw", action="store_true", help="show raw register words")
    ap.add_argument("--show-unsupported", action="store_true", help="list registers the inverter refused")
    ap.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    args = ap.parse_args(argv)

    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:<12} {p.description}")
        return 0
    if not args.port:
        ap.error("--port is required (use --list-ports to find it)")

    groups = [g.strip() for g in args.groups.split(",") if g.strip()]
    bad = [g for g in groups if g not in GROUPS]
    if bad:
        ap.error(f"unknown group(s): {', '.join(bad)}")

    log = (lambda m: print(m, file=sys.stderr))
    client = RtuClient(args.port, args.unit, args.baud, args.timeout, args.retries)
    try:
        while True:
            t0 = time.monotonic()
            results = poll(client, groups, not args.force_all, log)
            if not any(r.raw is not None for r in results.values()):
                log("No data received - check wiring (A/B), slave address, baud rate and that the inverter is powered.")
            elif args.json:
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
