"""MAVLink parameter helpers for PX4.

PX4 uses byte-wise encoding for integer parameters: the 4 bytes of the
int32 travel inside PARAM_SET/PARAM_VALUE's float field. Sending the
float 101.0 for an INT32 param therefore sets it to 1120534528 (the bit
pattern of 101.0f). These helpers pack and unpack correctly and read the
value back so a wrong set is visible.
"""
import struct
import time

from pymavlink import mavutil


def int_to_param_float(value):
    return struct.unpack('<f', struct.pack('<i', int(value)))[0]


def param_float_to_int(value):
    return struct.unpack('<i', struct.pack('<f', value))[0]


def _param_id(m):
    pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode(errors='replace')
    return pid.rstrip('\x00')


def set_int_param(gcs, name, value, tries=5, timeout=1.5):
    """Set an INT32 param and verify by readback. Returns True on match."""
    if isinstance(name, bytes):
        name = name.decode()
    for _ in range(tries):
        gcs.mav.param_set_send(gcs.target_system, gcs.target_component, name.encode(),
                               int_to_param_float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = gcs.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if m is None or _param_id(m) != name:
                continue
            if m.param_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                                mavutil.mavlink.MAV_PARAM_TYPE_INT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
                                mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT8):
                got = param_float_to_int(m.param_value)
            else:
                got = int(round(m.param_value))
            return got == int(value)
    return False


def set_float_param(gcs, name, value, tries=5, timeout=1.5):
    """Set a REAL32 param and verify by readback (within 1e-4 relative)."""
    for _ in range(tries):
        gcs.mav.param_set_send(gcs.target_system, gcs.target_component, name.encode(),
                               float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = gcs.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if m is None or _param_id(m) != name:
                continue
            return abs(m.param_value - float(value)) <= 1e-4 * max(1.0, abs(float(value)))
    return False
