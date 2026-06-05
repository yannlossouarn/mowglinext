#!/usr/bin/env python3
"""
Test UBX-CFG-VALGET for the 10 parameters that timeout in ublox_dgnss.
Sends each key individually (not batched) to determine whether the F9P
responds to them one-at-a-time.

Usage: stop mowgli-gps container first, then:
  python3 test_valget_one_by_one.py [/dev/gps] [baud]
"""

import serial, struct, time, sys

DEVICE = sys.argv[1] if len(sys.argv) > 1 else "/dev/gps"
BAUD   = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

# The 10 parameters that timeout in batched VALGET mode
FAILING_KEYS = {
    "CFG_ODO_USE_ODO":              0x10220001,
    "CFG_ODO_VELLPGAIN":            0x20220031,
    "CFG_RATE_TIMEREF":             0x20210003,
    "CFG_SEC_JAMDET_SENSITIVITY_HI": 0x10f60051,
    "CFG_SEC_SPOOFDET_SIM_SIG_DIS": 0x10f6005d,
    "CFG_SIGNAL_QZSS_L1S_ENA":     0x10310014,
    "CFG_SIGNAL_SBAS_L1CA_ENA":    0x10310005,
    "CFG_SPARTN_USE_SOURCE":        0x20a70001,
    "CFG_TMODE_ECEF_X":            0x40030003,
    "CFG_TMODE_ECEF_X_HP":         0x20030006,
}

# A few known-good keys to use as a sanity baseline
KNOWN_GOOD = {
    "CFG_RATE_MEAS":           0x30210001,
    "CFG_RATE_NAV":            0x30210002,
    "CFG_NAVHPG_DGNSSMODE":   0x20240011,
}


def ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_valget(key_id: int, layer: int = 0) -> bytes:
    """Build a UBX-CFG-VALGET frame for a single key."""
    cls, msg_id = 0x06, 0x8B
    # payload: version(1) layer(1) position(2) key_id(4)
    payload = struct.pack("<BBH I", 0, layer, 0, key_id)
    length  = len(payload)
    header  = struct.pack("<BBHH", 0xB5, 0x62, (cls << 8) | msg_id, length)
    # checksum covers class + id + length + payload
    ck_data = struct.pack("<HH", (cls << 8) | msg_id, length) + payload
    ck_a, ck_b = ubx_checksum(bytes([cls, msg_id]) + struct.pack("<H", length) + payload)
    return bytes([0xB5, 0x62, cls, msg_id]) + struct.pack("<H", length) + payload + bytes([ck_a, ck_b])


def read_ubx_response(ser: serial.Serial, timeout_s: float = 2.0,
                       want_class: int | None = None,
                       want_id: int | None = None) -> bytes | None:
    """Read UBX frames until we get one matching (want_class, want_id), or timeout.

    Skips unsolicited frames (NAV-PVT, etc.) that arrive while waiting.
    If want_class/want_id are None, returns the first valid frame.
    """
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(256)
        if b:
            buf.extend(b)
        # scan for UBX sync
        i = 0
        while i < len(buf) - 1:
            if buf[i] != 0xB5 or buf[i + 1] != 0x62:
                i += 1
                continue
            if len(buf) - i < 6:
                break
            length = struct.unpack_from("<H", buf, i + 4)[0]
            total  = 6 + length + 2
            if len(buf) - i < total:
                break
            frame = bytes(buf[i:i + total])
            buf   = buf[i + total:]
            i     = 0
            # verify checksum
            ck_a, ck_b = ubx_checksum(frame[2:-2])
            if frame[-2] != ck_a or frame[-1] != ck_b:
                continue  # bad checksum, skip
            cls, mid = frame[2], frame[3]
            # If we're filtering, skip non-matching frames
            if want_class is not None and cls != want_class:
                continue
            if want_id is not None and mid != want_id:
                continue
            return frame
    return None


def test_key(ser: serial.Serial, name: str, key_id: int) -> str:
    ser.reset_input_buffer()
    frame = build_valget(key_id, layer=0)  # RAM layer
    ser.write(frame)
    # Wait specifically for a VALGET response (0x06/0x8B) or ACK/NAK (0x05/*)
    resp = read_ubx_response(ser, timeout_s=2.0, want_class=0x06, want_id=0x8B)
    if resp is None:
        # Also check for NAK (0x05/0x00)
        ser.reset_input_buffer()
        frame = build_valget(key_id, layer=0)
        ser.write(frame)
        resp = read_ubx_response(ser, timeout_s=2.0, want_class=0x05)
        if resp is None:
            return "TIMEOUT (no VALGET response or ACK/NAK within 2 s)"
        mid = resp[3]
        if mid == 0x00:
            return "NAK  (device explicitly rejected key — not supported on this firmware)"
        return f"ACK  (unexpected bare ACK, class=0x05 id=0x{mid:02x})"
    length = struct.unpack_from("<H", resp, 4)[0]
    return f"OK   (payload {length} B)"


def main():
    print(f"Opening {DEVICE} at {BAUD} baud …")
    with serial.Serial(DEVICE, BAUD, timeout=0.1) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        print("\n── Sanity baseline (known-good keys) ──────────────────")
        for name, key_id in KNOWN_GOOD.items():
            result = test_key(ser, name, key_id)
            print(f"  {name:<35} {result}")

        print("\n── Failing keys (individual VALGET) ───────────────────")
        for name, key_id in FAILING_KEYS.items():
            result = test_key(ser, name, key_id)
            print(f"  {name:<40} {result}")

    print()


if __name__ == "__main__":
    main()
