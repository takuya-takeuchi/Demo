import sys
import uuid
from pathlib import Path

TARGET_UUID = uuid.UUID("09452e60-e626-c69e-a7ad-5ad265449e64").bytes

def find_start_codes(data: bytes):
    i = 0
    n = len(data)
    while i + 3 < n:
        if data[i:i+3] == b"\x00\x00\x01":
            yield i, 3
            i += 3
        elif i + 4 < n and data[i:i+4] == b"\x00\x00\x00\x01":
            yield i, 4
            i += 4
        else:
            i += 1

def split_nals_annexb(data: bytes):
    starts = list(find_start_codes(data))
    for idx, (pos, sc_len) in enumerate(starts):
        start = pos + sc_len
        end = starts[idx + 1][0] if idx + 1 < len(starts) else len(data)
        nal = data[start:end]
        if nal:
            yield nal

def rbsp_from_ebsp(ebsp: bytes) -> bytes:
    out = bytearray()
    i = 0
    zeros = 0
    while i < len(ebsp):
        b = ebsp[i]
        if zeros == 2 and b == 0x03:
            i += 1
            zeros = 0
            continue
        out.append(b)
        if b == 0:
            zeros += 1
        else:
            zeros = 0
        i += 1
    return bytes(out)

def parse_sei_rbsp(rbsp: bytes):
    i = 0
    results = []
    # rbsp trailing bits の前まで雑に走査
    while i + 1 < len(rbsp):
        payload_type = 0
        while i < len(rbsp) and rbsp[i] == 0xFF:
            payload_type += 255
            i += 1
        if i >= len(rbsp):
            break
        payload_type += rbsp[i]
        i += 1

        payload_size = 0
        while i < len(rbsp) and rbsp[i] == 0xFF:
            payload_size += 255
            i += 1
        if i >= len(rbsp):
            break
        payload_size += rbsp[i]
        i += 1

        if i + payload_size > len(rbsp):
            break

        payload = rbsp[i:i+payload_size]
        i += payload_size
        results.append((payload_type, payload))
    return results

def main(path: str):
    data = Path(path).read_bytes()
    found = 0

    for nal_index, nal in enumerate(split_nals_annexb(data)):
        nal_type = nal[0] & 0x1F  # H.264
        if nal_type != 6:
            continue

        rbsp = rbsp_from_ebsp(nal[1:])  # NAL header 1 byte を除く
        for payload_type, payload in parse_sei_rbsp(rbsp):
            if payload_type != 5:  # user_data_unregistered
                continue
            if len(payload) < 16:
                continue

            sei_uuid = payload[:16]
            body = payload[16:]

            if sei_uuid != TARGET_UUID:
                continue

            print(f"[MATCH] nal_index={nal_index}")

            print("  uuid =", str(uuid.UUID(bytes=sei_uuid)))
            print("  body_len =", len(body))

            # あなたの現在の実装:
            # byte 0      : version
            # byte 1..7   : reserved
            # byte 8..15  : wallclock_ns (big-endian uint64)
            if len(body) >= 16:
                version = body[0]
                wallclock_ns = int.from_bytes(body[8:16], "big")
                print("  version =", version)
                print("  wallclock_ns =", wallclock_ns)
            else:
                print("  body too short for wallclock format")

            found += 1

    if found == 0:
        print("No matching user_data_unregistered SEI found for target UUID.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python read_sei_wallclock.py out.h264")
        sys.exit(1)
    main(sys.argv[1])