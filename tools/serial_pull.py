"""Pull a file off the PortaPack over USB serial -- no SD swap, no firmware change.

The v240 shell's fread already works (the old "reads are broken" note was
wrong): fopen, then fread returns the bytes hex-encoded, 62 raw bytes per call.
This drives that: fopen -> filesize -> loop fread -> hex-decode -> reassemble.

    python3 serial_pull.py /LOGS/MESHTAST.TXT out.txt [--port /dev/ttyACM1]

Reading the live log while the app appends to it is the whole point; if FatFS
refuses the second open, close the app first (this reports the fopen error
rather than hanging).
"""
import argparse
import re
import sys
import time

import serial

PROMPT = b"ch> "


def drain(s, quiet=0.25, cap=8.0):
    """Read until the shell prompt returns or output goes quiet."""
    buf = bytearray()
    t0 = time.time()
    last = time.time()
    while time.time() - t0 < cap:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            last = time.time()
            if buf.endswith(PROMPT):
                break
        else:
            if time.time() - last > quiet and buf.endswith(PROMPT):
                break
            time.sleep(0.01)
    return buf.decode("latin1", "replace")


def cmd(s, c, cap=8.0):
    s.reset_input_buffer()
    s.write((c + "\r\n").encode())
    return drain(s, cap=cap)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("remote")
    ap.add_argument("out")
    ap.add_argument("--port", default="/dev/ttyACM1")
    ap.add_argument("--chunk", type=int, default=2048,
                    help="bytes per fread request (device caps each read at 62)")
    args = ap.parse_args()

    s = serial.Serial(args.port, 115200, timeout=2)
    cmd(s, "")  # sync to a prompt

    r = cmd(s, f"filesize {args.remote}")
    m = re.search(r"\r\n(\d+)\r\nok", r)
    if not m:
        print(f"filesize failed: {r!r}", file=sys.stderr)
        return 1
    total = int(m.group(1))
    print(f"{args.remote}: {total} bytes")

    r = cmd(s, f"fopen {args.remote}")
    if "ok" not in r:
        print(f"fopen failed (app may hold the file): {r!r}", file=sys.stderr)
        return 1
    cmd(s, "fseek 0")

    data = bytearray()
    t0 = time.time()
    while len(data) < total:
        want = min(args.chunk, total - len(data))
        r = cmd(s, f"fread {want}")
        # Strip the echoed command and the trailing ok/prompt; keep hex only.
        hexs = "".join(re.findall(r"[0-9A-Fa-f]{2,}", r.split("\r\n", 1)[-1]))
        # Guard: the "ok" and prompt are not hex pairs of our data.
        try:
            chunk = bytes.fromhex(hexs)
        except ValueError:
            chunk = bytes.fromhex(hexs[: len(hexs) // 2 * 2])
        if not chunk:
            print(f"\nstalled at {len(data)}/{total}: {r!r}", file=sys.stderr)
            break
        data += chunk
        print(f"\r  {len(data)}/{total} bytes", end="", flush=True)
    cmd(s, "fclose")
    s.close()

    data = data[:total]
    with open(args.out, "wb") as fh:
        fh.write(data)
    dt = time.time() - t0
    print(f"\n{len(data)} bytes in {dt:.1f}s ({len(data)/max(dt,0.1):.0f} B/s) -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
