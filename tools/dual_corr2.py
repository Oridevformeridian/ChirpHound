"""Two radios, matched per transmission. Noise or systematic?

The first attempt keyed on beacon sequence numbers and got n=3, because full
decryption is rare and the plaintext itself is corrupted ('BECCMN', 'BEACO\\x0e')
so the word is not even reliably present.

This matches on time instead, which works at the frame level and gives ~10x the
sample. The two captures started about a second apart, and beacons are 5 s
apart, so the offset is recoverable: try candidate offsets and keep the one
that aligns the most preamble locks.

Success is graded, not binary, because full decryption is too rare to count:

  frame     the header decoded at all
  sender    the 4-byte sender reads !4358ab2c exactly -- 32 bits, so it cannot
            happen by chance, and it is far more common than a full decrypt
  decrypt   cryptographic confirmation

If the same transmissions succeed on both radios, the fault is systematic. If
success is roughly independent, it is thermal noise and the lever is RF.
"""
import sys

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

TRUE_SENDER = 0x4358AB2C
TOL_MS = 1200          # under half the 5 s beacon spacing


def events(path, chans):
    """[(t_ms, has_frame, sender_ok, decrypted)] for frames; plus lock times."""
    frames, locks = [], []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 2:
                continue
            try:
                t = int(p[0])
            except ValueError:
                continue
            if p[1] == "P":
                locks.append(t)
            elif p[1] == "F" and len(p) > 2:
                sender_ok = dec = False
                try:
                    raw = bytes.fromhex(p[2])
                    rec = packets.process(raw, chans)
                    if rec:
                        sender_ok = (rec.get("sender") == TRUE_SENDER)
                        dec = bool(rec.get("decrypted"))
                except Exception:
                    pass
                frames.append((t, True, sender_ok, dec))
    return frames, locks


def best_offset(la, lb):
    """Offset (ms) applied to B that aligns the most locks with A."""
    best, best_n = 0, -1
    for off in range(-4000, 4001, 50):
        n = 0
        j = 0
        sb = sorted(x + off for x in lb)
        for t in sorted(la):
            while j < len(sb) and sb[j] < t - TOL_MS:
                j += 1
            if j < len(sb) and abs(sb[j] - t) <= TOL_MS:
                n += 1
        if n > best_n:
            best_n, best = n, off
    return best, best_n


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    chans = keys.load(keys.DEFAULT_PATH)
    fa, la = events(sys.argv[1], chans)
    fb, lb = events(sys.argv[2], chans)

    off, aligned = best_offset(la, lb)
    print(f"radio A: {len(la)} locks, {len(fa)} frames")
    print(f"radio B: {len(lb)} locks, {len(fb)} frames")
    print(f"time offset for B: {off:+d} ms  ({aligned} locks aligned "
          f"of {min(len(la), len(lb))})\n")

    # Build the union of transmissions from the aligned locks.
    tx = sorted(la + [x + off for x in lb])
    merged = []
    for t in tx:
        if merged and t - merged[-1][-1] <= TOL_MS:
            merged[-1].append(t)
        else:
            merged.append([t])
    slots = [sum(g) / len(g) for g in merged]
    print(f"{len(slots)} distinct transmissions heard by at least one radio\n")

    def hit(frames, t, idx, shift=0):
        for f in frames:
            if abs((f[0] + shift) - t) <= TOL_MS and f[idx]:
                return True
        return False

    print(f"  {'metric':<9} {'A only':>7} {'B only':>7} {'both':>6} "
          f"{'union':>6} {'expect':>7} {'ratio':>6}")
    for name, idx in (("frame", 1), ("sender", 2), ("decrypt", 3)):
        a = set(i for i, t in enumerate(slots) if hit(fa, t, idx))
        b = set(i for i, t in enumerate(slots) if hit(fb, t, idx, off))
        both, union = a & b, a | b
        exp = len(a) * len(b) / len(slots) if slots else 0
        ratio = (len(both) / exp) if exp > 0 else 0.0
        print(f"  {name:<9} {len(a-b):>7} {len(b-a):>7} {len(both):>6} "
              f"{len(union):>6} {exp:>7.1f} {ratio:>6.2f}")

    print("\n  ratio ~1  => independent: thermal noise decides, lever is RF")
    print("  ratio >2  => systematic: same packets fail on both radios")
    return 0


if __name__ == "__main__":
    sys.exit(main())
