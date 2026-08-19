"""Decode success against per-packet signal strength.

The captures already contain what a new experiment would have to stage: the
beacon, the T-Beam, the pager and third-party nodes, at a spread of distances
and therefore a spread of strengths. Every lock now reports its mean preamble
sharpness, so success can be plotted against strength without recording
anything new.

Each lock is paired with its own outcome -- the state machine runs
lock -> sync -> header -> payload -> (F | H) -> reset, so the next F or H after
a P belongs to that P.

    frame    the header decoded
    sender   the 4-byte sender reads !4358ab2c exactly (32 bits; not chance)
    decrypt  cryptographic confirmation

A steep rise with sharpness means noise-limited: the decoder works and the
device is short of signal. A flat curve means strength is not what decides,
and something structural is still wrong.
"""
import sys

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

TRUE_SENDER = 0x4358AB2C


def unlog(b):
    return 2.0 ** (b / 25.0)


def locks(path, chans):
    """[(sharpness, got_frame, sender_ok, decrypted)] one per lock."""
    out = []
    pending = None            # sharpness of the lock awaiting an outcome
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 2:
                continue
            tag = p[1]
            if tag == "P" and len(p) > 2:
                raw = bytes.fromhex(p[2])
                if pending is not None:      # died before header search
                    out.append((pending, False, False, False, False))
                pending = unlog(raw[4]) if len(raw) >= 5 else None
            elif tag == "H":
                # Reached header search: sync was found, so this really was a
                # LoRa preamble followed by a sync word -- not a carrier.
                if pending is not None:
                    out.append((pending, False, False, False, True))
                    pending = None
            elif tag == "F" and len(p) > 2:
                ok = dec = False
                try:
                    rec = packets.process(bytes.fromhex(p[2]), chans)
                    if rec:
                        ok = (rec.get("sender") == TRUE_SENDER)
                        dec = bool(rec.get("decrypted"))
                except Exception:
                    pass
                if pending is not None:
                    out.append((pending, True, ok, dec, True))
                    pending = None
    if pending is not None:
        out.append((pending, False, False, False, False))
    return out


def main():
    chans = keys.load(keys.DEFAULT_PATH)
    rows = []
    for path in sys.argv[1:]:
        r = locks(path, chans)
        rows += r
        print(f"{path.split('/')[-1]:<22} {len(r):>4} locks")
    if not rows:
        print("no locks with a strength byte -- is the new build in place?")
        return 1

    rows.sort(key=lambda x: x[0])
    print(f"\n{len(rows)} locks total, sharpness "
          f"{rows[0][0]:.0f} to {rows[-1][0]:.0f}\n")

    # Equal-count bins: the distribution is skewed, so fixed-width bins would
    # put almost everything in the lowest bucket.
    NB = 6
    per = max(1, len(rows) // NB)
    print(f"  {'sharpness':>16} {'n':>5} {'sync':>6} {'frame|sync':>11} "
          f"{'sender':>7} {'decrypt':>8}")
    print("  " + "-" * 60)
    for i in range(0, len(rows), per):
        chunk = rows[i:i + per]
        if len(chunk) < per // 2 and i > 0:
            break
        lo, hi = chunk[0][0], chunk[-1][0]
        n = len(chunk)
        synced = [c for c in chunk if c[4]]
        sy = len(synced) / n
        # Conditional on reaching header search: this is the number that
        # answers "given a real packet, does more signal decode it?"
        fcond = (sum(1 for c in synced if c[1]) / len(synced)) if synced else 0.0
        sd = sum(1 for c in chunk if c[2]) / n
        dc = sum(1 for c in chunk if c[3]) / n
        bar = "#" * int(round(fcond * 30))
        print(f"  {lo:>7.0f}-{hi:<8.0f} {n:>5} {sy:>5.0%} {fcond:>10.1%} "
              f"{sd:>6.1%} {dc:>7.1%}  {bar}")

    lowq = rows[:len(rows) // 4]
    highq = rows[-len(rows) // 4:]
    sl_ = [c for c in lowq if c[4]]
    sh2 = [c for c in highq if c[4]]
    fl = (sum(1 for c in sl_ if c[1]) / len(sl_)) if sl_ else 0
    fh = (sum(1 for c in sh2 if c[1]) / len(sh2)) if sh2 else 0
    print(f"\n  reached header search: bottom {len(sl_)}/{len(lowq)}, "
          f"top {len(sh2)}/{len(highq)}")
    sl = sum(1 for c in lowq if c[2]) / len(lowq)
    sh_ = sum(1 for c in highq if c[2]) / len(highq)
    print(f"  bottom quartile: frame|sync {fl:.1%}, sender {sl:.1%}")
    print(f"  top quartile:    frame|sync {fh:.1%}, sender {sh_:.1%}")
    print()
    if fh > 2 * max(fl, 0.01):
        print("  => STRONGLY SNR-DEPENDENT. The decoder works and the limit is")
        print("     signal. Siting and antenna are the lever, not more DSP.")
    elif fh > 1.3 * max(fl, 0.01):
        print("  => partly SNR-dependent, but far from a clean threshold --")
        print("     something structural is still costing frames at good SNR.")
    else:
        print("  => NOT SNR-dependent. Strong packets fail about as often as")
        print("     weak ones, so more signal will not fix this.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
