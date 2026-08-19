"""Does the discarded fractional timing error predict decode failure?

peak_bin measures a signed sub-bin timing error per symbol and process_symbol
drops it. Before spending RAM and cycles on a correction -- and after three
speculative changes that each made things worse -- establish that the quantity
actually separates good frames from bad.

Each frame's D line carries the mean |terr| over its payload symbols, emitted
immediately after the frame, so pairing is by position. Decryption is the
ground truth: cryptographic, so it cannot pass by accident.

    distrobox enter sdrbox -- python3 terr_corr.py hostsim/run_terr.txt

Reads: if decrypting frames have clearly lower |terr|, sub-bin timing is the
residual and correction is justified. If the distributions overlap, it is not,
and a timing loop would be the fourth speculative change in a row.
"""
import sys

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets


def pairs(path):
    """(frame_hex, mean_abs_terr, mean_signed_terr, n_symbols), in order."""
    out = []
    pending = None
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 2:
                continue
            tag = p[1]
            if tag == "F" and len(p) > 2:
                pending = p[2]
            elif tag == "D" and len(p) > 2 and pending is not None:
                raw = bytes.fromhex(p[2])
                if len(raw) >= 3:
                    mabs = raw[0] / 255.0
                    msig = (raw[1] / 255.0) * 2.0 - 1.0
                    # byte 3, when present, is mean sharpness on a log scale:
                    # 25 counts per doubling, so decode as 2**(enc/25).
                    sharp = 2.0 ** (raw[3] / 25.0) if len(raw) >= 4 else 0.0
                    out.append((pending, mabs, msig, raw[2], sharp))
                pending = None
    return out


def stats(xs):
    if not xs:
        return 0.0, 0.0
    m = sum(xs) / len(xs)
    var = sum((x - m) ** 2 for x in xs) / len(xs)
    return m, var ** 0.5


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "hostsim/run_terr.txt"
    rows = pairs(path)
    if not rows:
        print("no F/D pairs -- is the instrumented build in place?")
        return 1

    chans = keys.load(keys.DEFAULT_PATH)
    good, bad = [], []
    good_sh, bad_sh = [], []
    good_sig, bad_sig = [], []
    for hexs, mabs, msig, n, sharp in rows:
        try:
            rec = packets.process(bytes.fromhex(hexs), chans)
        except Exception:
            rec = None
        ok = bool(rec and rec.get("decrypted"))
        (good if ok else bad).append(mabs)
        (good_sig if ok else bad_sig).append(msig)
        (good_sh if ok else bad_sh).append(sharp)

    gm, gs = stats(good)
    bm, bs = stats(bad)
    print(f"{len(rows)} frames paired with a timing measurement")
    print(f"  decrypted    {len(good):>3}   mean |terr| {gm:.3f}  sd {gs:.3f}")
    print(f"  failed       {len(bad):>3}   mean |terr| {bm:.3f}  sd {bs:.3f}")

    if not good or not bad:
        print("\nneed both classes present to say anything")
        return 0

    # Separation in pooled standard deviations. Anything under ~0.5 is not a
    # discriminator you could gate on, whatever the means look like.
    pooled = (((len(good) - 1) * gs ** 2 + (len(bad) - 1) * bs ** 2) /
              max(len(good) + len(bad) - 2, 1)) ** 0.5
    d = (bm - gm) / pooled if pooled else 0.0
    print(f"\n  separation: {d:+.2f} pooled sd "
          f"({'failed frames have higher |terr|' if d > 0 else 'no, or inverted'})")

    ghm, ghs = stats(good_sh)
    bhm, bhs = stats(bad_sh)
    pooled_h = ((((len(good_sh) - 1) * ghs ** 2 + (len(bad_sh) - 1) * bhs ** 2) /
                 max(len(good_sh) + len(bad_sh) - 2, 1)) ** 0.5)
    dh = (ghm - bhm) / pooled_h if pooled_h else 0.0
    print(f"\n  mean SHARPNESS: decrypted {ghm:.1f}, failed {bhm:.1f}"
          f"   separation {dh:+.2f} pooled sd")
    print("    sharpness is peak/mean of the FFT, i.e. an SNR proxy; the")
    print("    detection gate is 8. Strong separation => the limit is RF,")
    print("    not DSP, and no restructuring will recover these frames.")

    gsm, _ = stats(good_sig)
    bsm, _ = stats(bad_sig)
    print(f"  mean SIGNED terr: decrypted {gsm:+.3f}, failed {bsm:+.3f}")
    print("    a consistent non-zero sign is a fixed offset (correctable);")
    print("    near zero with large |terr| is jitter (needs per-symbol work)")

    # Would gating on |terr| have worked? A threshold test is the honest way to
    # ask whether this is actionable rather than merely correlated.
    allr = sorted(set(round(x, 3) for x in good + bad))
    best = None
    for t in allr:
        kept_g = sum(1 for x in good if x <= t)
        kept_b = sum(1 for x in bad if x <= t)
        if kept_g + kept_b == 0:
            continue
        prec = kept_g / (kept_g + kept_b)
        if kept_g >= max(1, len(good) // 2) and (best is None or prec > best[1]):
            best = (t, prec, kept_g, kept_b)
    if best:
        t, prec, kg, kb = best
        print(f"\n  best threshold |terr| <= {t:.3f}: keeps {kg}/{len(good)} good "
              f"and {kb}/{len(bad)} bad ({prec:.0%} precision)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
