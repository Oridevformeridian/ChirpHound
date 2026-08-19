"""Fit a slope to each preamble-bin sequence: is the sample clock drifting?

Each B line is the raw FFT bin of the consecutive preamble symbols at one lock.
A preamble is identical symbols, so a flat sequence means the sampling point
held still and a sloped one means it walked -- sample-frequency offset (SFO).
The slope is bins per symbol; at SF7/BW500 one symbol is 128 chips, so a slope
of s bins/symbol is s/128 of a chip of clock error per symbol.

Reported: the distribution of per-lock slopes, and the mean |slope|. Comparing
device against host (HackRF IQ, known near-flat) says whether the PortaPack's
clock is the culprit and by how much.

    python3 drift.py devlogs/run.TXT [label]
"""
import sys


def bins_per_lock(path):
    out = []
    for line in open(path):
        p = line.split()
        if len(p) > 2 and p[1] == "B":
            try:
                out.append(list(bytes.fromhex(p[2])))
            except ValueError:
                pass
    return out


def unwrap(seq, n=128):
    """Undo mod-128 wrap so a drift across the 127/0 seam reads as a line."""
    out = [seq[0]]
    for v in seq[1:]:
        prev = out[-1] % n
        step = ((v - prev + n // 2) % n) - n // 2
        out.append(out[-1] + step)
    return out


def slope(seq):
    """Least-squares slope of index -> unwrapped bin, in bins/symbol."""
    y = unwrap(seq)
    m = len(y)
    xs = list(range(m))
    mx = sum(xs) / m
    my = sum(y) / m
    num = sum((x - mx) * (v - my) for x, v in zip(xs, y))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den else 0.0


def main():
    path = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else path.split("/")[-1]
    locks = [b for b in bins_per_lock(path) if len(b) >= 6]
    if not locks:
        print(f"{label}: no B lines")
        return 1

    slopes = [slope(b) for b in locks]
    aslopes = sorted(abs(x) for x in slopes)
    mean_abs = sum(aslopes) / len(aslopes)
    median_abs = aslopes[len(aslopes) // 2]

    print(f"{label}: {len(locks)} locks with a preamble history")
    print(f"  mean |slope|   {mean_abs:.3f} bins/symbol")
    print(f"  median |slope| {median_abs:.3f}")
    print(f"  90th pct       {aslopes[int(0.9*len(aslopes))]:.3f}")
    # A whole chip of drift over an 8-symbol header (positions where decay was
    # measured) needs slope ~ 128/8 = 16 to move a full bin per symbol; but
    # even ~0.5 bins/symbol accumulates several bins across a header and would
    # corrupt late symbols. Flag the fraction above that.
    frac = sum(1 for x in aslopes if x >= 0.5) / len(aslopes)
    print(f"  fraction |slope| >= 0.5: {frac:.0%}")

    # A few examples spanning the range.
    ranked = sorted(zip((abs(x) for x in slopes), locks), key=lambda t: t[0])
    print("\n  flattest / typical / steepest preamble:")
    for idx in (0, len(ranked) // 2, -1):
        a, seq = ranked[idx]
        print(f"    slope {a:+.2f}  {seq}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
