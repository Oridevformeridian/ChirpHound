"""Where do the byte errors fall? Drift and noise look different.

Every decoded frame carries eight bytes whose truth is known exactly -- a
beacon is a broadcast from the Heltec, so dest is ffffffff and sender is
!4358ab2c little-endian. That is a free error pattern on every frame, with no
extra instrumentation and no firmware change.

The shape of it discriminates between the remaining hypotheses:

  * accuracy falling with byte position  -> something drifts over the packet
    (sample-clock offset, accumulating timing error); the fix is tracking
  * accuracy flat across position        -> a constant per-symbol error rate;
    the fix is symbol quality (CFO, sub-chip timing, peak picking)
  * errors clustered in specific bits    -> a systematic mapping bug
    (rotation, bit order, interleaver) rather than anything analogue

Run against a hostsim or device log:
    python3 err_shape.py hostsim/run_f2_baseline.txt
"""
import collections
import sys

TRUTH = bytes.fromhex("ffffffff2cab5843")   # dest ffffffff, sender !4358ab2c


def frames(path):
    out = []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            for i, tok in enumerate(p):
                if tok == "F" and i + 1 < len(p):
                    try:
                        out.append(bytes.fromhex(p[i + 1]))
                    except ValueError:
                        pass
                    break
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "hostsim/run_f2_baseline.txt"
    fs = [f for f in frames(path) if len(f) >= len(TRUTH)]
    if not fs:
        print("no frames")
        return 1

    print(f"{len(fs)} frames, comparing the first {len(TRUTH)} bytes "
          f"against the known beacon header\n")

    # 1. accuracy by byte position
    print("byte position accuracy (drift would show as a downward slope)")
    print(f"  {'pos':>3} {'truth':>6} {'right':>7} {'rate':>7}   histogram")
    slope = []
    for i, want in enumerate(TRUTH):
        hits = sum(1 for f in fs if f[i] == want)
        rate = hits / len(fs)
        slope.append(rate)
        bar = "#" * int(round(rate * 40))
        print(f"  {i:>3} {want:>6x} {hits:>7} {rate:>6.1%}   {bar}")

    first_half = sum(slope[:4]) / 4
    second_half = sum(slope[4:]) / 4
    print(f"\n  bytes 0-3 mean {first_half:.1%}, bytes 4-7 mean {second_half:.1%}"
          f"   delta {(second_half-first_half)*100:+.1f} pts")

    # 2. bit-level: how wrong is a wrong byte?
    #    One flipped bit points at FEC failing on a single symbol error; a
    #    uniformly random byte points at the symbol being lost entirely.
    print("\nwhen a byte is wrong, how many bits differ?")
    bits = collections.Counter()
    for f in fs:
        for i, want in enumerate(TRUTH):
            if f[i] != want:
                bits[bin(f[i] ^ want).count("1")] += 1
    total_wrong = sum(bits.values())
    for n in sorted(bits):
        share = bits[n] / total_wrong
        print(f"  {n} bit(s): {bits[n]:>5}  {share:>6.1%}  "
              f"{'#' * int(round(share * 40))}")
    mean_bits = sum(n * c for n, c in bits.items()) / max(total_wrong, 1)
    print(f"\n  mean {mean_bits:.2f} bits wrong per wrong byte "
          f"(4.0 = uniformly random)")

    # 3. bit-position map.
    #    Analogue noise hits every bit alike. An interleaver or rotation bug
    #    hits the SAME bit positions on every frame, because the mapping from
    #    symbol to bit is fixed. This separates them outright.
    print("\nerror rate per bit position (row = byte, col = bit 7..0)")
    print(f"  {'pos':>3}  " + " ".join(f"b{b}" for b in range(7, -1, -1)))
    col_tot = [0] * 8
    for i, want in enumerate(TRUTH):
        cells = []
        for b in range(7, -1, -1):
            wrong = sum(1 for f in fs if ((f[i] >> b) & 1) != ((want >> b) & 1))
            rate = wrong / len(fs)
            col_tot[7 - b] += wrong
            cells.append(f"{int(round(rate*9)) if rate else '.':>2}")
        print(f"  {i:>3}  " + " ".join(cells))
    print("\n  column totals (0-9 scale, . = never wrong):")
    n = len(fs) * len(TRUTH)
    print(f"  {'all':>3}  " + " ".join(
        f"{int(round(c/n*9)) if c else '.':>2}" for c in col_tot))
    print("\n  A bit that is wrong on nearly every frame is a mapping fault;")
    print("  noise would spread evenly across all eight columns.")

    # 4. is any single frame nearly right?
    best = max(fs, key=lambda f: sum(1 for a, b in zip(TRUTH, f) if a == b))
    hits = sum(1 for a, b in zip(TRUTH, best) if a == b)
    print(f"\nbest frame: {hits}/{len(TRUTH)} bytes  {best[:len(TRUTH)].hex()}")
    print(f"      truth: {'':>{len(TRUTH)}} {TRUTH.hex()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
