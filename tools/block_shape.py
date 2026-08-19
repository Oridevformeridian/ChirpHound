"""Is the first payload block worse, or is one lucky byte faking it?

The byte-position view suggested bytes 0-3 decode worse than 4-7, which would
point at the first payload block. But bytes are the wrong unit: LoRa decodes
in blocks of cw_len symbols that produce sf_app nibbles, so at SF7/CR4/5 a
block is 5 symbols -> 7 nibbles = 3.5 bytes, and every block boundary lands
mid-byte. Grouping by byte smears two blocks together.

This regroups the same known-truth bytes into nibbles, assigns each nibble to
its block, and reports per-block accuracy -- the unit the decoder actually
works in. It also reports the byte gap with the single best byte excluded,
because one easy byte can manufacture a trend across eight samples.
"""
import sys

TRUTH = bytes.fromhex("ffffffff2cab5843")
SF_APP = 7          # LDRO off at SF7
CW_LEN = 5          # CR 4/5
NIB_PER_BLOCK = SF_APP


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
    return [f for f in out if len(f) >= len(TRUTH)]


def nibbles(b):
    out = []
    for byte in b:
        out.append((byte >> 4) & 0xF)     # high nibble first
        out.append(byte & 0xF)
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "hostsim/run_minus1.txt"
    fs = frames(path)
    if not fs:
        print("no frames")
        return 1
    want = nibbles(TRUTH)
    n_known = len(want)

    print(f"{len(fs)} frames, {n_known} known nibbles "
          f"(block = {NIB_PER_BLOCK} nibbles from {CW_LEN} symbols)\n")

    print(f"  {'nib':>3} {'blk':>3} {'truth':>5} {'rate':>7}  histogram")
    per_block = {}
    for i in range(n_known):
        blk = i // NIB_PER_BLOCK
        hits = sum(1 for f in fs if nibbles(f)[i] == want[i])
        rate = hits / len(fs)
        per_block.setdefault(blk, []).append(rate)
        print(f"  {i:>3} {blk:>3} {want[i]:>5x} {rate:>6.1%}  "
              f"{'#' * int(round(rate * 40))}")

    print("\nper block (the unit the decoder works in):")
    for blk in sorted(per_block):
        r = per_block[blk]
        cover = "complete" if len(r) == NIB_PER_BLOCK else f"{len(r)} nibbles only"
        print(f"  block {blk}: {sum(r)/len(r):>6.1%}   ({cover})")

    # Byte view with the best byte removed -- one outlier across eight samples
    # is enough to invent a trend.
    rates = []
    for i, w in enumerate(TRUTH):
        rates.append(sum(1 for f in fs if f[i] == w) / len(fs))
    best_i = max(range(len(rates)), key=lambda i: rates[i])
    lo = [rates[i] for i in range(4) if i != best_i]
    hi = [rates[i] for i in range(4, 8) if i != best_i]
    print(f"\nbyte view: 0-3 {sum(rates[:4])/4:.1%} vs 4-7 {sum(rates[4:])/4:.1%}"
          f"   (gap {(sum(rates[4:])/4 - sum(rates[:4])/4)*100:+.1f} pts)")
    print(f"  excluding byte {best_i} (the best, {rates[best_i]:.1%}): "
          f"{sum(lo)/len(lo):.1%} vs {sum(hi)/len(hi):.1%}   "
          f"gap {(sum(hi)/len(hi) - sum(lo)/len(lo))*100:+.1f} pts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
