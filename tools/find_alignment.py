"""Find where the LoRa header really starts, from raw captured symbols.

The device logs 32 symbols per preamble lock (`C <hex>`). Somewhere in that
run are the tail of the preamble, the sync word, the 2.25-symbol SFD and then
the 8-symbol header. On-device SFD detection has been unreliable, so rather
than guess the offset and reflash per hypothesis, try every one here.

A header is accepted only if it looks like a real Meshtastic header:
  * coding rate 1  (Meshtastic uses 4/5; 0 and 5-7 are invalid outright)
  * payload length in a plausible range for this mesh
A hit at a consistent offset across several captures is the answer; a hit at a
different offset each time is coincidence, and is reported as such.
"""
import argparse
import collections
import re
import sys

N = 128
SF = 7


def gray_demap(s):
    return s ^ (s >> 1)


def deinterleave(sym, sf_app, cw_len):
    out = [0] * sf_app
    for i in range(cw_len):
        for j in range(sf_app):
            if (sym[i] >> (sf_app - 1 - j)) & 1:
                out[(i - j - 1) % sf_app] |= 1 << (cw_len - 1 - i)
    return out


def hamming_decode(cw, cr_app, msb_first):
    ln = cr_app + 4
    if msb_first:
        c = [(cw >> (ln - 1 - i)) & 1 for i in range(ln)] + [0] * (8 - ln)
    else:
        c = [(cw >> i) & 1 for i in range(8)]
    nib = (c[3] << 3) | (c[2] << 2) | (c[1] << 1) | c[0]
    if cr_app in (3, 4):
        s0 = c[0] ^ c[1] ^ c[2] ^ c[4]
        s1 = c[1] ^ c[2] ^ c[3] ^ c[5]
        s2 = c[0] ^ c[1] ^ c[3] ^ c[6]
        nib ^= {5: 1, 7: 2, 3: 4, 6: 8}.get(s0 | (s1 << 1) | (s2 << 2), 0)
    return nib & 0x0F



def header_checksum_ok(n):
    """LoRa's 5-bit explicit-header checksum, verbatim from gr-lora_sdr.

    Without this, header validation is far too permissive: "coding rate 1 and
    a plausible length" passes by chance about 1 trial in 10, and an offset
    search runs ~96 trials per capture, so it reported ten candidate offsets
    and picked the wrong one. The checksum makes acceptance ~32x more
    selective, which is the difference between a search and a coin toss.
    """
    c4 = ((n[0] & 0b1000) >> 3) ^ ((n[0] & 0b0100) >> 2) ^ ((n[0] & 0b0010) >> 1) ^ (n[0] & 1)
    c3 = ((n[0] & 0b1000) >> 3) ^ ((n[1] & 0b1000) >> 3) ^ ((n[1] & 0b0100) >> 2) ^ ((n[1] & 0b0010) >> 1) ^ (n[2] & 1)
    c2 = ((n[0] & 0b0100) >> 2) ^ ((n[1] & 0b1000) >> 3) ^ (n[1] & 1) ^ ((n[2] & 0b1000) >> 3) ^ ((n[2] & 0b0010) >> 1)
    c1 = ((n[0] & 0b0010) >> 1) ^ ((n[1] & 0b0100) >> 2) ^ (n[1] & 1) ^ ((n[2] & 0b0100) >> 2) ^ ((n[2] & 0b0010) >> 1) ^ (n[2] & 1)
    c0 = (n[0] & 1) ^ ((n[1] & 0b0010) >> 1) ^ ((n[2] & 0b1000) >> 3) ^ ((n[2] & 0b0100) >> 2) ^ ((n[2] & 0b0010) >> 1) ^ (n[2] & 1)
    want = ((n[3] & 1) << 4) + n[4]
    return want == ((c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0)

def try_header(sym8, offset_minus1, msb_first, div4=True, base=0, rot=0):
    """base: the preamble bin, subtracted to remove the alignment/CFO bias.

    Symbol values are relative to the preamble, which is symbol 0 by
    definition. The captures prove it: subtracting the preamble bin turns the
    two sync symbols into exactly 0x10 and 0x58 -- LoRa's encoding of sync
    word 0x2B -- in every capture. Without this every symbol carries a
    constant bias and nothing downstream can decode.
    """
    gray = []
    for s in sym8:
        s = (s - base - rot) % N
        v = (s - 1) % N if offset_minus1 else s
        gray.append(gray_demap(v // 4 if div4 else v))
    cw = deinterleave(gray, SF - 2, 8)
    nib = [hamming_decode(c, 4, msb_first) for c in cw]
    ln = (nib[0] << 4) | nib[1]
    cr = (nib[2] & 0x0E) >> 1
    crc = bool(nib[2] & 0x01)
    return nib, ln, cr, crc


def main():
    ap = argparse.ArgumentParser(description="search for header alignment")
    ap.add_argument("log")
    ap.add_argument("--min-len", type=int, default=8)
    ap.add_argument("--max-len", type=int, default=240)
    args = ap.parse_args()

    caps = []
    for line in open(args.log, errors="replace"):
        m = re.search(r"\bC\s+([0-9a-fA-F]{16,})", line)
        if m:
            h = m.group(1)
            caps.append([int(h[i:i + 2], 16) for i in range(0, len(h), 2)])
    if not caps:
        print("no C (capture) lines found", file=sys.stderr)
        return 1
    print(f"{len(caps)} capture(s), {len(caps[0])} symbols each\n")

    hits = collections.Counter()
    detail = collections.defaultdict(list)
    for idx, sym in enumerate(caps):
        # The preamble bin is whatever value the leading run repeats.
        base = collections.Counter(sym[:10]).most_common(1)[0][0]
        for off in range(0, len(sym) - 8 + 1):
            for minus1 in (False, True):
              for msb in (False, True):
                # The SFD's trailing quarter symbol rotates every later bin by
                # N/4. Search those rotations rather than assume none.
                for rot in (0, N // 4, N // 2, 3 * N // 4):
                    nib, ln, cr, crc = try_header(
                        sym[off:off + 8], minus1, msb, base=base, rot=rot)
                    if (header_checksum_ok(nib) and cr == 1
                            and args.min_len <= ln <= args.max_len):
                        key = (off, minus1, msb, rot)
                        hits[key] += 1
                        detail[key].append((idx, ln, crc))

    if not hits:
        print("No offset produced a valid header in any capture.")
        print("That points upstream of the decode chain -- alignment or the")
        print("dechirp itself -- rather than at bit order or the -1 correction.")
        return 2

    print(f"{'offset':>6} {'-1':>5} {'bits':>5} {'rot':>4} {'hits':>5}  lengths")
    print("-" * 58)
    for key, n in hits.most_common(12):
        off, minus1, msb, rot = key
        lens = ",".join(str(d[1]) for d in detail[key][:6])
        print(f"{off:>6} {str(minus1):>5} {'msb' if msb else 'lsb':>5} "
              f"{rot:>4} {n:>5}  {lens}")

    best, n = hits.most_common(1)[0]
    print(f"\nbest: offset {best[0]}, minus1={best[1]}, "
          f"{'msb' if best[2] else 'lsb'}-first, rot={best[3]}")
    if n < max(2, len(caps) // 2):
        print("WARNING: that offset does not recur across captures -- likely "
              "coincidence, not the true alignment.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
