"""Solve the symbol->byte mapping against a frame whose bytes are known.

This is no longer a guessing problem. The reference receiver decoded the same
beacon off the air, so the first eight bytes are known exactly:

    ff ff ff ff 2c ab 58 43        broadcast, sender !4358ab2c

Only the packet id and ciphertext vary between beacons, so those eight bytes
are a fixed 16-nibble constraint. Search the remaining unknowns -- where the
header starts, the bin rotation, bit order, the reference's mod(bin-1) -- and
accept only a combination that reproduces them. A wrong combination has about
a 16^-16 chance of matching, so a hit is the answer rather than a coincidence.
"""
import collections
import re
import sys

sys.path.insert(0, "/tmp/claude-1000/-var-home-mycroft/b220d75f-2477-4a02-b896-99d240179945/scratchpad")
from find_alignment import gray_demap, deinterleave, hamming_decode, header_checksum_ok, N

WHITEN = None  # filled from the firmware table below

TARGET = bytes.fromhex("ffffffff2cab5843")
SF = 7


def load_whitening():
    import pathlib
    txt = pathlib.Path("/tmp/claude-1000/-var-home-mycroft/b220d75f-2477-4a02-b896-99d240179945/scratchpad/whitening.txt").read_text()
    return [int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", txt)]


def decode_block(syms, base, rot, minus1, msb, sf_app, cw_len, div):
    gray = []
    for s in syms:
        v = (s - base - rot) % N
        if minus1:
            v = (v - 1) % N
        gray.append(gray_demap(v // div))
    cw = deinterleave(gray, sf_app, cw_len)
    return [hamming_decode(c, cw_len - 4, msb) for c in cw]


def attempt(sym, base, hoff, rot, minus1, msb):
    """Decode header then payload; return the first bytes if it all holds."""
    if hoff + 8 > len(sym):
        return None
    nib = decode_block(sym[hoff:hoff + 8], base, rot, minus1, msb,
                       SF - 2, 8, 4)
    if not header_checksum_ok(nib):
        return None
    ln = (nib[0] << 4) | nib[1]
    cr = (nib[2] & 0x0E) >> 1
    if cr != 1 or ln != 34:
        return None

    cw_len = cr + 4          # 5
    sf_app = SF              # LDRO off at SF7
    nibbles = []
    p = hoff + 8
    while p + cw_len <= len(sym) and len(nibbles) < 2 * (ln + 2):
        nibbles += decode_block(sym[p:p + cw_len], base, rot, minus1, msb,
                                sf_app, cw_len, 1)
        p += cw_len

    out = bytearray()
    for i in range(0, min(len(nibbles) - 1, 2 * len(TARGET)), 2):
        off = i // 2
        low = nibbles[i] ^ (WHITEN[off] & 0x0F)
        high = nibbles[i + 1] ^ ((WHITEN[off] & 0xF0) >> 4)
        out.append(((high << 4) | (low & 0x0F)) & 0xFF)
    return bytes(out[:len(TARGET)])


WHITEN = load_whitening()
caps = []
for line in open("/tmp/claude-1000/-var-home-mycroft/b220d75f-2477-4a02-b896-99d240179945/scratchpad/fullcap.txt",
                 errors="replace"):
    m = re.search(r"\bC\s+([0-9a-fA-F]{32,})", line)
    if m:
        h = m.group(1)
        caps.append([int(h[i:i + 2], 16) for i in range(0, len(h), 2)])

print(f"{len(caps)} captures, target {TARGET.hex()}\n")
solutions = collections.Counter()
partial = collections.Counter()

for ci, sym in enumerate(caps):
    base_guess = collections.Counter(sym[:12]).most_common(1)[0][0]
    for base in {base_guess, (base_guess + 1) % N, (base_guess - 1) % N}:
        for hoff in range(0, 40):
            for rot in range(N):
                for minus1 in (False, True):
                    for msb in (False, True):
                        got = attempt(sym, base, hoff, rot, minus1, msb)
                        if got is None:
                            continue
                        key = (hoff, rot, minus1, msb)
                        if got == TARGET:
                            solutions[key] += 1
                        elif got[:4] == TARGET[:4]:
                            partial[key] += 1

print(f"exact matches: {len(solutions)}")
for k, n in solutions.most_common(8):
    print(f"  hoff={k[0]:3d} rot={k[1]:3d} minus1={str(k[2]):5s} "
          f"{'msb' if k[3] else 'lsb'}   in {n} capture(s)")
print(f"\npartial (dest ffffffff only): {len(partial)}")
for k, n in partial.most_common(5):
    print(f"  hoff={k[0]:3d} rot={k[1]:3d} minus1={str(k[2]):5s} "
          f"{'msb' if k[3] else 'lsb'}   in {n} capture(s)")
