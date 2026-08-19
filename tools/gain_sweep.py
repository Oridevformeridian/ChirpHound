"""Score decode quality per RX gain config. Each F line is followed by a
G <amp> <lna> <vga> line tagging the live gain, so group frames by config."""
import re
import sys
sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

TRUTH = bytes.fromhex("ffffffff2cab5843")
chans = keys.load(keys.DEFAULT_PATH)

groups = {}
pend = None
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) > 2 and p[1] == "F":
        pend = p[2]
    elif len(p) >= 5 and p[1] == "G" and pend is not None:
        cfg = (int(p[2]), int(p[3]), int(p[4]))  # amp, lna, vga
        groups.setdefault(cfg, []).append(pend)
        pend = None

print(f"{'amp':>3} {'lna':>3} {'vga':>3} {'N':>3} {'hdrbytes':>9} "
      f"{'perfect':>8} {'decrypt':>8} {'clean':>6}")
print("-" * 52)
rows = []
for cfg in sorted(groups, key=lambda c: -len(groups[c])):
    fr = groups[cfg]
    n = len(fr)
    hb = perfect = dec = clean = 0
    tot_bytes = 0
    for hx in fr:
        try:
            b = bytes.fromhex(hx)
        except ValueError:
            continue
        if len(b) >= len(TRUTH):
            right = sum(1 for a, c in zip(TRUTH, b) if a == c)
            hb += right
            tot_bytes += len(TRUTH)
            if right == len(TRUTH):
                perfect += 1
        try:
            rec = packets.process(b, chans)
        except Exception:
            rec = None
        if rec and rec.get("decrypted"):
            dec += 1
            if re.fullmatch(r"BEACON \d{5}", str(rec.get("text") or "")):
                clean += 1
    pct = (100.0 * hb / tot_bytes) if tot_bytes else 0
    print(f"{cfg[0]:>3} {cfg[1]:>3} {cfg[2]:>3} {n:>3} {pct:>8.1f}% "
          f"{perfect:>8} {dec:>8} {clean:>6}")
    rows.append((cfg, n, pct))

# Highlight: does header accuracy trend with VGA (the swept axis)?
print("\nby VGA (amp off, lna 40):")
vga_rows = sorted((c[2], n, pct) for c, n, pct in rows if c[0] == 0 and c[1] == 40)
for vga, n, pct in vga_rows:
    print(f"  vga {vga:>2}: {pct:>5.1f}% hdr  (n={n})")
