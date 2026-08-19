"""Does a stronger signal lock EARLIER, and does that break the header search?

Strong packets decode worse, consistently across four captures. A mechanism
that would explain it without any appeal to RF: the detector declares a lock
once 6 consecutive symbols pass the sharpness gate, so a stronger packet
crosses that gate earlier in its preamble and locks at a different point in
the frame. Everything downstream is timed from the lock, and only 3 header
offsets are searched -- so a systematically different lock point would put the
header outside the search window.

Both quantities are already on every P line (preamble_run, sharpness); this
needs no new instrumentation and no new capture.
"""
import sys
sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

TRUE_SENDER = 0x4358AB2C


def rows(path, chans):
    out, pend = [], None
    for line in open(path):
        p = line.split()
        if len(p) < 2:
            continue
        if p[1] == "P" and len(p) > 2:
            raw = bytes.fromhex(p[2])
            if pend:
                out.append(pend + (False, False))
            pend = (raw[1], 2.0 ** (raw[4] / 25.0)) if len(raw) >= 5 else None
        elif p[1] == "H":
            if pend:
                out.append(pend + (False, False)); pend = None
        elif p[1] == "F" and len(p) > 2:
            ok = False
            try:
                rec = packets.process(bytes.fromhex(p[2]), chans)
                ok = bool(rec and rec.get("sender") == TRUE_SENDER)
            except Exception:
                pass
            if pend:
                out.append(pend + (True, ok)); pend = None
    if pend:
        out.append(pend + (False, False))
    return out


chans = keys.load(keys.DEFAULT_PATH)
all_rows = []
for path in sys.argv[1:]:
    all_rows += rows(path, chans)

print(f"{len(all_rows)} locks\n")

print("preamble_run vs signal strength")
print(f"  {'run':>4} {'n':>5} {'mean sharp':>11} {'frame':>7} {'sender':>7}")
by_run = {}
for r, sh, f, ok in all_rows:
    by_run.setdefault(r, []).append((sh, f, ok))
for r in sorted(by_run):
    v = by_run[r]
    if len(v) < 5:
        continue
    print(f"  {r:>4} {len(v):>5} {sum(x[0] for x in v)/len(v):>11.1f} "
          f"{sum(1 for x in v if x[1])/len(v):>6.1%} "
          f"{sum(1 for x in v if x[2])/len(v):>6.1%}")

# Is run confounded with strength? If strong packets lock at a different run,
# the two explanations are entangled and run is the one that is actionable.
strong = [x for x in all_rows if x[1] >= 72]
weak = [x for x in all_rows if x[1] < 40]
def mrun(v):
    return sum(x[0] for x in v) / len(v) if v else 0
print(f"\n  mean preamble_run: weak {mrun(weak):.2f} (n={len(weak)}), "
      f"strong {mrun(strong):.2f} (n={len(strong)})")
print("  if these differ, lock POINT -- not signal level -- is what varies")
