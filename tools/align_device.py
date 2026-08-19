"""Label every device reception with the beacon that caused it.

The device stamps its log from an RTC that was never set (1980), but the clock
runs at the right rate, so the offset to real time is a single constant. The
beacon logs each transmission with a real timestamp *and a sequence number*,
so recovering that one offset turns every lock into a labelled sample: which
transmission it was, and therefore what the payload should have been.

That is the ground truth this project has been missing on the device side.
Until now a device log could say "147 locks" but not "of the 108 beacons sent,
it heard these 97 and missed those 11" -- and the difference between missing a
transmission and mis-decoding one needs opposite fixes.

Offset is found the same way the two HackRFs were aligned: try candidates, keep
the one that matches the most events. Beacons are 10 s apart, so a 1 s log
resolution is ample and the match is unambiguous.

    python3 align_device.py devlogs/MESHTAST_fix.TXT beacons.txt
"""
import re
import sys
from datetime import datetime

# Must be well UNDER HALF the beacon spacing. At 3 s against a 5 s-spaced era
# every timestamp matches something, the search reports a perfect 147/147, and
# the "missed" list comes out as every odd sequence number -- a factor-of-two
# artefact that reads like a real result.
TOL = 2.0


def device_events(path):
    """[(t_seconds, tag)] from the device log."""
    out = []
    for line in open(path):
        p = line.split()
        if len(p) < 2 or not p[0].isdigit() or len(p[0]) != 14:
            continue
        try:
            t = datetime.strptime(p[0], "%Y%m%d%H%M%S").timestamp()
        except ValueError:
            continue
        out.append((t, p[1]))
    return out


def beacon_sends(path):
    """[(t_seconds, seq)] from the journal capture."""
    out = []
    pat = re.compile(r"(\w{3})\s+(\d+)\s+(\d{2}):(\d{2}):(\d{2}).*sent 'BEACON (\d+)'")
    months = {m: i + 1 for i, m in enumerate(
        "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec".split())}
    for line in open(path):
        m = pat.search(line)
        if not m:
            continue
        mon, day, hh, mm, ss, seq = m.groups()
        # Year is not in the journal's short format; these runs are 2026.
        t = datetime(2026, months[mon], int(day), int(hh), int(mm),
                     int(ss)).timestamp()
        out.append((t, int(seq)))
    return out


def main():
    dev = device_events(sys.argv[1])
    bcn = beacon_sends(sys.argv[2])
    if not dev or not bcn:
        print("missing data", file=sys.stderr)
        return 1

    locks = [t for t, tag in dev if tag == "P"]
    print(f"device: {len(dev)} events ({len(locks)} locks), "
          f"span {(max(locks)-min(locks))/60:.1f} min")
    print(f"beacon: {len(bcn)} sends, span {(bcn[-1][0]-bcn[0][0])/60:.1f} min")

    # Coarse then fine search for the constant clock offset.
    # Scan the WHOLE journal span, not a window around the first beacon.
    # Anchoring on bcn[0] and scanning +/-1 h could only ever align the device
    # to the earliest beacon era; this run happened hours later, under a
    # different interval, and the search found a false peak there instead
    # (54 beacons in an 18 min window = one per 20 s, i.e. the wrong era).
    bt = sorted(t for t, _ in bcn)
    lo_off = bt[0] - max(locks)
    hi_off = bt[-1] - min(locks)
    best, best_n = 0.0, -1
    step = 5
    for off in range(int(lo_off), int(hi_off) + 1, step):
        n = 0
        j = 0
        for t in sorted(locks):
            tt = t + off
            while j < len(bt) and bt[j] < tt - TOL:
                j += 1
            if j < len(bt) and abs(bt[j] - tt) <= TOL:
                n += 1
        if n > best_n:
            best_n, best = n, off
    # Refine to 1 s around the coarse winner.
    for off in range(int(best) - step, int(best) + step + 1):
        n = 0
        j = 0
        for t in sorted(locks):
            tt = t + off
            while j < len(bt) and bt[j] < tt - TOL:
                j += 1
            if j < len(bt) and abs(bt[j] - tt) <= TOL:
                n += 1
        if n > best_n:
            best_n, best = n, off
    print(f"\nclock offset: {best:.0f} s  ({best_n}/{len(locks)} locks matched "
          f"a transmission)")

    # Label each beacon that fell inside the device's listening window.
    lo, hi = min(locks) + best, max(locks) + best
    inwin = [(t, s) for t, s in bcn if lo - TOL <= t <= hi + TOL]
    print(f"{len(inwin)} beacons transmitted while the device was listening\n")

    heard = reached = framed = 0
    missed = []
    for t, seq in inwin:
        near = [(abs((dt + best) - t), tag) for dt, tag in dev
                if abs((dt + best) - t) <= TOL]
        tags = {tag for _, tag in near}
        if "P" in tags:
            heard += 1
            if tags & {"H", "F", "E"}:
                reached += 1
            if "F" in tags:
                framed += 1
        else:
            missed.append(seq)

    n = len(inwin)
    print(f"  locked on            {heard:>4}/{n}  ({100*heard/n:.0f}%)")
    print(f"  reached header       {reached:>4}/{n}  ({100*reached/n:.0f}%)")
    print(f"  produced a frame     {framed:>4}/{n}  ({100*framed/n:.0f}%)")
    print(f"\n  never detected: {len(missed)} beacons"
          + (f" (seq {missed[:12]}{'...' if len(missed) > 12 else ''})"
             if missed else ""))
    # Count LOCKS with no transmission, not locks-minus-beacons: several locks
    # can belong to one beacon, so the naive difference overstates false locks
    # badly (46 vs the real 8 on the first run of this).
    unmatched = 0
    for dt, tag in dev:
        if tag != "P":
            continue
        if not any(abs((dt + best) - t) <= TOL for t, _ in bcn):
            unmatched += 1
    print(f"  locks with no transmission: {unmatched} of {len(locks)} "
          f"({'other traffic or false locks' if unmatched else 'none'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
