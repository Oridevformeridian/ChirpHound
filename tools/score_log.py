"""Turn a PortaPack capture log into a number, so builds can be compared.

Without this there is no feedback loop on the device half: a build is judged by
eyeballing a log, which cannot distinguish "better" from "different run". Every
gain on this project came from a measurement, and the one measurement missing
is the one that scores the firmware itself.

The device emits one tagged line per event:

    P <hex>   preamble locked      (bin, run, detection count)
    C <hex>   raw symbols          (diagnostic dumps)
    H <syms>  header search failed (no offset passed the checksum)
    T         SFD never arrived after a lock
    E <nib>   header decode failed (nibbles, plus S <symbols>)
    F <hex>   frame decoded

So the funnel is P -> E|F, and the ratios that matter are:

    lock rate      P per minute of capture -- is the front end hearing anything
    header rate    F / (E + F) -- of the packets we locked, how many decoded
    yield          F / P -- end to end

Compare two logs to see whether a change helped, and by how much. Counts alone
mislead: a build that locks twice as often and decodes the same fraction looks
better on F but has not improved the decoder at all, so header rate is reported
separately from yield.

    python3 score_log.py baseline_720549d3.txt
    python3 score_log.py new.txt --against baseline_720549d3.txt
"""
import argparse
import collections
import sys

TAGS = ("P", "C", "E", "F", "H", "T")


def parse(path):
    """Count tagged events. Tolerates junk: the device shares the log with
    other output and a corrupt line must not abort a run's score."""
    counts = collections.Counter()
    frames, bad = [], 0
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            parts = line.split()
            hit = False
            for i, tok in enumerate(parts):
                # T is the one tag with no payload -- it means "locked but the
                # SFD never arrived". Requiring a following token silently
                # classified every one of them as junk, which mattered the
                # moment a real device log was scored.
                if tok == "T":
                    counts["T"] += 1
                    hit = True
                    break
                if tok in TAGS and i + 1 < len(parts):
                    counts[tok] += 1
                    if tok == "F":
                        frames.append(parts[i + 1])
                    hit = True
                    break
            if not hit and line.strip():
                bad += 1
    counts["junk"] = bad
    return counts, frames


def rates(c):
    p, e, f = c["P"], c["E"], c["F"]
    attempts = e + f + c["H"]
    return {
        "locks": p,
        "header_attempts": attempts,
        "frames": f,
        "header_rate": (f / attempts) if attempts else 0.0,
        "yield": (f / p) if p else 0.0,
    }


# Every beacon is a broadcast from the Heltec, so the first eight bytes of a
# correct decode are fixed: dest ffffffff, sender !4358ab2c little-endian.
# Confirmed against 1412 decrypted beacons in the meshpipe store.
BEACON_PREFIX = "ffffffff2cab5843"


def prefix_accuracy(frames, expect=BEACON_PREFIX):
    """Mean fraction of known header bytes each frame gets right.

    Pass/fail decryption is the honest end goal but far too coarse to steer by:
    at the current yield a run holds a couple of frames, so the score is 0 or 1
    and a real improvement is invisible until it is total. Byte accuracy over a
    prefix we know exactly moves smoothly, which is what a feedback loop needs.

    Frames from other senders drag this down legitimately -- it is a fleet
    average over a channel that is mostly beacons, not a per-frame verdict.
    """
    want = bytes.fromhex(expect)
    scores = []
    exact = 0
    for hx in frames:
        try:
            got = bytes.fromhex(hx)
        except ValueError:
            continue
        if len(got) < len(want):
            scores.append(0.0)
            continue
        hits = sum(1 for a, b in zip(want, got) if a == b)
        scores.append(hits / len(want))
        if hits == len(want):
            exact += 1
    return (sum(scores) / len(scores) if scores else 0.0), exact, len(scores)


def frame_sanity(frames):
    """A decoded frame should look like a Meshtastic header, not noise.

    Cheap structural check only -- full parsing lives in meshpipe.replay, and
    duplicating it here would couple the scorer to the channel keys. What this
    catches is a build that emits frames which are the right length but junk,
    which otherwise scores as an improvement.
    """
    ok = 0
    for hx in frames:
        try:
            b = bytes.fromhex(hx)
        except ValueError:
            continue
        # dest(4) src(4) id(4) flags(1) hash(1) = 14 byte header minimum
        if len(b) >= 14:
            ok += 1
    return ok


def show(name, c, f):
    r = rates(c)
    print(f"\n{name}")
    print("-" * len(name))
    print(f"  preamble locks      {r['locks']:>6}")
    print(f"  header attempts     {r['header_attempts']:>6}   "
          f"(E {c['E']} + H {c['H']} + F {c['F']})")
    print(f"  frames decoded      {r['frames']:>6}")
    print(f"  symbol dumps        {c['C']:>6}")
    # H and T separate the two ways a lock dies, which need opposite fixes:
    # H is a decode problem, T is an alignment problem.
    print(f"  header search fail  {c['H']:>6}   H (no offset passed checksum)")
    print(f"  sfd timeouts        {c['T']:>6}   T (locked, SFD never came)")
    print(f"  header rate         {r['header_rate']:>6.1%}   F/(E+F)")
    print(f"  yield               {r['yield']:>6.1%}   F/P")
    if f:
        print(f"  frames >= 14B       {frame_sanity(f):>6}/{len(f)}")
        acc, exact, n = prefix_accuracy(f)
        print(f"  header bytes right  {acc:>6.1%}   vs known beacon prefix "
              f"({n} frames)")
        print(f"  perfect headers     {exact:>6}/{n}")
    if c["junk"]:
        print(f"  unparsed lines      {c['junk']:>6}")
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log")
    ap.add_argument("--against", help="baseline log to compare against")
    args = ap.parse_args()

    c, f = parse(args.log)
    new = show(args.log.split("/")[-1], c, f)

    if not args.against:
        return 0

    bc, bf = parse(args.against)
    base = show(args.against.split("/")[-1], bc, bf)

    print("\ndelta")
    print("-----")
    for key, label, pct in (("locks", "preamble locks", False),
                            ("frames", "frames decoded", False),
                            ("header_rate", "header rate", True),
                            ("yield", "yield", True)):
        b, n = base[key], new[key]
        if pct:
            print(f"  {label:<18} {b:>7.1%} -> {n:>7.1%}   "
                  f"{'+' if n >= b else ''}{(n-b)*100:>5.1f} pts")
        else:
            print(f"  {label:<18} {b:>7} -> {n:>7}   "
                  f"{'+' if n >= b else ''}{n-b:>5}")

    # The honest caveat: two captures are two different sets of packets.
    print("\nNote: different runs hear different traffic. Header rate is the")
    print("comparable figure; lock count mostly reflects what was on the air.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
