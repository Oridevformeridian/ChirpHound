"""Do symbols degrade with position WITHIN a packet on the device?

The H line is the header-search window: the symbols immediately after sync.
Beacons are near-identical transmissions, so at any given position the symbol
value should repeat across packets. Measuring how much it repeats, position by
position, separates two very different faults:

  flat across positions   -> a constant error rate; every symbol equally good
  rising with position    -> integrity is lost partway through the packet,
                             i.e. samples are being dropped or timing drifts
                             as the packet proceeds

The second cannot be reproduced by the host harness at all: it processes a file
with unlimited time per buffer, while the device has 256 us per symbol. If the
device decays and the host does not, the harness is blind to the device's real
bottleneck.

Metric: at each position, the frequency of the single most common value across
all H lines. High = consistent, ~1/128 = random.
"""
import collections
import sys


def windows(path):
    out = []
    for line in open(path):
        p = line.split()
        if len(p) > 2 and p[1] == "H":
            try:
                out.append(list(bytes.fromhex(p[2])))
            except ValueError:
                pass
    return out


def profile(path, label):
    w = [x for x in windows(path) if len(x) >= 11]
    if not w:
        print(f"{label}: no usable H lines")
        return
    print(f"\n{label}  ({len(w)} header windows)")
    print(f"  {'pos':>3} {'top value':>10} {'share':>7}  consistency")
    for i in range(11):
        c = collections.Counter(x[i] for x in w)
        val, n = c.most_common(1)[0]
        share = n / len(w)
        bar = "#" * int(round(share * 40))
        print(f"  {i:>3} {val:>10x} {share:>6.1%}  {bar}")
    early = sum(collections.Counter(x[i] for x in w).most_common(1)[0][1]
                for i in range(4)) / (4 * len(w))
    late = sum(collections.Counter(x[i] for x in w).most_common(1)[0][1]
               for i in range(7, 11)) / (4 * len(w))
    print(f"  positions 0-3 mean {early:.1%}, positions 7-10 mean {late:.1%}"
          f"   decay {(early-late)*100:+.0f} pts")
    return early, late


a = profile(sys.argv[1], "DEVICE")
b = profile(sys.argv[2], "HOST (same code, captured IQ)")
if a and b:
    print(f"\n  device decay {(a[0]-a[1])*100:+.0f} pts vs "
          f"host {(b[0]-b[1])*100:+.0f} pts")
    if (a[0] - a[1]) > 0.15 and (b[0] - b[1]) < 0.10:
        print("\n  => The device loses symbol integrity as a packet proceeds and")
        print("     the host does not. That is a REAL-TIME fault -- dropped")
        print("     buffers or drift -- and the host harness cannot see it,")
        print("     because it has unlimited time per buffer.")
