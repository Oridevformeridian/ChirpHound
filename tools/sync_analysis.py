"""Why does the sync hunt fail? Absent, mistimed, or offset?

Each S line is the sequence of rel = (bin - preamble_bin) mod N values seen
while hunting 0x10 then 0x58. Three failure modes need opposite fixes:

  absent    no pair resembling the sync word at any shift -> the lock was not
            on a real preamble, or the symbols are too corrupted to use
  mistimed  the pair is present at shift 0 but the state machine walked past
            it -> a logic or tolerance problem, fixable for free
  offset    the pair is present at a consistent shift d != 0 -> preamble_bin is
            biased by d, and correcting that recovers every one of these

The third is the interesting one, and it is testable: for each trace, search
for the shift d that makes some symbol land on 0x10+d and a later one on
0x58+d. A consistent non-zero d across traces is a bias, not a coincidence.
"""
import collections
import sys

SYNC1, SYNC2 = 0x10, 0x58
N = 128
TOL = 2


def traces(path):
    out = []
    for line in open(path):
        p = line.split()
        if len(p) > 2 and p[1] == "S":
            try:
                out.append(list(bytes.fromhex(p[2])))
            except ValueError:
                pass
    return out


def find_shift(tr, tol=TOL):
    """Shifts d for which 0x10+d appears and 0x58+d appears later."""
    hits = []
    for d in range(N):
        a = [i for i, v in enumerate(tr) if min((v - (SYNC1 + d)) % N,
                                                ((SYNC1 + d) - v) % N) <= tol]
        if not a:
            continue
        b = [j for j, v in enumerate(tr) if min((v - (SYNC2 + d)) % N,
                                                ((SYNC2 + d) - v) % N) <= tol]
        if any(j > i for i in a for j in b):
            hits.append(d)
    return hits


def main():
    tr = traces(sys.argv[1] if len(sys.argv) > 1 else "hostsim/run_sync.txt")
    if not tr:
        print("no S lines -- is the traced build in place?")
        return 1
    print(f"{len(tr)} failed sync hunts, "
          f"mean {sum(len(t) for t in tr)/len(tr):.1f} symbols each\n")

    shift_hist = collections.Counter()
    none_found = 0
    zero_ok = 0
    for t in tr:
        hits = find_shift(t)
        if not hits:
            none_found += 1
            continue
        if 0 in hits:
            zero_ok += 1
        for d in hits:
            shift_hist[d] += 1

    print(f"  no sync pair at ANY shift : {none_found:>4} "
          f"({100*none_found/len(tr):.0f}%)")
    print(f"  pair present at shift 0   : {zero_ok:>4} "
          f"({100*zero_ok/len(tr):.0f}%)  <- would be a state-machine miss")
    print(f"  pair present at some shift: {len(tr)-none_found:>4} "
          f"({100*(len(tr)-none_found)/len(tr):.0f}%)")

    print("\n  most common shifts (d, and how many traces admit it):")
    for d, c in shift_hist.most_common(8):
        signed = d if d < N // 2 else d - N
        print(f"    d = {signed:>+4}   {c:>4} traces "
              f"{'#' * min(40, c)}")

    # A bias would show as one shift dominating. Random corruption admits many
    # shifts per trace, so report how selective the match is.
    multi = sum(1 for t in tr if len(find_shift(t)) > 4)
    print(f"\n  traces admitting >4 shifts: {multi} "
          f"({100*multi/len(tr):.0f}%) -- high means the match is not selective "
          f"and shift evidence is weak")
    return 0


if __name__ == "__main__":
    sys.exit(main())
