"""Is sequential phase probing good enough, or does it need parallel FFTs?

phase_sweep.py established that every burst has a decimation phase with a flat
preamble, and that flat always produced the correct sync word (21/21). So the
fix is real. This asks the *implementation* question, which decides how much
M4 budget it costs:

  Parallel: run all four phases every symbol -- 4 dechirp+FFTs per 256us
  symbol. Certain, expensive, needs a CPU budget it may not have.

  Sequential: cycle the phase across preamble symbols, one measurement each,
  pick the winner and commit. Nearly free -- but a preamble is only 8-10
  symbols, so each phase gets 2 or 3 looks, and a metric that needs many
  samples will pick noise.

Simulated honestly: symbol t is taken from the stream of whichever phase the
probe schedule had selected at time t, exactly as the firmware would see it.
Changing phase shifts the grid by a quarter chip, far less than a symbol, so
symbol t lines up across streams.

Two candidate metrics, because they cost differently:
  * sharpness  -- peak/mean, already computed, works on ONE symbol per phase
  * spread     -- needs >=2 symbols at the same phase to mean anything

The output is the fraction of bursts where the probe picks a phase that is
both flat and sync-correct. Ground truth is phase_sweep's full 4-phase sweep.
"""
import sys

import numpy as np

sys.path.insert(0, "/var/home/mycroft/rf/mayhem/portapack")
from phase_sweep import (DECIM, N, PREAMBLE_MIN, RATE, circular_spread,
                         find_bursts, find_preamble, symbolize, sync_ok)


def truth_for(seg):
    """Full sweep: which phases are flat and sync-correct?"""
    out = {}
    for p in range(DECIM):
        bins, sharp = symbolize(seg, p)
        pre = find_preamble(bins, sharp)
        if pre is None:
            out[p] = (999, False, None, None)
            continue
        a, b = pre
        pb = bins[a:b]
        ok, _, _ = sync_ok(bins, b, int(pb[0]))
        out[p] = (circular_spread(pb), ok, a, b)
    return out


def probe(seg, truth, hold):
    """Simulate the firmware cycling phases during the preamble.

    `hold` is how many consecutive symbols each phase is kept for before
    stepping. hold=1 gives 4 phases in 4 symbols; hold=2 needs 8, which is
    most of a preamble.
    """
    # Anchor the schedule on a phase that actually locks, the way the device
    # does: it is already in Search on some phase when the preamble arrives.
    anchor = next((p for p in range(DECIM) if truth[p][2] is not None), None)
    if anchor is None:
        return None
    a, b = truth[anchor][2], truth[anchor][3]
    n_pre = b - a
    if n_pre < DECIM * hold:
        return "too_short"

    streams = {p: symbolize(seg, p) for p in range(DECIM)}

    per_phase_bins = {p: [] for p in range(DECIM)}
    per_phase_sharp = {p: [] for p in range(DECIM)}
    for i in range(DECIM * hold):                 # only the symbols we can spend
        t = a + i
        p = (i // hold) % DECIM
        bins, sharp = streams[p]
        if t >= len(bins):
            return "too_short"
        per_phase_bins[p].append(int(bins[t]))
        per_phase_sharp[p].append(float(sharp[t]))

    picks = {}
    picks["sharp"] = max(range(DECIM),
                         key=lambda p: np.mean(per_phase_sharp[p]))
    if hold >= 2:
        picks["spread"] = min(
            range(DECIM),
            key=lambda p: (circular_spread(np.array(per_phase_bins[p])),
                           -np.mean(per_phase_sharp[p])))
    return picks


def main():
    raw = np.fromfile("/var/home/mycroft/rf/mesh_iq2.bin", dtype=np.int8)
    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    bursts = find_bursts(iq)
    print(f"{len(bursts)} bursts\n")

    results = {}
    for hold in (1, 2, 3):
        tally = {}
        for s, e in bursts:
            seg = iq[max(0, s - 4 * N * DECIM): e + 4 * N * DECIM]
            if len(seg) < N * DECIM * (PREAMBLE_MIN + 2):
                continue
            truth = truth_for(seg)
            good = {p for p, (sp, ok, _, _) in truth.items() if sp == 0 and ok}
            if not good:
                continue
            picks = probe(seg, truth, hold)
            if picks is None or picks == "too_short":
                tally["too_short"] = tally.get("too_short", 0) + 1
                continue
            tally["n"] = tally.get("n", 0) + 1
            for metric, p in picks.items():
                if p in good:
                    tally[metric] = tally.get(metric, 0) + 1
        results[hold] = tally

    print(f"{'hold':>5} {'symbols':>8} {'usable':>7} {'short':>6} "
          f"{'sharp':>10} {'spread':>10}")
    print("-" * 52)
    for hold, t in results.items():
        n = t.get("n", 0)
        sh = f"{t.get('sharp',0)}/{n}" if n else "-"
        sp = f"{t.get('spread',0)}/{n}" if (n and hold >= 2) else "n/a"
        print(f"{hold:>5} {DECIM*hold:>8} {n:>7} {t.get('too_short',0):>6} "
              f"{sh:>10} {sp:>10}")

    print("\n'usable' = bursts that had a flat+sync phase to find at all.")
    print("A metric scores when its pick lands on one of those phases.")


if __name__ == "__main__":
    main()
