"""The design decision, in one table.

Three ways to pick a decimation phase, scored on whether the pick lands on a
phase that is both flat and sync-correct:

  fixed      what the firmware does today (one free-running grid)
  sequential cycle phases across preamble symbols, one look each -- nearly free
  parallel   evaluate all four every preamble symbol, pick min spread -- 4x FFT

Ground truth is the full sweep in phase_sweep.py.
"""
import sys

import numpy as np

sys.path.insert(0, "/var/home/mycroft/rf/mayhem/portapack")
from phase_sweep import (DECIM, N, PREAMBLE_MIN, circular_spread, find_bursts,
                         find_preamble, symbolize, sync_ok)

raw = np.fromfile("/var/home/mycroft/rf/mesh_iq2.bin", dtype=np.int8)
iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
bursts = find_bursts(iq)

score = {"fixed": 0, "seq_sharp": 0, "seq_spread": 0, "parallel": 0}
total = 0
rows = []

for bi, (s, e) in enumerate(bursts):
    seg = iq[max(0, s - 4 * N * DECIM): e + 4 * N * DECIM]
    if len(seg) < N * DECIM * (PREAMBLE_MIN + 2):
        continue

    truth, streams = {}, {}
    for p in range(DECIM):
        bins, sharp = symbolize(seg, p)
        streams[p] = (bins, sharp)
        pre = find_preamble(bins, sharp)
        if pre is None:
            truth[p] = (999, False, None, None)
            continue
        a, b = pre
        pb = bins[a:b]
        ok, _, _ = sync_ok(bins, b, int(pb[0]))
        truth[p] = (circular_spread(pb), ok, a, b)

    good = {p for p, (sp, ok, _, _) in truth.items() if sp == 0 and ok}
    if not good:
        continue
    total += 1
    anchor = next(p for p in range(DECIM) if truth[p][2] is not None)
    a, b = truth[anchor][2], truth[anchor][3]
    n_pre = b - a

    # fixed: the grid the device happens to be on
    fixed_ok = 0 in good

    # sequential: one symbol per phase, round robin
    seq_sharp_scores, seq_spread_bins = {}, {}
    for i in range(min(n_pre, DECIM * 2)):
        p = i % DECIM
        bins, sharp = streams[p]
        t = a + i
        if t >= len(bins):
            break
        seq_sharp_scores.setdefault(p, []).append(float(sharp[t]))
        seq_spread_bins.setdefault(p, []).append(int(bins[t]))
    seq_sharp_pick = max(seq_sharp_scores, key=lambda p: np.mean(seq_sharp_scores[p]))
    seq_spread_pick = min(
        seq_spread_bins,
        key=lambda p: (circular_spread(np.array(seq_spread_bins[p])),
                       -np.mean(seq_sharp_scores[p])))

    # parallel: every phase measured over the whole preamble, min spread wins
    par_pick = min(range(DECIM), key=lambda p: truth[p][0])

    score["fixed"] += fixed_ok
    score["seq_sharp"] += seq_sharp_pick in good
    score["seq_spread"] += seq_spread_pick in good
    score["parallel"] += par_pick in good
    rows.append((bi, n_pre, sorted(good), int(fixed_ok),
                 seq_sharp_pick, seq_spread_pick, par_pick))

print(f"{'burst':>5} {'pre':>4} {'flat phases':>14} {'fixed':>6} "
      f"{'seq-sh':>7} {'seq-sp':>7} {'par':>5}")
print("-" * 56)
for bi, npre, good, fx, ss, sp, pp in rows:
    def m(p):
        return f"{p}{'' if p in good else '*'}"
    print(f"{bi:>5} {npre:>4} {str(good):>14} {'Y' if fx else 'n':>6} "
          f"{m(ss):>7} {m(sp):>7} {m(pp):>5}")

print("\n* = pick missed a flat phase\n")
print(f"{'strategy':<24} {'hits':>8}   {'FFTs/symbol in Search'}")
print("-" * 60)
for k, label, cost in (("fixed", "fixed grid (today)", "1"),
                       ("seq_sharp", "sequential, sharpness", "1"),
                       ("seq_spread", "sequential, spread", "1"),
                       ("parallel", "parallel, min spread", "4")):
    print(f"{label:<24} {score[k]:>3}/{total:<4} ({100*score[k]/total:>3.0f}%)   {cost}")
