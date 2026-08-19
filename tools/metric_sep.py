"""Why did sequential probing gain nothing? Look at whether the metrics
separate the phases at all.

If sharpness is flat across phases during a preamble, no amount of probing
schedule will find the aligned one -- the signal the selector needs simply is
not in that number, and the fix has to come from a different measurement.
"""
import sys

import numpy as np

sys.path.insert(0, "/var/home/mycroft/rf/mayhem/portapack")
from phase_sweep import (DECIM, N, PREAMBLE_MIN, circular_spread, find_bursts,
                         find_preamble, symbolize, sync_ok)

raw = np.fromfile("/var/home/mycroft/rf/mesh_iq2.bin", dtype=np.int8)
iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
bursts = find_bursts(iq)

print(f"{'burst':>5}  {'phase':>5} {'spread':>7} {'sync':>5} "
      f"{'mean sharp':>11} {'peak/next':>10}")
print("-" * 52)

sep_sharp = []
sep_ratio = []
for bi, (s, e) in enumerate(bursts):
    seg = iq[max(0, s - 4 * N * DECIM): e + 4 * N * DECIM]
    if len(seg) < N * DECIM * (PREAMBLE_MIN + 2):
        continue

    rows = []
    for p in range(DECIM):
        bins, sharp = symbolize(seg, p)
        pre = find_preamble(bins, sharp)
        if pre is None:
            continue
        a, b = pre
        pb = bins[a:b]
        sp = circular_spread(pb)
        ok, _, _ = sync_ok(bins, b, int(pb[0]))

        # Adjacent-bin ratio: a tone centred on a bin leaves its neighbour
        # small; a tone halfway between two bins splits energy evenly. This is
        # the classic sub-bin timing estimate and it needs no extra FFT.
        x = seg[p:]
        n = (len(x) // DECIM) * DECIM
        dec = x[:n].reshape(-1, DECIM).sum(axis=1)
        nsym = len(dec) // N
        frames = dec[:nsym * N].reshape(nsym, N)
        spec = np.fft.fft(frames * __import__("phase_sweep").CHIRP, axis=1)
        mag = spec.real ** 2 + spec.imag ** 2
        ratios = []
        for t in range(a, min(b, nsym)):
            m = mag[t]
            k = int(m.argmax())
            nb = max(m[(k - 1) % N], m[(k + 1) % N])
            ratios.append(m[k] / max(nb, 1e-12))
        rows.append((p, sp, ok, float(np.mean(sharp[a:b])),
                     float(np.mean(ratios)) if ratios else 0.0))

    if not rows:
        continue
    flat = [r for r in rows if r[1] == 0]
    wob = [r for r in rows if r[1] > 0]
    for p, sp, ok, sh, ra in rows:
        mark = " <- flat" if sp == 0 else ""
        print(f"{bi:>5}  {p:>5} {sp:>7} {'Y' if ok else 'n':>5} "
              f"{sh:>11.1f} {ra:>10.2f}{mark}")
    if flat and wob:
        sep_sharp.append(np.mean([r[3] for r in flat]) - np.mean([r[3] for r in wob]))
        sep_ratio.append(np.mean([r[4] for r in flat]) - np.mean([r[4] for r in wob]))
    print()

print("=" * 52)
print("Separation = mean(flat phases) - mean(wobbly phases), per burst.")
print("A metric that can drive a selector must be reliably positive.\n")
for name, vals in (("sharpness", sep_sharp), ("peak/next-bin ratio", sep_ratio)):
    if not vals:
        print(f"  {name:<22} no burst had both flat and wobbly phases")
        continue
    v = np.array(vals)
    print(f"  {name:<22} mean {v.mean():>8.2f}   positive in "
          f"{int((v > 0).sum())}/{len(v)} bursts")
