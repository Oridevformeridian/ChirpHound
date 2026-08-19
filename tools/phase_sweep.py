"""Does choosing a decimation phase fix the wobbly preambles?

The device's own symbol logs show the discriminator cleanly: a preamble is ten
identical symbols by definition, so when its FFT bins wobble by +/-1 the
sampling window is sitting between chip boundaries and noise picks the bin.
Flat preamble -> decodes. Wobbly -> fails. That is measured, not theory.

What is NOT established is the proposed fix. The claim is that the 2 Msps
stream carries four decimation phases, so one of them must land near a chip
boundary and produce a flat preamble. That is a hypothesis about the signal,
and it is testable offline against raw IQ without touching the hardware.

This models the PortaPack chain exactly as proc_meshtastic.cpp implements it:

  * 8-bit IQ at 2 Msps in
  * boxcar decimate by 4 (sum four consecutive samples), starting at phase p
  * fixed 128-sample framing -- deliberately NOT aligned to symbols, because
    the device does not align either; that is why a preamble bin is nonzero at
    all, it *is* the timing offset
  * dechirp by the conjugate upchirp, 128-point FFT, peak bin = symbol

Then for every burst it sweeps p = 0..3 and reports the preamble bin spread.
The question the numbers answer:

  does every burst have at least one phase with spread 0?

If yes, min-spread phase selection is worth building in firmware. If no, the
residual timing error is finer than a quarter chip and the SFD/CFO route is
the only thing that will work -- and we learn that for free, here, instead of
after another card swap.
"""
import argparse
import sys

import numpy as np

SF = 7
N = 1 << SF            # 128
DECIM = 4              # 2 Msps -> 500 kHz chip rate
RATE = 2_000_000
PREAMBLE_MIN = 6       # what the firmware requires to declare a lock
BIN_TOL = 1            # firmware's bin_tolerance


def downchirp():
    """Conjugate upchirp, matching the firmware's constructor exactly:
    phase = pi*k^2/N, stored as (cos(-phase), sin(-phase))."""
    k = np.arange(N, dtype=np.float64)
    phase = np.pi * k * k / N
    return np.exp(-1j * phase).astype(np.complex64)


CHIRP = downchirp()


def symbolize(iq, phase):
    """IQ -> symbol bins, the way the device does it.

    phase selects which of the four 2 Msps samples starts each boxcar group,
    which is the quarter-chip timing knob the firmware currently does not
    actually apply.
    """
    x = iq[phase:]
    n = (len(x) // DECIM) * DECIM
    if n < DECIM:
        return np.empty(0, dtype=np.int32), np.empty(0, dtype=np.float32)
    dec = x[:n].reshape(-1, DECIM).sum(axis=1)

    nsym = len(dec) // N
    if nsym == 0:
        return np.empty(0, dtype=np.int32), np.empty(0, dtype=np.float32)
    frames = dec[: nsym * N].reshape(nsym, N)

    spec = np.fft.fft(frames * CHIRP, axis=1)
    mag = (spec.real ** 2 + spec.imag ** 2)
    bins = mag.argmax(axis=1).astype(np.int32)
    peak = mag.max(axis=1)
    mean = mag.mean(axis=1)
    # Firmware's sharpness gate: peak must hold a good fraction of the energy.
    sharp = np.divide(peak, np.maximum(mean, 1e-12))
    return bins, sharp.astype(np.float32)


def circular_spread(bins):
    """Spread of a set of bins on a 128-point circle.

    Plain max-min is wrong when the preamble sits near bin 0: bins 127 and 0
    are adjacent, not 127 apart, and that would report a flat preamble as the
    worst one in the set.
    """
    if len(bins) == 0:
        return 999
    d = ((bins - bins[0] + N // 2) % N) - N // 2
    return int(d.max() - d.min())


def find_preamble(bins, sharp, min_run=PREAMBLE_MIN):
    """First run of consecutive near-identical sharp symbols, as the firmware
    hunts it: sharpness gate, then bins agreeing within bin_tolerance."""
    best = None
    run_start = None
    for i in range(len(bins)):
        if sharp[i] <= 8.0:
            run_start = None
            continue
        if run_start is None:
            run_start = i
            continue
        d = abs(int(bins[i]) - int(bins[i - 1]))
        d = min(d, N - d)
        if d > BIN_TOL:
            if i - run_start >= min_run:
                best = (run_start, i)
                break
            run_start = i
    if best is None and run_start is not None and len(bins) - run_start >= min_run:
        best = (run_start, len(bins))
    return best


# LoRa encodes sync word 0x2B as two symbols; relative to the preamble bin
# they come out as exactly these two values in every capture the device logged.
# Checking them turns "flat preamble" from a proxy into evidence of a decodable
# packet -- a flat preamble that produced the wrong sync word would mean the
# discriminator was a coincidence.
SYNC = (0x10, 0x58)


def sync_ok(bins, pre_end, pre_bin):
    """Do the two symbols after the preamble carry the sync word?

    The preamble bin is the timing offset, so everything is measured relative
    to it. Searched over a small window rather than a fixed position because
    the preamble run length varies per packet (2 to 10 observed on-device).
    """
    for start in range(pre_end - 2, min(pre_end + 6, len(bins) - 1)):
        if start < 0:
            continue
        a = (int(bins[start]) - pre_bin) % N
        b = (int(bins[start + 1]) - pre_bin) % N
        for tol in (0, 1):
            if (abs(a - SYNC[0]) <= tol and abs(b - SYNC[1]) <= tol):
                return True, start, (a, b)
    return False, -1, (-1, -1)


def find_bursts(iq, floor_db=None):
    """Energy bursts, coarse. Only needs to bracket a packet, not frame it."""
    mag = np.abs(iq[::16])                       # decimated envelope is plenty
    win = 64
    sm = np.convolve(mag, np.ones(win) / win, mode="same")
    db = 20 * np.log10(np.maximum(sm, 1e-6))
    floor = floor_db if floor_db is not None else np.percentile(db, 50)
    hot = db > (floor + 6.0)

    bursts, start = [], None
    for i, h in enumerate(hot):
        if h and start is None:
            start = i
        elif not h and start is not None:
            if i - start > 20:                   # ignore specks
                bursts.append((start * 16, i * 16))
            start = None
    if start is not None:
        bursts.append((start * 16, len(hot) * 16))
    return bursts


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("iq", nargs="?", default="/var/home/mycroft/rf/mesh_iq2.bin")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--max-bursts", type=int, default=40)
    args = ap.parse_args()

    count = int(args.seconds * RATE) * 2
    raw = np.fromfile(args.iq, dtype=np.int8, count=count)
    iq = (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32))
    print(f"loaded {len(iq)/RATE:.1f}s of IQ ({len(iq)} samples)")

    bursts = find_bursts(iq)
    print(f"{len(bursts)} bursts detected\n")

    hdr = f"{'burst':>5} {'ms':>7} " + " ".join(f"{'p'+str(p):>18}" for p in range(DECIM))
    print(hdr)
    print(f"{'':>5} {'':>7} " + " ".join(f"{'sprd syn  bin':>18}" for _ in range(DECIM)))
    print("-" * len(hdr))

    stats = {"any_flat": 0, "p0_flat": 0, "total": 0}
    for bi, (s, e) in enumerate(bursts[: args.max_bursts]):
        # A little padding: the detector's edges are soft.
        seg = iq[max(0, s - 4 * N * DECIM): e + 4 * N * DECIM]
        if len(seg) < N * DECIM * (PREAMBLE_MIN + 2):
            continue

        row, spreads, syncs = [], [], []
        for p in range(DECIM):
            bins, sharp = symbolize(seg, p)
            pre = find_preamble(bins, sharp)
            if pre is None:
                row.append(f"{'--':>4} {'-':>3} {'--':>4}")
                spreads.append(999)
                syncs.append(False)
                continue
            a, b = pre
            pb = bins[a:b]
            sp = circular_spread(pb)
            spreads.append(sp)
            ok, _, _ = sync_ok(bins, b, int(pb[0]))
            syncs.append(ok)
            row.append(f"{sp:>4} {'Y' if ok else 'n':>3} {int(pb[0]):>4}")

        if min(spreads) == 999:
            continue
        stats["total"] += 1
        if min(spreads) == 0:
            stats["any_flat"] += 1
        if spreads[0] == 0:
            stats["p0_flat"] += 1
        # The discriminator itself: does flat imply sync-correct?
        for sp, ok in zip(spreads, syncs):
            if sp == 999:
                continue
            key = ("flat" if sp == 0 else "wobbly") + ("_sync" if ok else "_nosync")
            stats[key] = stats.get(key, 0) + 1
        if any(sp == 0 and ok for sp, ok in zip(spreads, syncs)):
            stats["recoverable"] = stats.get("recoverable", 0) + 1

        flag = "" if min(spreads) == 0 else "   <-- no flat phase"
        print(f"{bi:>5} {s/RATE*1000:>7.0f} " + " ".join(f"{c:>18}" for c in row) + flag)

    t = stats["total"]
    if not t:
        print("\nno bursts yielded a preamble -- detector or format mismatch")
        return 1
    print(f"\n{'-'*60}")
    print(f"bursts with a preamble:            {t}")
    print(f"  flat at the fixed phase (p0):    {stats['p0_flat']:>3}  "
          f"({100*stats['p0_flat']/t:.0f}%)   <- what the device gets today")
    print(f"  flat at SOME phase:              {stats['any_flat']:>3}  "
          f"({100*stats['any_flat']/t:.0f}%)   <- ceiling for phase selection")
    print(f"\n  discriminator, over all {DECIM} phases x {t} bursts:")
    for k in ("flat_sync", "flat_nosync", "wobbly_sync", "wobbly_nosync"):
        print(f"    {k:<16} {stats.get(k, 0):>4}")
    print(f"\n  bursts with a phase that is BOTH flat and sync-correct: "
          f"{stats.get('recoverable', 0)}/{t}")
    gain = stats["any_flat"] - stats["p0_flat"]
    print(f"\nphase selection would recover {gain} of {t - stats['p0_flat']} "
          f"currently-wobbly bursts.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
