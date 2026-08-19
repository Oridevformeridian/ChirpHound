"""Confirm we captured LongFast (SF11/BW250) and start the host decode prototype.

LongFast: SF11, BW250, CR4/5, captured at 1 Msps (OS=4 -> 250 kHz chip rate),
so a 2^11 = 2048-point FFT per symbol. Trivial in numpy; the whole point of
proving it here before fighting the device's 256-point FFT cap.

Two things this answers:
  1. Are there SF11 bursts at this frequency? (a LongFast frame is LONG -- a
     34-byte payload is ~0.5-1 s of airtime, vs ~30 ms for SHORT_TURBO -- so
     burst DURATION alone distinguishes it.)
  2. Does dechirping with the SF11 reference collapse the preamble to a stable
     bin? That is the go/no-go for "we can decode LongFast".
"""
import sys
import numpy as np

SF = 11
N = 1 << SF          # 2048
DECIM = 4            # 1 Msps -> 250 kHz chip rate
RATE = 1_000_000


def downchirp():
    k = np.arange(N, dtype=np.float64)
    return np.exp(-1j * np.pi * k * k / N).astype(np.complex64)


CHIRP = downchirp()


def load(path, seconds):
    raw = np.fromfile(path, dtype=np.int8, count=int(seconds * RATE) * 2)
    return raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)


def find_bursts(iq):
    mag = np.abs(iq[::32])
    sm = np.convolve(mag, np.ones(256) / 256, mode="same")
    db = 20 * np.log10(np.maximum(sm, 1e-6))
    floor = np.percentile(db, 40)
    hot = db > floor + 6
    bursts, start = [], None
    for i, h in enumerate(hot):
        if h and start is None:
            start = i
        elif not h and start is not None:
            if i - start > 100:            # >100*32 samples ~ 3 ms, ignore specks
                bursts.append((start * 32, i * 32))
            start = None
    return bursts


def symbolize(seg):
    n = (len(seg) // DECIM) * DECIM
    dec = seg[:n].reshape(-1, DECIM).sum(axis=1)
    ns = len(dec) // N
    if ns == 0:
        return np.empty(0, int), np.empty(0)
    frames = dec[: ns * N].reshape(ns, N)
    spec = np.fft.fft(frames * CHIRP, axis=1)
    mag = spec.real ** 2 + spec.imag ** 2
    bins = mag.argmax(axis=1)
    peak = mag.max(axis=1)
    mean = mag.mean(axis=1)
    return bins, peak / np.maximum(mean, 1e-9)


def main():
    path = sys.argv[1]
    iq = load(path, 40)
    print(f"loaded {len(iq)/RATE:.1f}s @ {RATE/1e6:.1f} Msps")
    bursts = find_bursts(iq)
    print(f"{len(bursts)} bursts found")
    for bi, (s, e) in enumerate(bursts[:12]):
        dur_ms = (e - s) / RATE * 1000
        seg = iq[s:e]
        bins, sharp = symbolize(seg)
        if len(bins) < 8:
            print(f"  burst {bi}: {dur_ms:6.0f} ms  (too short: {len(bins)} sym)")
            continue
        # a LoRa preamble = run of identical symbols. Look for the longest such
        # run among sharp symbols.
        good = sharp > 8
        best_run = run = 0
        run_bin = -1
        for i in range(len(bins)):
            if good[i] and (run == 0 or abs(int(bins[i]) - run_bin) <= 2):
                run += 1
                run_bin = int(bins[i])
                best_run = max(best_run, run)
            else:
                run = 1 if good[i] else 0
                run_bin = int(bins[i]) if good[i] else -1
        tag = "SF11 PREAMBLE!" if best_run >= 6 else "no clean preamble"
        print(f"  burst {bi}: {dur_ms:6.0f} ms  {len(bins):3d} sym  "
              f"max preamble run {best_run:2d}  peak sharp {sharp.max():6.0f}  {tag}")


if __name__ == "__main__":
    main()
