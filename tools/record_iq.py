"""Record continuous int8 IQ -- a fixture for the host harness.

meshpipe.capture writes *bursts* as .npz, which is right for the pipeline and
wrong for hostsim: the harness feeds a continuous stream through the real
processor, exactly as the radio would, so it needs one unbroken file.

Format is chosen to match what a PortaPack baseband actually receives:
interleaved signed 8-bit I/Q at 2 Msps. CS8 comes straight off SoapySDR with no
conversion, so nothing is rescaled behind our back -- what lands in the file is
what the ADC produced, at the resolution the device works in.

Centred *on* the channel, not offset. The wideband scanner deliberately offset
tunes to dodge the LO spike, but the PortaPack tunes to the channel and its
baseband sees the signal at DC, spike and all. A fixture that removes an
artifact the target hardware has would flatter the decoder.

    python3 record_iq.py out.bin --seconds 180 --gain 40
"""
import argparse
import os
import sys
import time

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS8, SOAPY_SDR_CS16, SOAPY_SDR_RX

# oob channel: Meshtastic derives this from bandwidth and channel number,
# freq = 902.0 + bw/2 + n*bw for US915 -- BW500 slot 30 lands here.
DEFAULT_FREQ = 917.25e6
DEFAULT_RATE = 2_000_000      # shortturbo: SF7 BW500, OS=4


def main():
    ap = argparse.ArgumentParser(description="record continuous int8 IQ")
    ap.add_argument("out")
    ap.add_argument("--driver", default="lime")
    ap.add_argument("--serial", default=None,
                    help="pick one of several identical radios")
    ap.add_argument("--freq", type=float, default=DEFAULT_FREQ)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--antenna", default=None)
    ap.add_argument("--seconds", type=float, default=180.0)
    args = ap.parse_args()

    # A dict is *not* accepted here even though enumerate() takes one; the SWIG
    # binding will not convert it into matchable kwargs and raises "no match"
    # with the device plainly present.
    args_str = f"driver={args.driver}"
    if args.serial:
        args_str += f",serial={args.serial}"
    dev = SoapySDR.Device(args_str)
    dev.setSampleRate(SOAPY_SDR_RX, 0, args.rate)
    dev.setFrequency(SOAPY_SDR_RX, 0, args.freq)
    dev.setGain(SOAPY_SDR_RX, 0, args.gain)
    if args.antenna:
        dev.setAntenna(SOAPY_SDR_RX, 0, args.antenna)

    print(f"{args.driver}{'/' + args.serial[-6:] if args.serial else ''} @ {args.freq/1e6:.3f} MHz, {args.rate/1e6:.2f} Msps, "
          f"gain {args.gain}, {args.seconds:.0f}s -> {args.out}", flush=True)

    # CS8 is what the file wants and what a PortaPack works in, but LMS7 does
    # not offer it ("unsupported stream format"), so fall back to CS16 and
    # narrow it here. Which path ran matters for interpreting the peak.
    try:
        stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS8)
        native8 = True
    except RuntimeError:
        stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16)
        native8 = False
        print("  CS8 unsupported by this driver; reading CS16 and scaling")
    dev.activateStream(stream)

    CHUNK = 1 << 16
    dtype = np.int8 if native8 else np.int16
    buf = np.empty(CHUNK * 2, dtype)       # interleaved I,Q
    want = int(args.seconds * args.rate)
    got = 0
    overflows = 0
    peak = 0
    clipped = 0
    total_samp = 0
    t0 = time.time()

    # Auto-range against the NOISE FLOOR, not the pre-scan peak.
    #
    # mesh_iq2.bin peaks at +/-3 of a possible +/-127, so its dechirp ran on
    # about two bits; a fixture must not repeat that. But the first version of
    # this scaled the pre-scan peak to 96 and clipped the whole recording: a
    # half-second pre-scan almost never contains a burst, so it measured noise
    # and then amplified noise to full scale, leaving nothing for the signal.
    #
    # A burst sits tens of dB above the floor, so put the floor low and keep
    # the range for the packets. Noise RMS near 6 leaves ~26 dB of headroom
    # before clipping, and still uses far more than two bits.
    scale = 1.0
    if not native8:
        seen = 0
        acc = 0.0
        n_acc = 0
        while seen < int(args.rate * 0.5):
            sr = dev.readStream(stream, [buf], CHUNK, timeoutUs=2_000_000)
            if sr.ret <= 0:
                continue
            x = buf[: sr.ret * 2].astype(np.float64)
            acc += float(np.sum(x * x))
            n_acc += x.size
            seen += sr.ret
        rms = (acc / max(n_acc, 1)) ** 0.5
        scale = max(1.0, rms / 6.0)
        print(f"  noise rms {rms:.0f} -> divisor {scale:.1f} "
              f"(floor ~6/127, ~26 dB headroom for bursts)", flush=True)

    try:
        with open(args.out, "wb") as fh:
            while got < want:
                sr = dev.readStream(stream, [buf], CHUNK, timeoutUs=2_000_000)
                if sr.ret <= 0:
                    # -4 is overflow: the radio outran us. Worth counting --
                    # a fixture with dropped samples has packets sliced in
                    # half, which decodes as nothing and looks like a decoder
                    # bug.
                    if sr.ret == -4:
                        overflows += 1
                        continue
                    print(f"readStream: {sr.ret}", file=sys.stderr)
                    if sr.ret == -1:
                        continue
                    break
                n = sr.ret
                if native8:
                    out = buf[: n * 2]
                else:
                    out = np.clip(np.rint(buf[: n * 2] / scale),
                                  -128, 127).astype(np.int8)
                fh.write(out.tobytes())
                peak = max(peak, int(np.abs(out).max()))
                clipped += int(np.count_nonzero(np.abs(out) >= 127))
                total_samp += out.size
                got += n
                if got % (args.rate * 10) < CHUNK:
                    print(f"  {got/args.rate:5.0f}s  peak |sample| {peak:4d}/127",
                          flush=True)
    finally:
        dev.deactivateStream(stream)
        dev.closeStream(stream)

    dur = time.time() - t0
    size = os.path.getsize(args.out)
    print(f"\n{got/args.rate:.1f}s recorded in {dur:.1f}s wall, "
          f"{size/1e6:.0f} MB, {overflows} overflows")
    frac = (clipped / total_samp) if total_samp else 0.0
    print(f"peak |sample| {peak}/127, clipped {clipped} ({frac:.3%})", end="")
    # A handful of clipped samples on the strongest burst is normal; a
    # percent of them means the fixture is distorted and any decode result
    # measured on it is about the clipping, not the algorithm.
    if peak < 16:
        print("  -- LOW: most of the int8 range unused, raise --gain")
    elif frac > 0.001:
        print("  -- CLIPPING: lower --gain")
    else:
        print("  -- healthy")
    return 0


if __name__ == "__main__":
    sys.exit(main())
