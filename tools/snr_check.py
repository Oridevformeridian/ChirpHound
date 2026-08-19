"""Does the near radio actually have more SNR -- and does it help?

One HackRF sits on top of the transmitter, the other is 3 m away. That is a
large SNR difference by construction. It makes the dual-radio result testable
in a way I did not have when I interpreted it:

  * if the near radio has much higher sharpness AND a much better decode rate
    -> noise-limited, as concluded
  * if it has much higher sharpness and a SIMILAR decode rate
    -> NOT noise-limited. Something else is failing, and the independence
       between radios comes from something that differs per receiver but is
       not thermal noise -- each radio's own sample-clock phase being the
       obvious candidate, since the ADC grid is unrelated between them

Sharpness (peak/mean of the symbol FFT) is already carried on every D line.
"""
import sys

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

TRUE_SENDER = 0x4358AB2C


def scan(path, chans):
    sharps, ok_sharps, bad_sharps = [], [], []
    frames = dec = sender_ok = 0
    locks = 0
    pending = None
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 2:
                continue
            if p[1] == "P":
                locks += 1
            elif p[1] == "F" and len(p) > 2:
                frames += 1
                pending = p[2]
            elif p[1] == "D" and len(p) > 2 and pending is not None:
                raw = bytes.fromhex(p[2])
                sharp = 2.0 ** (raw[3] / 25.0) if len(raw) >= 4 else 0.0
                good = False
                try:
                    rec = packets.process(bytes.fromhex(pending), chans)
                    if rec:
                        if rec.get("sender") == TRUE_SENDER:
                            sender_ok += 1
                            good = True
                        if rec.get("decrypted"):
                            dec += 1
                except Exception:
                    pass
                sharps.append(sharp)
                (ok_sharps if good else bad_sharps).append(sharp)
                pending = None
    return dict(locks=locks, frames=frames, dec=dec, sender_ok=sender_ok,
                sharps=sharps, ok=ok_sharps, bad=bad_sharps)


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def main():
    chans = keys.load(keys.DEFAULT_PATH)
    print(f"{'radio':<8} {'locks':>6} {'frames':>7} {'hdr%':>6} "
          f"{'sender':>7} {'decrypt':>8} {'mean sharp':>11} {'max':>7}")
    res = {}
    for label, path in (("A near", sys.argv[1]), ("B far", sys.argv[2])):
        r = scan(path, chans)
        res[label] = r
        hdr = 100.0 * r["frames"] / r["locks"] if r["locks"] else 0
        print(f"{label:<8} {r['locks']:>6} {r['frames']:>7} {hdr:>5.1f}% "
              f"{r['sender_ok']:>7} {r['dec']:>8} {mean(r['sharps']):>11.1f} "
              f"{max(r['sharps']) if r['sharps'] else 0:>7.1f}")

    a, b = res["A near"], res["B far"]
    sa, sb = mean(a["sharps"]), mean(b["sharps"])
    print(f"\n  sharpness ratio near/far: {sa/sb:.2f}x" if sb else "")
    ha = a["frames"] / a["locks"] if a["locks"] else 0
    hb = b["frames"] / b["locks"] if b["locks"] else 0
    print(f"  header-rate ratio near/far: {ha/hb:.2f}x" if hb else "")
    print()
    if sb and sa / sb > 1.5 and hb and ha / hb < 1.3:
        print("  => NOT noise-limited. The near radio has substantially more")
        print("     signal and converts almost none of it into extra frames.")
        print("     The earlier 'independent => thermal noise' reading was")
        print("     wrong: the independence must come from something that")
        print("     differs per receiver but is not SNR -- each radio's own")
        print("     sample-clock phase relative to the transmitter is the")
        print("     obvious candidate, and it is exactly what phase_sweep")
        print("     measured offline.")
    elif sb and sa / sb > 1.5:
        print("  => more signal does buy more frames: noise-limited stands.")
    else:
        print("  => the two radios do not differ enough in SNR to separate")
        print("     the hypotheses; check gains and antennas.")

    for label in ("A near", "B far"):
        r = res[label]
        print(f"\n  {label}: sharpness of sender-correct frames "
              f"{mean(r['ok']):.1f} (n={len(r['ok'])}) vs failed "
              f"{mean(r['bad']):.1f} (n={len(r['bad'])})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
