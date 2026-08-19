"""Two receivers, same packets: is the residual noise or is it systematic?

This is the one experiment that separates the two remaining explanations, and
it needs no DSP change at all.

Two HackRFs hear the same transmissions with *independent* thermal noise. So:

  * SNR-limited  -> which packets decode is a coin flip per receiver, and the
                    overlap between the two sets is roughly what independence
                    predicts. Union >> either alone.
  * systematic   -> the same packets fail on both, because the fault is in the
                    signal or the decoder, not the channel. Overlap ~ the
                    smaller set, union ~ the larger.

Beacons carry a sequence number in their plaintext, so a decrypted frame
identifies its own transmission exactly -- no timestamp alignment needed
between two independently started captures.

    distrobox enter sdrbox -- python3 dual_corr.py runA.txt runB.txt
"""
import re
import sys

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

SEQ = re.compile(r"BEACON\s+(\d+)")


def decoded(path, chans):
    """(set of beacon seqs decrypted, n_frames, n_decrypted)."""
    seqs, frames, dec = set(), 0, 0
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 3 or p[1] != "F":
                continue
            frames += 1
            try:
                rec = packets.process(bytes.fromhex(p[2]), chans)
            except Exception:
                continue
            if not rec or not rec.get("decrypted"):
                continue
            dec += 1
            m = SEQ.search(str(rec.get("text") or ""))
            if m:
                seqs.add(int(m.group(1)))
    return seqs, frames, dec


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    chans = keys.load(keys.DEFAULT_PATH)
    A, fa, da = decoded(sys.argv[1], chans)
    B, fb, db = decoded(sys.argv[2], chans)

    print(f"radio A: {fa:>4} frames, {da:>3} decrypted, {len(A):>3} with a seq")
    print(f"radio B: {fb:>4} frames, {db:>3} decrypted, {len(B):>3} with a seq")

    if not A or not B:
        print("\nOne radio decrypted nothing identifiable -- cannot compare "
              "sets. That is itself a result: the radios are not equivalent.")
        return 0

    both = A & B
    either = A | B
    print(f"\n  both radios : {len(both):>3}")
    print(f"  A only      : {len(A - B):>3}")
    print(f"  B only      : {len(B - A):>3}")
    print(f"  union       : {len(either):>3}")

    # Independence baseline. The population is every beacon transmitted in the
    # window; approximate it by the span of sequence numbers actually seen,
    # which is the only handle we have on how many went out.
    lo, hi = min(either), max(either)
    n_tx = hi - lo + 1
    exp = len(A) * len(B) / n_tx if n_tx else 0
    print(f"\n  beacons transmitted in span: ~{n_tx} (seq {lo}..{hi})")
    print(f"  overlap expected if independent: {exp:.1f}")
    print(f"  overlap observed:                {len(both)}")

    if exp > 0:
        ratio = len(both) / exp
        print(f"  ratio: {ratio:.2f}x")
        print()
        if ratio > 2.0:
            print("  => SYSTEMATIC. The same packets decode on both radios far")
            print("     more often than chance. Independent noise is not what")
            print("     is deciding success; the fault is in the signal or the")
            print("     decoder, and more SNR will not fix it.")
        elif ratio < 1.5:
            print("  => NOISE-LIMITED. Success is close to independent per")
            print("     receiver, so the decoder is working and the device is")
            print("     near its sensitivity floor. The lever is RF, and")
            print("     diversity combining would nearly double the yield.")
        else:
            print("  => MIXED / underpowered. Not separable at this sample")
            print("     size; more capture needed before concluding.")

    gain = len(either) / max(len(A), len(B)) if either else 0
    print(f"\n  diversity gain if combined: {gain:.2f}x the better radio")
    return 0


if __name__ == "__main__":
    sys.exit(main())
