# LongFast (SF11 / BW250) — the DEFAULT-preset port

ChirpHound started on **SHORT_TURBO** (SF7/BW500) because it's the fastest preset
to iterate on. But the preset that matters is **LongFast** — Meshtastic's DEFAULT,
the one real public mesh traffic uses. SF11 carries ~12 dB more processing gain
than SF7, and that is exactly the margin the PortaPack's modest front end needs:
where SHORT_TURBO preambles were marginal, LongFast preambles lock rock-solid —
**and the frame decodes clean and decrypts end to end.**

```
DECRYPTED  !4358ab2c -> !ffffffff  ch eb  text='BEACON 00004'
DECRYPTED  !4358ab2c -> !ffffffff  ch eb  text='BEACON 00005'
```

*Two LongFast beacon bursts, decoded by the real firmware DSP (via `lf_hostsim`)
and decrypted on the DEFAULT channel — byte-identical to the host reference.*

## What it took

**The FFT.** SF11 needs a 2048-point FFT; the stock `dsp_fft` twiddle table
(`common/dsp_fft.hpp`) capped out at 256 (`K_max = 8`). The FFT itself is generic
— only the `wp_table` (each entry `exp(-iπ/2^k) - 1`) needed three more rows (see
`firmware/dsp_fft_sf11.patch`). SF11 is *slow* (122 symbols/s, 8.19 ms/symbol), so
even a 2048-point FFT leaves ample budget.

**The decoder.** `baseband/proc_meshtastic_lf.{hpp,cpp}`: preamble lock →
fractional-CFO estimate → symbol capture → header (reduced-rate `sf_app = SF-2`,
`sym >> 2`) → payload (`sf_app = SF`, LDRO off, `padj = -1`) → Hamming/dewhiten via
the shared `lora_decode` primitives → **LoRa CRC-16** gate. Every uncertain SF11
parameter was pinned in the numpy reference (`host/longfast_decode.py`) against
real signal, then ported to the firmware DSP.

**A CRC bug worth noting.** `lora::crc16_ok` (sx127x LoRa CRC-16, poly 0x1021) must
run over the first `plen-2` payload bytes and fold in the last two — an off-by-two
that silently fails *every* frame, perfect ones included. Fixed and unit-checked.

**Timing recovery — the last symbol.** The streaming decoder windows symbols at
fixed 2048-sample intervals from lock, offset from the *true* symbol boundary by
an unknown number of decimated samples. Normalisation hides that as a bin shift
for clean symbols, but the residual ISI flips whichever symbol is marginal — one
per burst on this capture (read bin 1528 where truth was 1430). The fix is a
**frame-boundary sweep**: re-window the burst over `toff = 0..N` decimated samples
(× decimation-phase shift × a fractional-CFO grid) and let the CRC pick the
alignment that lands *every* symbol. That closes it — full CRC-clean decode,
byte-for-byte with the reference, in ~3 s for two bursts.

## On-device decoder (candidate streaming)

The frame-boundary sweep proves the signal is decodable but buffers the whole
burst's raw IQ — impossible on the M4 (96 KB RAM, ~80 KB heap). The deployable
decoder gets the same answer with a **candidate-streaming** design that stores
bins, not samples (`LF_CAND`):

- **Preamble-seeded timing.** The frame-timing offset tracks the preamble bin
  (`toff ~= C - pre_bin`), so a tiny grid is centred there instead of searched.
- **K = 6 candidates** (2 timing offsets x 3 CFO values) run in lockstep as the
  frame streams. Each re-windows from a shared **2-symbol ring** at its own
  `toff`, dechirps with its own CFO (a complex-rotation *recurrence* — no
  per-sample trig), FFTs, and buffers only its bins. At burst end each runs the
  proven `decode_full` + CRC; the first CRC pass wins.
- **Fits the hardware, measured:** builds to **20 KB flash (62%)**; the processor
  object is **73.8 KB**, inside the **79.6 KB** heap (thread stacks are static, so
  nothing competes). Six 2048-pt FFTs/symbol sit inside SF11's 8.19 ms budget.
  int16 sample buffers and dropping the CFO-correction/preamble-magnitude buffers
  (unused here) are what bring it under the ceiling.

Validated in `lf_hostsim`: the candidate decoder decodes **both** probe bursts
byte-identical to the reference and decrypts end to end.

**SFD-tracked timing.** The frame boundary is now read per-burst from the SFD:
`STO = (pre_bin - sfd_bin)/2` is CFO-invariant, so `toff = const - STO` tracks any
transmitter's carrier offset (the SFD is cleanly detectable via the conjugate
downchirp -- up-dechirp sharpness jumps ~3 -> 1220 at it). `const` is a geometric
constant, not signal-dependent. Timing therefore generalises across nodes. The
*fractional* CFO de-rotation still uses a fixed grid centred where this hardware
sits; tracking it per-transmitter runs into a half-integer/STO-coupling subtlety
(the parabolic sub-bin estimate is unstable when the CFO lands near a half bin, as
the beacon's does) -- a bounded refinement left for real multi-node traffic.

Without `LF_CAND` or `LF_BATCH`, the streaming decoder still runs: rock-solid
detection plus a best-effort (header + addresses) decode, only CRC-verified frames
marked clean.

## Reproduce

```bash
# host reference (numpy; a Meshtastic key set for the decrypt step)
python3 host/longfast_decode.py longfast_probe.bin

# firmware DSP over the same capture, with timing recovery:
#   cp -r hostsim lf_hostsim   # then build with the LF processor
#   make -f Makefile.lf CXXFLAGS="... -DLF_BATCH -DLF_NPHASE=1 ..."
#   ./hostsim longfast_probe.bin      # P = preamble, F = CRC-clean, H = best-effort
```
