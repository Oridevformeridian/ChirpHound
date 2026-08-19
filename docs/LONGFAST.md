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

## Host vs. device

The frame-boundary sweep is behind `#define LF_BATCH` because it buffers the
burst's raw IQ (a few MB) to re-window it — fine for the host / `hostsim`, too big
for the M4's SRAM. The **device-deployable** path is the same timing recovery done
*streaming*: detect the SFD (the down-chirps that follow the preamble) to set the
frame boundary directly instead of searching it — O(1) memory. The batch search
proves the alignment exists and the DSP is correct; SFD-locked streaming is the
next step to put a clean LongFast decode on the handheld itself.

Without `LF_BATCH`, the streaming decoder still runs: **detection is rock-solid**
and it emits the near-complete decode (header + addresses) as best-effort, with
only fully CRC-verified frames marked clean.

## Reproduce

```bash
# host reference (numpy; a Meshtastic key set for the decrypt step)
python3 host/longfast_decode.py longfast_probe.bin

# firmware DSP over the same capture, with timing recovery:
#   cp -r hostsim lf_hostsim   # then build with the LF processor
#   make -f Makefile.lf CXXFLAGS="... -DLF_BATCH -DLF_NPHASE=1 ..."
#   ./hostsim longfast_probe.bin      # P = preamble, F = CRC-clean, H = best-effort
```
