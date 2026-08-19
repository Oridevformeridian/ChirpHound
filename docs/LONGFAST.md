# LongFast (SF11 / BW250) — the DEFAULT-preset port

ChirpHound started on **SHORT_TURBO** (SF7/BW500) because it's the fastest preset
to iterate on. But the preset that actually matters is **LongFast** — Meshtastic's
DEFAULT, the one real public mesh traffic uses. SF11 carries ~12 dB more
processing gain than SF7, and that is exactly the margin the PortaPack's modest
front end needs: where SHORT_TURBO preambles were marginal, LongFast preambles
lock rock-solid.

## What it took

**The FFT.** SF11 needs a 2048-point FFT; the stock `dsp_fft` twiddle table
(`common/dsp_fft.hpp`) capped out at 256 (`K_max = 8`). The FFT itself is generic
— only the `wp_table` (each entry `exp(-iπ/2^k) - 1`) needed three more rows:

```cpp
constexpr size_t K_max = 11;   // was 8
...
{-7.52981608554970094e-05f, -0.0122715382857199254f},   //  512
{-1.88247173988909111e-05f, -0.00613588464915447527f},   // 1024
{-4.70619042380882036e-06f, -0.00306795676296597614f},   // 2048
```

That is the whole FFT change — no new radix, no new routine. SF11 is *slow*
(122 symbols/s, 8.19 ms/symbol), so even a 2048-point FFT fits the per-symbol
budget with room for several decimation phases in parallel.

**The decoder.** `baseband/proc_meshtastic_lf.{hpp,cpp}` is the streaming SF11
receiver: preamble lock → fractional-CFO estimate off the preamble → symbol
capture → header (reduced-rate `sf_app = SF-2`, `sym >> 2`) → payload
(`sf_app = SF`, LDRO off, `padj = -1`) → Hamming/dewhiten via the shared
`lora_decode` primitives → **LoRa CRC-16** gate. All of the uncertain SF11
parameters were pinned first in the host reference decoder
(`host/longfast_decode.py`, pure numpy) against a real off-air capture, then
ported to the firmware DSP.

**A CRC bug worth noting.** `lora::crc16_ok` (sx127x LoRa CRC-16, poly 0x1021)
must run over the first `plen-2` payload bytes and fold in the last two — an
easy off-by-two that silently fails *every* frame, including perfect ones.
Fixed and unit-checked against known-good symbols.

## Where it lands

Validated with the `lf_hostsim` harness (the SHORT_TURBO `hostsim` retargeted to
the LF processor at 1 Msps), byte-for-byte against `longfast_decode.py`:

- **Detection: rock-solid.** Every burst locks; the SF11 preamble is unmistakable.
- **Decode: the firmware DSP is correct.** On the reference capture the streaming
  C++ reproduces the validated decoder's symbol stream **byte-identical for 42 of
  43 symbols per burst** — the extended FFT, dechirp, normalization and
  `lora_decode` all check out.
- **CRC-clean frames** require every symbol to land. A single-pass real-time
  decoder loses ~1 marginal symbol/burst on this front end (a windowing/timing
  near-tie the offline exhaustive search wins but streaming doesn't). Those
  frames still ship as **best-effort** (header + addresses) for the log; only
  fully CRC-verified frames are marked clean.

So LongFast is the same honest story as SHORT_TURBO — **a great detector and a
best-effort decoder** — but far closer to clean, thanks to the SF11 gain. Closing
the last symbol is a symbol-timing-recovery problem, not a correctness one:
`longfast_decode.py` proves the capture decodes and decrypts end to end.

## Reproduce

```bash
# host reference (needs numpy; a Meshtastic key set for the decrypt step)
python3 host/longfast_decode.py longfast_probe.bin

# firmware DSP over the same capture (retarget hostsim to the LF processor)
#   cp -r hostsim lf_hostsim && build proc_meshtastic_lf.cpp + lora_decode.cpp
#   ./hostsim longfast_probe.bin      # P = preamble, F = CRC-clean, H = best-effort
```
