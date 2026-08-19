# ChirpHound

A Meshtastic (LoRa **SHORT_TURBO**, SF7/BW500) receiver + packet **detector** for
the PortaPack Mayhem — the first working CSS demodulator to run on this hardware.
It locks LoRa preambles, decodes frames, logs everything to SD or over USB serial,
and shows a static detector panel (preambles / headers / decodes / addresses).

![ChirpHound detector screen](docs/detector.png)

*The static detector panel, captured in the field — preamble / header / decode / address counts update in place (210 preambles, 27 decodes here).*

![Live beacon decode](docs/decode.png)

*A live decode off the air: `!4358AB2C > bcast chEB 34B` — the beacon node, decrypted on the bench with the channel key.*

Two halves:
- **Firmware** — `meshtastic_rx` external app + M4 baseband (in the `meshtastic-rx`
  branch of `mayhem-v240`). Drops onto the SD card, no reflash of the base firmware.
- **Host toolchain** (this repo) — `hostsim/` compiles the *real* baseband DSP for
  x86 and runs it over captured IQ at ~90x realtime, byte-identical to on-device.
  That harness is what found every fix below.

## Results (measured)

### Demod / decode — host harness, captured IQ (controlled A/B)
`beacon_fixture2.bin` (900 s HackRF capture): **379 preamble locks, 66 frames**

| decoder | decrypted | clean `BEACON NNNNN` |
|---|---|---|
| hard-decision | 11 | 5 |
| **+ per-bit LLR soft-decision** | **15** | **13** |

`beacon_fixture1.bin` (720 s): 19 frames → soft-decision 7 decrypted, 5 clean.

### On hardware (PortaPack, live off-air)
- **First cryptographically-verified decode on the device:** `!4358ab2c ->
  broadcast [oob] TEXT 'BEACON 04797'`.
- Sync working: **~75-80 % of preamble locks reach header search** (post `sync_tol` fix).
- Best RX gain config: **amp OFF / LNA 40 / VGA 38 → 58 % header-byte accuracy**.

### The three DSP fixes (all host-validated, hardware-confirmed)
1. **payload `(bin-1) mod N`** before Gray demap — decrypts 0 → 4-11. The header
   tolerated its absence (÷4 quantised it, CR4/8 corrected it); the CR4/5 payload
   could not.
2. **`sync_tol` 2 → 3** — the grid shift lands the sync word 2-3 bins off 0x10/0x58,
   so a tolerance of 2 missed by one. Sync failures 116 → 32, decrypts 4 → 11.
3. **per-bit LLR soft-decision** (max-log demapper) — on a CR4/5 parity failure,
   flip the codeword bit with the lowest per-bit reliability. 11 → 15 decrypts,
   5 → 13 clean.

### The RF finding: front-end OVERLOAD
The receiver was *overloading*, not starving. Controlled on-device gain sweep:

| config | header bytes | decrypt |
|---|---|---|
| **RF amp ON**, LNA 32, VGA 38 | 26.8 % | 0 |
| **RF amp OFF**, LNA 40, VGA 38 | **58.0 %** | 4 |

The +14 dB amp halves decode and zeroes decryption on the strong indoor signal.
Antenna length/tuning barely mattered — the fix was **backing gain off**.

### What was ruled out (by measurement, not guessing)
Dropped samples (M4 sustains a flat 2 Msps), sample-clock drift (preamble bins
flat), carrier offset (device CFO = HackRF, ~0). The residual limit is per-symbol
SNR — an RF/front-end property, now mitigated by the amp-off config.

## Toolchain
`hostsim/` (real DSP on x86) · `serial_pull.py` (read device logs over
`/dev/ttyACM1`, no SD swap) · `score_log.py` / `clean_beacon.py` / `gain_sweep.py`
(scoring) · `record_iq.py` (fixtures) · plus the diagnostics that killed the
dead-end hypotheses (`symbol_decay`, `drift`, `dual_corr`, `snr_curve`, ...).
Reads go over serial; flashing a new build still needs the card (serial writes
wedge the CDC — do not use `fwb`).

## More

- [`docs/WIRING.md`](docs/WIRING.md) — how the app + baseband wire into Mayhem, and the signal flow.
- [`examples/`](examples/) — real field captures (stationary + walkabout) you can score with the host tools.
