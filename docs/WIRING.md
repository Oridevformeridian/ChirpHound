# Wiring: how ChirpHound integrates into Mayhem

ChirpHound is an **external app** — it drops onto the SD card, no reflash of the
base firmware needed. But an external app that carries its own baseband DSP has
to be wired into the Mayhem build at five points. `scripts/wire.sh` copies the
sources and applies `firmware/wiring.patch`, which touches exactly these:

```
1. common/spi_image.hpp
     image_tag_meshtastic{'P','M','S','H'}      # 4-char tag pairing the two halves

2. baseband/CMakeLists.txt
     DeclareTargets(PMSH meshtastic)            # builds a standalone M4 baseband
                                                # image (proc_meshtastic + lora_decode)

3. application/external/external.cmake
     EXTCPPSRC += meshtastic_rx/*.cpp           # the M0 app sources
     EXTAPPLIST += meshtastic_rx               # register the app

4. application/external/external.ld
     ram_external_app_meshtastic_rx @ 0xAE040000, 32k   # its own RAM region
     .external_app_meshtastic_rx output section          # + the section that fills it
     # miss the section and it compiles but ships empty -> IndexError at pack time

5. firmware/application/external/meshtastic_rx/main.cpp
     int main() { EventDispatcher{...}.run(); } # baseband entry point per proc
```

The `.ppma` **bundles both halves** — the M0 UI app and the M4 baseband image,
paired by the `PMSH` tag — so there is a single file to copy to `/APPS`.

## Signal flow

```
  antenna
    |                          M4 (baseband, proc_meshtastic.cpp)
    v                       .----------------------------------------.
  RX @ 917.25 MHz  2 Msps   |  decimate x4 -> 500 kHz chip rate       |
  ----------------------->  |  dechirp (conj upchirp) -> 128-pt FFT   |
                            |  peak bin = symbol                      |
                            |  preamble lock -> sync (0x10/0x58)      |
                            |  header (CR4/8) -> payload (CR4/5)      |
                            |  gray demap / deinterleave / hamming    |
                            |  per-bit LLR soft-decision              |
                            '------------------|---------------------'
                                               | emit(ACARSPacketMessage)
                                               v  (shared_memory queue, M4->M0)
                            .----------------------------------------.
                            |  M0 (app, meshtastic_app.cpp)          |
                            |  detector panel: PREAMBLE/HEADER/      |
                            |    DECODE/ADDRS/LAST                    |
                            |  log_str -> /LOGS/MESHTAST.TXT (+serial)|
                            '----------------------------------------'
```

`ACARSPacketMessage` is borrowed as the M4->M0 transport (a length + 250-byte
buffer); a proper `MeshtasticPacketMessage` id would need the base firmware
rebuilt, which would break the SD-card drop-in. The `state` field carries the
line kind (frame / preamble / header-fail / diagnostics).

## Constraints that shaped it (SF7/BW500 only, for now)

- **FFT caps at 256 points** (`dsp_fft.hpp` twiddle tables). SF7 = 128-point,
  fits; SF9+ needs new tables + RAM.
- **BW500 @ 2 Msps is exactly 4x decimation** — no resampling.
- **32 KB app RAM region** — the soft-decision LLR + magnitude buffers fit
  (~2 KB); larger SF would not without care.
