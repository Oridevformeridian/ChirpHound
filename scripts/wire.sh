#!/usr/bin/env bash
# Overlay ChirpHound's sources onto a fresh mayhem-firmware checkout.
#   scripts/wire.sh /path/to/mayhem-firmware
# Copies the SHORT_TURBO + LongFast apps and baseband DSP into place, applies the
# SHORT_TURBO wiring as a patch, then inserts the LongFast wiring (PMLF baseband
# target + meshtastic_lf_rx app) idempotently. Run against the pinned MAYHEM_REF.
set -euo pipefail
CH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MF="${1:?usage: wire.sh <mayhem-firmware-dir>}"

# baseband DSP (both presets share lora_decode)
cp "$CH"/firmware/baseband/proc_meshtastic.cpp    "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/proc_meshtastic.hpp    "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/proc_meshtastic_lf.cpp "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/proc_meshtastic_lf.hpp "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/lora_decode.cpp        "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/lora_decode.hpp        "$MF"/firmware/baseband/

# M0 apps
for app in meshtastic_rx meshtastic_lf_rx; do
  mkdir -p "$MF"/firmware/application/external/$app
  cp "$CH"/firmware/application/external/$app/* "$MF"/firmware/application/external/$app/
done

# SHORT_TURBO wiring (patch) + the SF11 FFT twiddle extension
git -C "$MF" apply "$CH"/firmware/wiring.patch
git -C "$MF" apply "$CH"/firmware/dsp_fft_sf11.patch

# LongFast wiring (idempotent inserts)
python3 - "$MF" <<'PY'
import sys, io
mf = sys.argv[1]
def edit(path, anchor, insert, tag):
    s = open(path).read()
    if tag in s:
        return
    assert anchor in s, f"anchor not found in {path}: {anchor[:40]!r}"
    s = s.replace(anchor, anchor + insert, 1)
    open(path, "w").write(s)

# 1) baseband target PMLF
edit(mf + "/firmware/baseband/CMakeLists.txt",
     "DeclareTargets(PMSH meshtastic)\n",
     "\n### MESHTASTIC LONGFAST RX\n\nset(MODE_CPPSRC\n\tproc_meshtastic_lf.cpp\n\tlora_decode.cpp\n)\nDeclareTargets(PMLF meshtasticlf)\n",
     "PMLF meshtasticlf")

# 2) external.cmake: sources + app list
edit(mf + "/firmware/application/external/external.cmake",
     "\texternal/meshtastic_rx/meshtastic_app.cpp\n",
     "\n\t#meshtastic longfast\n\texternal/meshtastic_lf_rx/main.cpp\n\texternal/meshtastic_lf_rx/meshtastic_app.cpp\n",
     "meshtastic_lf_rx/main.cpp")
edit(mf + "/firmware/application/external/external.cmake",
     "\tmeshtastic_rx\n",
     "\tmeshtastic_lf_rx\n",
     "\tmeshtastic_lf_rx\n")

# 3) external.ld: RAM region + section
edit(mf + "/firmware/application/external/external.ld",
     "    ram_external_app_meshtastic_rx         (rwx) : org = 0xAE040000, len = 32k\n",
     "    ram_external_app_meshtastic_lf_rx      (rwx) : org = 0xAE050000, len = 32k\n",
     "ram_external_app_meshtastic_lf_rx")
edit(mf + "/firmware/application/external/external.ld",
     "    } > ram_external_app_meshtastic_rx\n",
     "\n    .external_app_meshtastic_lf_rx : ALIGN(4) SUBALIGN(4)\n    {\n        KEEP(*(.external_app.app_meshtastic_lf_rx.application_information));\n        *(*ui*external_app*meshtastic_lf_rx*);\n    } > ram_external_app_meshtastic_lf_rx\n",
     ".external_app_meshtastic_lf_rx")

# 4) spi_image.hpp: image tag
edit(mf + "/firmware/common/spi_image.hpp",
     "constexpr image_tag_t image_tag_meshtastic{'P', 'M', 'S', 'H'};\n",
     "constexpr image_tag_t image_tag_meshtastic_lf{'P', 'M', 'L', 'F'};\n",
     "image_tag_meshtastic_lf")
print("LongFast wiring inserted")
PY
echo "ChirpHound (SHORT_TURBO + LongFast) wired into $MF"
