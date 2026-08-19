#!/usr/bin/env bash
# Overlay ChirpHound's sources onto a fresh mayhem-firmware checkout.
#   scripts/wire.sh /path/to/mayhem-firmware
# Copies the meshtastic_rx app + baseband DSP into place and applies the five
# wiring points (external.cmake, external.ld, baseband/CMakeLists.txt,
# common/spi_image.hpp) as a patch. Run against the pinned MAYHEM_REF.
set -euo pipefail
CH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MF="${1:?usage: wire.sh <mayhem-firmware-dir>}"

cp "$CH"/firmware/baseband/proc_meshtastic.cpp "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/proc_meshtastic.hpp "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/lora_decode.cpp     "$MF"/firmware/baseband/
cp "$CH"/firmware/baseband/lora_decode.hpp     "$MF"/firmware/baseband/
mkdir -p "$MF"/firmware/application/external/meshtastic_rx
cp "$CH"/firmware/application/external/meshtastic_rx/* \
   "$MF"/firmware/application/external/meshtastic_rx/
git -C "$MF" apply "$CH"/firmware/wiring.patch
echo "ChirpHound wired into $MF"
