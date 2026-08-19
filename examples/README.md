# Example captures

Real device logs pulled off two PortaPacks over USB serial (`../tools/serial_pull.py`).
Score them with the host tools:

```
python3 ../tools/score_log.py walkabout.txt        # the funnel
python3 ../tools/clean_beacon.py stationary.txt     # clean BEACON decodes
# with channel keys, offline decrypt (meshpipe):
python3 -m meshpipe.replay stationary.txt
```

- **stationary.txt** — a PortaPack sitting near the nodes. Decoded real
  traffic: the beacon (`BEACON 00025`), the agent `!c822ce1c`, and a live
  mesh-c2 command frame (`M1|...|sh|whoami|...`) intercepted off the air.
- **walkabout.txt** — carried around the building. **2035 preamble
  detections, 0 clean decodes** — the project thesis in one file: an
  excellent *detector*, a marginal *decoder* at range. The garbled one-off
  sender addresses are corrupted decodes, not real nodes.

Line tags: `P` preamble lock · `H` header-search fail · `F` decoded frame ·
`G <amp> <lna> <vga>` RX gain at that frame · `B/Q/R` DSP diagnostics.
