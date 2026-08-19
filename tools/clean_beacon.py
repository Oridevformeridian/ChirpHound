"""Count frames decoding to a CLEAN 'BEACON <5 digits>' vs garbled -- a finer
metric than binary decrypt (AES-CTR decrypts per-byte, so 1 bad symbol garbles
1 char but still 'decrypts')."""
import re, sys
sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets
chans = keys.load(keys.DEFAULT_PATH)
clean = dec = frames = 0
texts = []
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) > 2 and p[1] == "F":
        frames += 1
        try:
            rec = packets.process(bytes.fromhex(p[2]), chans)
        except Exception:
            rec = None
        if rec and rec.get("decrypted"):
            dec += 1
            t = str(rec.get("text") or "")
            texts.append(t)
            if re.fullmatch(r"BEACON \d{5}", t):
                clean += 1
print(f"frames {frames}  decrypted {dec}  CLEAN 'BEACON NNNNN' {clean}")
