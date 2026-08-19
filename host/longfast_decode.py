"""Full SF11/BW250 LongFast decode in numpy, ported from lora_decode.cpp.

The device's 256-point FFT can't do SF11, but x86 can, so prove the decode here
first. The uncertain SF11 params (LDRO on/off, header start offset, the (bin-1)
payload adjust, the quarter-symbol rotation) are SEARCHED, and a candidate is
accepted only when the 5-bit header checksum passes -- a wrong combination
passes ~1 in 32, so a hit that also decrypts is the answer.
"""
import sys
import numpy as np

sys.path.insert(0, "/var/home/mycroft/rf_expo")
from meshpipe import keys, packets

SF = 11
N = 1 << SF
DECIM = 4
RATE = 1_000_000
WHITEN = None  # filled from meshpipe/firmware if needed; we dewhiten by table below

k = np.arange(N)
CHIRP = np.exp(-1j * np.pi * k * k / N).astype(np.complex64)

# whitening sequence (same 255-byte table as lora_decode.cpp)
WHITEN = bytes([
0xFF,0xFE,0xFC,0xF8,0xF0,0xE1,0xC2,0x85,0x0B,0x17,0x2F,0x5E,0xBC,0x78,0xF1,0xE3,
0xC6,0x8D,0x1A,0x34,0x68,0xD0,0xA0,0x40,0x80,0x01,0x02,0x04,0x08,0x11,0x23,0x47,
0x8E,0x1C,0x38,0x71,0xE2,0xC4,0x89,0x12,0x25,0x4B,0x97,0x2E,0x5C,0xB8,0x70,0xE0,
0xC0,0x81,0x03,0x06,0x0C,0x19,0x32,0x64,0xC9,0x92,0x24,0x49,0x93,0x26,0x4D,0x9B,
0x37,0x6E,0xDC,0xB9,0x72,0xE4,0xC8,0x90,0x20,0x41,0x82,0x05,0x0A,0x15,0x2B,0x56,
0xAD,0x5B,0xB6,0x6D,0xDA,0xB5,0x6B,0xD6,0xAC,0x59,0xB2,0x65,0xCB,0x96,0x2C,0x58,
0xB0,0x61,0xC3,0x87,0x0F,0x1F,0x3E,0x7D,0xFB,0xF6,0xED,0xDB,0xB7,0x6F,0xDE,0xBD,
0x7A,0xF5,0xEB,0xD7,0xAE,0x5D,0xBA,0x74,0xE8,0xD1,0xA2,0x44,0x88,0x10,0x21,0x43,
0x86,0x0D,0x1B,0x36,0x6C,0xD8,0xB1,0x63,0xC7,0x8F,0x1E,0x3C,0x79,0xF3,0xE7,0xCE,
0x9C,0x39,0x73,0xE6,0xCC,0x98,0x31,0x62,0xC5,0x8B,0x16,0x2D,0x5A,0xB4,0x69,0xD2,
0xA4,0x48,0x91,0x22,0x45,0x8A,0x14,0x29,0x52,0xA5,0x4A,0x95,0x2A,0x54,0xA9,0x53,
0xA7,0x4E,0x9D,0x3B,0x77,0xEE,0xDD,0xBB,0x76,0xEC,0xD9,0xB3,0x67,0xCF,0x9E,0x3D,
0x7B,0xF7,0xEF,0xDF,0xBF,0x7E,0xFD,0xFA,0xF4,0xE9,0xD3,0xA6,0x4C,0x99,0x33,0x66,
0xCD,0x9A,0x35,0x6A,0xD4,0xA8,0x51,0xA3,0x46,0x8C,0x18,0x30,0x60,0xC1,0x83,0x07,
0x0E,0x1D,0x3A,0x75,0xEA,0xD5,0xAA,0x55,0xAB,0x57,0xAF,0x5F,0xBE,0x7C,0xF9,0xF2,
0xE5,0xCA,0x94,0x28,0x50,0xA1,0x42,0x84,0x09,0x13,0x27,0x4F,0x9F,0x3F,0x7F])


def gray_demap(s):
    return s ^ (s >> 1)


def deinterleave(inb, sf_app, cw_len):
    out = [0] * sf_app
    for i in range(cw_len):
        for j in range(sf_app):
            bit = (inb[i] >> (sf_app - 1 - j)) & 1
            row = (i - j - 1) % sf_app
            if bit:
                out[row] |= 1 << (cw_len - 1 - i)
    return out


def hamming_decode(cw, cr):
    length = cr + 4
    c = [(cw >> (length - 1 - i)) & 1 for i in range(length)]
    nib = (c[3] << 3) | (c[2] << 2) | (c[1] << 1) | c[0]
    return nib & 0xF


def checksum_ok(n):
    c4 = ((n[0]&0x8)>>3)^((n[0]&0x4)>>2)^((n[0]&0x2)>>1)^(n[0]&1)
    c3 = ((n[0]&0x8)>>3)^((n[1]&0x8)>>3)^((n[1]&0x4)>>2)^((n[1]&0x2)>>1)^(n[2]&1)
    c2 = ((n[0]&0x4)>>2)^((n[1]&0x8)>>3)^(n[1]&1)^((n[2]&0x8)>>3)^((n[2]&0x2)>>1)
    c1 = ((n[0]&0x2)>>1)^((n[1]&0x4)>>2)^(n[1]&1)^((n[2]&0x4)>>2)^((n[2]&0x2)>>1)^(n[2]&1)
    c0 = (n[0]&1)^((n[1]&0x2)>>1)^((n[2]&0x8)>>3)^((n[2]&0x4)>>2)^((n[2]&0x2)>>1)^(n[2]&1)
    want = ((n[3]&1)<<4)+n[4]
    got = (c4<<4)|(c3<<3)|(c2<<2)|(c1<<1)|c0
    return want == got


def dewhiten(nibbles, payload_len, crc):
    out = bytearray()
    off = 0
    i = 0
    total = payload_len + (2 if crc else 0)
    while i + 1 < len(nibbles) and off < total:
        if off < payload_len:
            low = nibbles[i] ^ (WHITEN[off] & 0x0F)
            high = nibbles[i+1] ^ ((WHITEN[off] & 0xF0) >> 4)
        else:
            low, high = nibbles[i], nibbles[i+1]
        out.append(((high << 4) | (low & 0x0F)) & 0xFF)
        off += 1
        i += 2
    return bytes(out)


def decimate(seg, sample_shift):
    seg = seg[sample_shift:]
    n = (len(seg) // DECIM) * DECIM
    return seg[:n].reshape(-1, DECIM).sum(1)


def estimate_cfo(dec):
    """Fractional bin offset of the preamble tone (parabolic interp of the
    averaged preamble spectrum). This is the CFO+STO fractional part that
    rounds boundary symbols the wrong way if left uncorrected."""
    ns = min(12, len(dec) // N)
    if ns < 4:
        return 0.0
    fr = dec[: ns * N].reshape(ns, N)
    mag = np.abs(np.fft.fft(fr * CHIRP, axis=1)).mean(0)
    b = int(mag.argmax())
    a0 = mag[(b - 1) % N]; a1 = mag[b]; a2 = mag[(b + 1) % N]
    den = a0 - 2 * a1 + a2
    return 0.5 * (a0 - a2) / den if abs(den) > 1e-9 else 0.0


def symbolize(seg, rotate, sample_shift=0, cfo=0.0):
    dec = decimate(seg, sample_shift)
    if cfo != 0.0:
        m = np.arange(len(dec))
        dec = dec * np.exp(-1j * 2 * np.pi * cfo * m / N).astype(np.complex64)
    ns = len(dec) // N
    fr = dec[: ns * N].reshape(ns, N)
    spec = np.fft.fft(fr * CHIRP, axis=1)
    mg = spec.real ** 2 + spec.imag ** 2
    bins = mg.argmax(1)
    sharp = mg.max(1) / np.maximum(mg.mean(1), 1e-9)
    return bins, sharp


def try_decode(seg, chans):
    for rotate in (0, N // 4):
        for shift in range(0, DECIM):
            cfo = estimate_cfo(decimate(seg, shift))
            bins, sharp = symbolize(seg, rotate, shift, cfo)
            if len(bins) < 20:
                continue
            pre = int(np.bincount(bins[:12]).argmax())
            norm = (bins.astype(int) - pre + rotate) % N
            for badj in (0, -1):
                sym = (norm + badj) % N
                # header starts somewhere after preamble(≈16)+sync(2)+sfd(2.25)
                for hstart in range(18, 30):
                    if hstart + 8 > len(sym):
                        break
                    for ldro in (True, False):
                        sf_app_h = SF - 2  # header always reduced
                        gray = [gray_demap(int(sym[hstart + i]) >> 2) for i in range(8)]
                        cw = deinterleave(gray, sf_app_h, 8)
                        nib = [hamming_decode(cw[i], 4) for i in range(sf_app_h)]
                        if not checksum_ok(nib[:5]):
                            continue
                        plen = (nib[0] << 4) | nib[1]
                        cr = (nib[2] & 0x0E) >> 1
                        crc = (nib[2] & 1) != 0
                        if cr < 1 or cr > 4 or plen == 0 or plen > 60:
                            continue
                        cw_len_p = cr + 4
                        for padj in (0, -1, 1):
                          for pldro in (True, False):
                            sf_app_p = (SF - 2) if pldro else SF
                            symp = (norm + padj) % N
                            nibs = list(nib[5:sf_app_h])
                            base = hstart + 8
                            while base + cw_len_p <= len(symp) and len(nibs) < (plen + 2) * 2 + 8:
                                g = [gray_demap(int(symp[base + i])) for i in range(cw_len_p)]
                                cwp = deinterleave(g, sf_app_p, cw_len_p)
                                nibs += [hamming_decode(cwp[i], cr) for i in range(sf_app_p)]
                                base += cw_len_p
                            raw = dewhiten(nibs, plen, crc)
                            if len(raw) < 12:
                                continue
                            rec = None
                            try:
                                rec = packets.process(bytes(raw), chans)
                            except Exception:
                                pass
                            known=bytes.fromhex("ffffffff2cab5843")
                            score=sum(1 for a,b in zip(known,raw) if a==b) if len(raw)>=8 else 0
                            yield dict(rotate=rotate, shift=shift, cfo=round(cfo,4), badj=badj,
                                       padj=padj, pldro=pldro, hstart=hstart, ldro=ldro,
                                       plen=plen, cr=cr, raw=raw.hex(), rec=rec, pre=pre, score=score)


def main():
    chans = keys.load(keys.DEFAULT_PATH)
    raw = np.fromfile(sys.argv[1], dtype=np.int8)
    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    mag = np.abs(iq[::32]); sm = np.convolve(mag, np.ones(256)/256, "same")
    db = 20*np.log10(np.maximum(sm,1e-6)); hot = db > np.percentile(db,40)+6
    bursts, s = [], None
    for i,h in enumerate(hot):
        if h and s is None: s=i
        elif not h and s is not None and i-s>100: bursts.append((s*32,i*32)); s=None
    print(f"{len(bursts)} bursts")
    hits = 0
    for bi,(bs,be) in enumerate(bursts):
        seg = iq[max(0,bs-2000):be+2000]
        best = None
        for r in try_decode(seg, chans):
            if r["rec"] and r["rec"].get("decrypted"):
                best = r; break
            if best is None or r["score"] > best["score"]:
                best = r
        if best and best['score']>=6:
            hits += 1
            rec = best["rec"]
            print(f"  burst {bi}: plen={best['plen']} cr={best['cr']} pldro={best.get('pldro')} "
                  f"shift={best.get('shift')} cfo={best.get('cfo')} rotate={best['rotate']} "
                  f"hstart={best['hstart']} hadj={best['badj']} padj={best.get('padj')} "
                  f"known-bytes {best['score']}/8")
            if rec:
                print(f"    sender !{rec.get('sender',0):08x} dest !{rec.get('dest',0):08x} "
                      f"id {rec.get('id',0):08x} hash {rec.get('chan_hash',0):02x} "
                      f"decrypted={rec.get('decrypted')} text={rec.get('text')!r}")
                print(f"    raw: {best['raw']}")
        else:
            print(f"  burst {bi}: no header/decrypt found")
    print(f"\n{hits}/{len(bursts)} bursts decoded")


if __name__ == "__main__":
    main()
