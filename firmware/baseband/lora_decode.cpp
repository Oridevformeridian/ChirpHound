/*
 * LoRa PHY decode stages. See lora_decode.hpp for provenance.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "lora_decode.hpp"

namespace lora {

const uint8_t whitening_seq[255] = {
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E,
    0xBC, 0x78, 0xF1, 0xE3, 0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40,
    0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47, 0x8E, 0x1C, 0x38, 0x71,
    0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C, 0xB8, 0x70, 0xE0,
    0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49,
    0x93, 0x26, 0x4D, 0x9B, 0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90,
    0x20, 0x41, 0x82, 0x05, 0x0A, 0x15, 0x2B, 0x56, 0xAD, 0x5B, 0xB6, 0x6D,
    0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59, 0xB2, 0x65, 0xCB, 0x96, 0x2C, 0x58,
    0xB0, 0x61, 0xC3, 0x87, 0x0F, 0x1F, 0x3E, 0x7D, 0xFB, 0xF6, 0xED, 0xDB,
    0xB7, 0x6F, 0xDE, 0xBD, 0x7A, 0xF5, 0xEB, 0xD7, 0xAE, 0x5D, 0xBA, 0x74,
    0xE8, 0xD1, 0xA2, 0x44, 0x88, 0x10, 0x21, 0x43, 0x86, 0x0D, 0x1B, 0x36,
    0x6C, 0xD8, 0xB1, 0x63, 0xC7, 0x8F, 0x1E, 0x3C, 0x79, 0xF3, 0xE7, 0xCE,
    0x9C, 0x39, 0x73, 0xE6, 0xCC, 0x98, 0x31, 0x62, 0xC5, 0x8B, 0x16, 0x2D,
    0x5A, 0xB4, 0x69, 0xD2, 0xA4, 0x48, 0x91, 0x22, 0x45, 0x8A, 0x14, 0x29,
    0x52, 0xA5, 0x4A, 0x95, 0x2A, 0x54, 0xA9, 0x53, 0xA7, 0x4E, 0x9D, 0x3B,
    0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xEC, 0xD9, 0xB3, 0x67, 0xCF, 0x9E, 0x3D,
    0x7B, 0xF7, 0xEF, 0xDF, 0xBF, 0x7E, 0xFD, 0xFA, 0xF4, 0xE9, 0xD3, 0xA6,
    0x4C, 0x99, 0x33, 0x66, 0xCD, 0x9A, 0x35, 0x6A, 0xD4, 0xA8, 0x51, 0xA3,
    0x46, 0x8C, 0x18, 0x30, 0x60, 0xC1, 0x83, 0x07, 0x0E, 0x1D, 0x3A, 0x75,
    0xEA, 0xD5, 0xAA, 0x55, 0xAB, 0x57, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2,
    0xE5, 0xCA, 0x94, 0x28, 0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F,
    0x9F, 0x3F, 0x7F,
};

void deinterleave(const uint16_t* in, uint8_t* out,
                  uint8_t sf_app, uint8_t cw_len) {
    for (uint8_t i = 0; i < sf_app; i++) out[i] = 0;

    /* deinter[(i - j - 1) mod sf_app][i] = inter[i][j]
     *
     * inter_bin[i] is symbol i expressed as sf_app bits, MSB first. The
     * modular row index is the diagonal: get the sign of the shift wrong and
     * every codeword still *looks* well-formed, which is why this is copied
     * verbatim from the reference rather than re-derived.
     */
    for (int32_t i = 0; i < cw_len; i++) {
        for (int32_t j = 0; j < sf_app; j++) {
            const uint8_t bit = (in[i] >> (sf_app - 1 - j)) & 1u;
            const int32_t row = mod_i(i - j - 1, sf_app);
            if (bit) out[row] |= static_cast<uint8_t>(1u << (cw_len - 1 - i));
        }
    }
}

uint8_t hamming_decode(uint8_t cw, uint8_t cr_app, bool* ok) {
    if (ok) *ok = true;

    /* Codeword bits, **MSB-first**, which is how the reference indexes them.
     *
     * gr-lora_sdr does int2bool(cw, cr_app+4) and then takes
     * {codeword[3], codeword[2], codeword[1], codeword[0]} as the data nibble.
     * int2bool emits most-significant bit first, so those are the *top* four
     * bits of the codeword. Indexing LSB-first here decoded the wrong four
     * bits and fed the syndrome equations the wrong positions -- the whole
     * chain then produces plausible-looking nonsense rather than an obvious
     * failure, which is exactly what makes this kind of bug expensive.
     */
    bool c[8];
    const uint8_t len = static_cast<uint8_t>(cr_app + 4);
    for (uint8_t i = 0; i < len; i++)
        c[i] = (cw >> (len - 1 - i)) & 1u;
    for (uint8_t i = len; i < 8; i++)
        c[i] = 0;

    /* data nibble is bits 0..3, reorganised MSB-first */
    uint8_t nib = static_cast<uint8_t>((c[3] << 3) | (c[2] << 2) |
                                       (c[1] << 1) | c[0]);

    if (cr_app == 3 || cr_app == 4) {
        const bool s0 = c[0] ^ c[1] ^ c[2] ^ c[4];
        const bool s1 = c[1] ^ c[2] ^ c[3] ^ c[5];
        const bool s2 = c[0] ^ c[1] ^ c[3] ^ c[6];
        const uint8_t syn = static_cast<uint8_t>(s0 | (s1 << 1) | (s2 << 2));
        if (syn) {
            /* Single-bit correction. The mapping from syndrome to data bit is
             * the reference's; anything not in it is a multi-bit error we
             * cannot fix, and saying so beats silently emitting a wrong byte. */
            switch (syn) {
                case 5: nib ^= 0b0001; break;  /* d3 */
                case 7: nib ^= 0b0010; break;  /* d2 */
                case 3: nib ^= 0b0100; break;  /* d1 */
                case 6: nib ^= 0b1000; break;  /* d0 */
                default:
                    if (ok) *ok = false;
                    break;
            }
        }
    } else {
        /* 4/5 and 4/6 carry parity only: detection, no correction. */
        bool parity = 0;
        for (uint8_t i = 0; i < len; i++) parity ^= c[i];
        if (parity && ok) *ok = false;
    }
    return static_cast<uint8_t>(nib & 0x0F);
}

void dewhiten(const uint8_t* nibbles, size_t nibble_count,
              uint8_t* out_bytes, size_t payload_len, bool crc_present) {
    size_t offset = 0;
    for (size_t i = 0; i + 1 < nibble_count && offset < payload_len + (crc_present ? 2u : 0u); i += 2) {
        uint8_t low, high;
        if (offset < payload_len) {
            low = static_cast<uint8_t>(nibbles[i] ^ (whitening_seq[offset] & 0x0F));
            high = static_cast<uint8_t>(nibbles[i + 1] ^ ((whitening_seq[offset] & 0xF0) >> 4));
        } else {
            /* The CRC bytes are not whitened. */
            low = nibbles[i];
            high = nibbles[i + 1];
        }
        out_bytes[offset] = static_cast<uint8_t>((high << 4) | (low & 0x0F));
        offset++;
    }
}

static bool checksum_ok(const uint8_t* n) {
    /* LoRa's 5-bit explicit-header checksum. Omitting it made header
     * validation far too permissive -- "cr==1 and a plausible length" passes
     * by chance roughly 1 trial in 10, so an offset search reported ten
     * candidates and picked the wrong one. This makes acceptance ~32x more
     * selective. */
    const bool c4 = ((n[0] & 0x8) >> 3) ^ ((n[0] & 0x4) >> 2) ^ ((n[0] & 0x2) >> 1) ^ (n[0] & 1);
    const bool c3 = ((n[0] & 0x8) >> 3) ^ ((n[1] & 0x8) >> 3) ^ ((n[1] & 0x4) >> 2) ^ ((n[1] & 0x2) >> 1) ^ (n[2] & 1);
    const bool c2 = ((n[0] & 0x4) >> 2) ^ ((n[1] & 0x8) >> 3) ^ (n[1] & 1) ^ ((n[2] & 0x8) >> 3) ^ ((n[2] & 0x2) >> 1);
    const bool c1 = ((n[0] & 0x2) >> 1) ^ ((n[1] & 0x4) >> 2) ^ (n[1] & 1) ^ ((n[2] & 0x4) >> 2) ^ ((n[2] & 0x2) >> 1) ^ (n[2] & 1);
    const bool c0 = (n[0] & 1) ^ ((n[1] & 0x2) >> 1) ^ ((n[2] & 0x8) >> 3) ^ ((n[2] & 0x4) >> 2) ^ ((n[2] & 0x2) >> 1) ^ (n[2] & 1);
    const uint8_t want = static_cast<uint8_t>(((n[3] & 1) << 4) + n[4]);
    const uint8_t got = static_cast<uint8_t>((c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0);
    return want == got;
}

bool parse_header(const uint8_t* n, Header* out) {
    /* Explicit header, 5 nibbles:
     *   n0,n1 = payload length (high, low)
     *   n2    = coding rate (bits 3..1), CRC presence (bit 0)
     *   n3,n4 = checksum
     */
    const uint8_t len = static_cast<uint8_t>((n[0] << 4) | n[1]);
    const uint8_t cr = static_cast<uint8_t>((n[2] & 0x0E) >> 1);
    const bool crc = (n[2] & 0x01) != 0;

    if (cr < 1 || cr > 4) return false;
    if (len == 0) return false;
    if (!checksum_ok(n)) return false;

    out->payload_len = len;
    out->cr = cr;
    out->crc_present = crc;
    return true;
}

}  // namespace lora
