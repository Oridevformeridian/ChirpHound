/*
 * LoRa PHY decode stages: gray demap, deinterleave, Hamming, dewhitening.
 *
 * Ported from gr-lora_sdr (tapparelj), which is the reference the host-side
 * pipeline already uses to decode this site's traffic -- so a packet that
 * fails here can be compared against a known-good decode of the same air
 * rather than debugged blind on a device with no debugger.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef __LORA_DECODE_H__
#define __LORA_DECODE_H__

#include <array>
#include <cstdint>
#include <cstddef>

namespace lora {

/* LoRa's payload whitening sequence (255 bytes, applied nibble-wise). */
extern const uint8_t whitening_seq[255];

/* symbol -> gray. One xor; the whole "gray mapping" block is this line. */
inline uint16_t gray_demap(uint16_t s) {
    return static_cast<uint16_t>(s ^ (s >> 1));
}

inline int32_t mod_i(int32_t a, int32_t n) {
    return ((a % n) + n) % n;
}

/* Diagonal deinterleave of one block.
 *
 *   sf_app = (is_header || ldro) ? sf - 2 : sf     bits taken per symbol
 *   cw_len = is_header ? 8 : cr + 4                codewords out
 *
 * in  : cw_len symbols, each carrying sf_app bits
 * out : sf_app codewords of cw_len bits
 */
void deinterleave(const uint16_t* in, uint8_t* out,
                  uint8_t sf_app, uint8_t cw_len);

/* Hamming decode one codeword. cr_app is 1..4 (4/5 .. 4/8).
 * Returns the data nibble; sets *ok false when an uncorrectable error is
 * detected (only meaningful for cr_app 3 and 4, which carry a syndrome). */
uint8_t hamming_decode(uint8_t codeword, uint8_t cr_app, bool* ok);

/* Dewhiten payload nibbles in place, skipping the CRC bytes at the end.
 * Nibbles arrive low-then-high, which is why this takes nibbles and not
 * bytes: pairing them the other way silently produces plausible garbage. */
void dewhiten(const uint8_t* nibbles, size_t nibble_count,
              uint8_t* out_bytes, size_t payload_len, bool crc_present);

/* Explicit header: 5 nibbles -> length, CRC presence, coding rate.
 * Returns false if the header checksum does not verify. */
struct Header {
    uint8_t payload_len;
    uint8_t cr;
    bool crc_present;
};
bool parse_header(const uint8_t* nibbles, Header* out);

}  // namespace lora

#endif /*__LORA_DECODE_H__*/
