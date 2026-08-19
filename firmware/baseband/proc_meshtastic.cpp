/*
 * Meshtastic / LoRa receiver, baseband half (M4).
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "portapack_shared_memory.hpp"
#include "proc_meshtastic.hpp"
#include "event_m4.hpp"

#include <cmath>

#ifndef CFO_CORRECT
#define CFO_CORRECT 0
#endif
#ifndef CFO_SIGN
#define CFO_SIGN (+1)
#endif
#ifndef SOFT_DECODE
#define SOFT_DECODE 1
#endif

/* Payload rotation adjustment, swept by the host harness. The header path was
 * solved empirically and its bin_rotate is calibrated for it; the payload runs
 * the same normalisation and needs its own value, so it is a parameter here
 * rather than a constant hidden in an expression. */
#ifndef PAYLOAD_ROT_ADJ
#define PAYLOAD_ROT_ADJ (-1)
#endif

MeshtasticProcessor::MeshtasticProcessor() {
    /* Reference chirps, computed once. A LoRa upchirp sweeps the whole
     * bandwidth across one symbol, so its phase is quadratic: phi(k) =
     * pi*k^2/N. Multiplying a received symbol by the conjugate collapses the
     * sweep into a single tone whose FFT bin is the symbol value.
     *
     * Both directions are needed: the downchirp reference demodulates data
     * symbols, and the upchirp reference is what makes the SFD's *down*chirps
     * collapse to a bin, which is how frame alignment is found.
     */
    /* Chirp rate must scale with oversampling.
     *
     * The sweep covers the bandwidth once per symbol, so the quadratic phase
     * is pi*k^2 / (O^2 * 2^sf) -- i.e. pi*k^2 / (fft_n * O), since
     * fft_n = O * 2^sf. At O=1 that is pi*k^2/128, which is what the working
     * 1x build used. Going to O=2 doubled fft_n to 256 but the denominator
     * needs to be 512, so leaving this as pi*k^2/fft_n made the reference
     * sweep at twice the correct rate and it matched nothing at all: no
     * preamble, no display, a total loss of detection rather than a
     * degradation. A reference chirp that is wrong does not degrade
     * gracefully.
     */
    for (size_t k = 0; k < fft_n; k++) {
        const float phase = 3.14159265358979f * static_cast<float>(k * k) /
                            static_cast<float>(fft_n * oversample);
        downchirp[k] = std::complex<float>(cosf(-phase), sinf(-phase));
        upchirp[k] = std::complex<float>(cosf(phase), sinf(phase));
    }
    downchirp_corr = downchirp;   /* until a lock refines it */
    reset();
}

void MeshtasticProcessor::reset() {
    pre_bins_n = 0;
    cfo_sum = 0.0f;
    cfo_n = 0;
    window_fill = 0;
    decim_phase = 0;
    decim_acc = {0.0f, 0.0f};
    last_bin = 0;
    preamble_run = 0;
    sym_count = 0;
    sfd_wait = 0;
    resync_skip = 0;
    sfd_tail = 0;
    sync_stage = 0;
    timing_err = 0.0f;
    timing_hold = 0;
    phase_offset = 0;
    phase_probe = 0;
    phase_best = 0.0f;
    phase_best_idx = 0;
    phase_tested = 0;
    phase_skip = 0;
    state = State::Search;
}

uint32_t MeshtasticProcessor::peak_bin(
    const std::array<std::complex<float>, fft_n>& src, float* sharpness,
    float* timing) {
    fft_swap(src, scratch);
    fft_c_preswapped(scratch, 0, fft_k);

    float best = 0.0f, total = 0.0f;
    uint32_t best_bin = 0;
    for (size_t k = 0; k < fft_n; k++) {
        const float re = scratch[k].real();
        const float im = scratch[k].imag();
        const float mag = re * re + im * im;
        mag_buf[k] = mag;
        total += mag;
        if (mag > best) {
            best = mag;
            best_bin = static_cast<uint32_t>(k);
        }
    }
    peak_mag = best;
    /* A chirp collapses to one bin; noise spreads over all of them. Peak-to-
     * mean is what separates "a symbol" from "the largest of 128 noise bins",
     * which would otherwise yield a steady stream of plausible garbage. */
    const float mean = total / static_cast<float>(fft_n);
    *sharpness = (mean > 0.0f) ? (best / mean) : 0.0f;

    if (timing != nullptr) {
        /* Neighbour imbalance = signed fractional timing error. */
        const size_t lo = (best_bin + fft_n - 1) % fft_n;
        const size_t hi = (best_bin + 1) % fft_n;
        const float ml = scratch[lo].real() * scratch[lo].real() +
                         scratch[lo].imag() * scratch[lo].imag();
        const float mh = scratch[hi].real() * scratch[hi].real() +
                         scratch[hi].imag() * scratch[hi].imag();
        /* Parabolic peak interpolation on magnitudes: the vertex offset is
         * the fractional bin position, i.e. the CFO/timing error in bins. */
        const float a = sqrtf(ml), b = sqrtf(best), c = sqrtf(mh);
        const float denom = a - 2.0f * b + c;
        *timing = (fabsf(denom) > 1e-6f) ? (0.5f * (a - c) / denom) : 0.0f;
    }
    return best_bin;
}

void MeshtasticProcessor::emit(uint8_t kind, const uint8_t* data, uint8_t len) {
    if (len > 240) len = 240;
    message.msg_len = len;
    message.state = kind;
    for (uint8_t i = 0; i < len; i++)
        message.message[i] = static_cast<char>(data[i]);
    shared_memory.application_queue.push(message);
}

bool MeshtasticProcessor::try_header_at(size_t off) {
    const uint8_t sf_app = static_cast<uint8_t>(sf - 2);
    const uint8_t cw_len = 8;
    uint16_t gray[8];
    for (size_t i = 0; i < cw_len; i++)
        gray[i] = lora::gray_demap(static_cast<uint16_t>(symbols[off + i] / 4));
    uint8_t cw[8] = {0};
    lora::deinterleave(gray, cw, sf_app, cw_len);
    uint8_t nib[8] = {0};
    for (uint8_t i = 0; i < sf_app; i++) {
        bool ok = true;
        nib[i] = lora::hamming_decode(cw[i], 4, &ok);
    }
    return lora::parse_header(nib, &header);
}

void MeshtasticProcessor::on_header_complete() {
    /* Try each candidate offset; the checksum decides. */
    bool found = false;
    for (size_t off = 0; off + 8 <= sym_count && off < hdr_search; off++) {
        if (try_header_at(off)) {
            hdr_offset = off;
            found = true;
            break;
        }
    }

    if (!found) {
        /* Report the whole search window, not a placeholder.
         *
         * The previous version emitted zeroed nibbles here, which displayed as
         * "hdr 00000" and read like a decode result -- it only ever meant "no
         * offset verified". Sending all 16 symbols instead lets the same
         * offline solver that cracked the alignment be pointed straight at
         * live-path symbols, which is the one thing that has reliably made
         * progress today.
         */
        uint8_t dbg[20];
        dbg[0] = 0xE3;  /* search failed, symbols follow */
        const size_t n = (sym_count < 16) ? sym_count : 16;
        for (size_t i = 0; i < n; i++)
            dbg[1 + i] = static_cast<uint8_t>(symbols[i] & 0xFF);
        emit(6, dbg, static_cast<uint8_t>(1 + n));
        reset();
        return;
    }

    const int32_t bits = 8 * static_cast<int32_t>(header.payload_len) -
                         4 * static_cast<int32_t>(sf) + 28 +
                         (header.crc_present ? 16 : 0);
    const int32_t den = 4 * static_cast<int32_t>(sf - 2);
    int32_t blocks = (bits + den - 1) / den;
    if (blocks < 0) blocks = 0;
    payload_symbols_needed = hdr_offset + 8 +
                             static_cast<size_t>(blocks) * (header.cr + 4);
    if (payload_symbols_needed > max_symbols) payload_symbols_needed = max_symbols;
    state = State::Payload;
}

#if SOFT_DECODE
/* Per-bit reliabilities of one payload symbol, in the codeword-bit domain
 * (after normalise + PAYLOAD_ROT_ADJ + gray_demap), quantised to 0..127. */
void MeshtasticProcessor::compute_bit_rel(size_t slot) {
    const uint8_t sf_app = static_cast<uint8_t>(sf);
    /* Codeword-domain value of the peak (the bits the deinterleaver reads). */
    auto fv = [&](uint32_t bin) -> uint16_t {
        const uint32_t nv = (((bin + fft_n - (preamble_bin % fft_n) + bin_rotate)
                              % fft_n) / oversample) % symbol_max;
        return lora::gray_demap(
            static_cast<uint16_t>((nv + 2 * fft_n + (PAYLOAD_ROT_ADJ)) % fft_n));
    };
    const uint16_t P = fv(last_peak_bin);
    float other[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (size_t k = 0; k < fft_n; k++) {
        const uint16_t g = fv(static_cast<uint32_t>(k));
        const float m = mag_buf[k];
        for (uint8_t b = 0; b < sf_app; b++) {
            const uint8_t sh = static_cast<uint8_t>(sf_app - 1 - b);
            if (((g >> sh) & 1u) != ((P >> sh) & 1u) && m > other[b])
                other[b] = m;
        }
    }
    for (uint8_t b = 0; b < sf_app; b++) {
        float r = (peak_mag > 0.0f) ? (peak_mag - other[b]) / peak_mag : 1.0f;
        if (r < 0.0f) r = 0.0f;
        bit_rel[slot][b] = static_cast<int8_t>(r * 127.0f);
    }
}
#endif

void MeshtasticProcessor::on_payload_complete() {
    const uint8_t sf_app = static_cast<uint8_t>(sf);  /* LDRO off at SF7 */
    const uint8_t cw_len = static_cast<uint8_t>(header.cr + 4);

    size_t nib_count = 0;
    for (size_t base = hdr_offset + 8; base + cw_len <= sym_count; base += cw_len) {
        uint16_t gray[8];
        for (size_t i = 0; i < cw_len; i++)
            /* (bin - 1) mod N before Gray demapping, the gr-lora_sdr
             * convention. The header path survives without it because its /4
             * quantises the offset away 3 times in 4 and CR4/8 corrects the
             * rest; the payload at CR4/5 cannot correct anything, so the same
             * off-by-one shows up as scattered single-bit errors. */
            gray[i] = lora::gray_demap(static_cast<uint16_t>(
                (symbols[base + i] + 2 * fft_n + (PAYLOAD_ROT_ADJ)) % fft_n));

        uint8_t cw[8] = {0};
        lora::deinterleave(gray, cw, sf_app, cw_len);
#if SOFT_DECODE
        /* CR4/5 parity can detect but not locate a single-bit error. Locate it
         * with the per-bit reliabilities: codeword row R, column i reads
         * symbol (base+i)'s bit j = (i - R - 1) mod sf_app. Flip the column
         * whose contributing bit is least reliable. */
        if (header.cr == 1) {
            for (uint8_t row = 0; row < sf_app; row++) {
                uint8_t par = 0;
                for (uint8_t b = 0; b < cw_len; b++) par ^= (cw[row] >> b) & 1u;
                if (!par) continue;
                int worst = 1000;
                uint8_t imin = 0;
                for (uint8_t i = 0; i < cw_len; i++) {
                    const uint8_t j = static_cast<uint8_t>(
                        lora::mod_i(static_cast<int32_t>(i) - row - 1, sf_app));
                    const int rel = bit_rel[base + i][j];
                    if (rel < worst) { worst = rel; imin = i; }
                }
                cw[row] ^= static_cast<uint8_t>(1u << (cw_len - 1 - imin));
            }
        }
#endif
        for (uint8_t i = 0; i < sf_app && nib_count < nibbles.size(); i++) {
            bool ok = true;
            nibbles[nib_count++] = lora::hamming_decode(cw[i], header.cr, &ok);
        }
    }

    lora::dewhiten(nibbles.data(), nib_count, bytes.data(),
                   header.payload_len, header.crc_present);

    frames++;
    /* Meshtastic's first 16 bytes are plaintext -- dest, sender, id, flags,
     * channel hash -- so addresses are readable with no key at all. Ship the
     * whole payload and let the app decide what to show. */
    emit(2, bytes.data(), header.payload_len);
    reset();
}

void MeshtasticProcessor::process_symbol() {
    float sharp = 0.0f;

    const std::array<std::complex<float>, fft_n>& dc =
        (state == State::Search) ? downchirp : downchirp_corr;
    for (size_t k = 0; k < fft_n; k++)
        dechirped[k] = window[k] * dc[k];
    float terr = 0.0f;
    const uint32_t bin = peak_bin(dechirped, &sharp, &terr);

    /* No timing loop. One was tried: after each symbol it measured the
     * neighbour-bin imbalance and dropped or repeated a sample. It made
     * things dramatically worse (1 frame vs 8-14), and the diagnosis was
     * plain -- not one search window verified at any offset afterwards.
     *
     * Two lessons, both paid for:
     *   - the implementation was asymmetric (it could drop samples but its
     *     "repeat" branch was a no-op), so it walked alignment out steadily;
     *   - and it was unnecessary. Genuine headers sit at exactly sync+4 in
     *     every capture that contains one, so alignment is not what is
     *     failing. What fails is symbol quality: 5 of 9 captures hold no
     *     decodable header at any offset or base, and no amount of tracking
     *     recovers those.
     */

    switch (state) {
        case State::Search: {
            /* Score this phase, then step to the next. Over consecutive
             * preamble symbols every phase gets measured against the same
             * signal, and the sharpest peak is the best-aligned one. */
            if (sharp > phase_best) {
                phase_best = sharp;
                phase_best_idx = phase_probe;
            }
            if (++phase_tested >= phase_count) {
                phase_tested = 0;
                phase_probe = (phase_probe + 1) % phase_count;
            }

            if (sharp <= 8.0f) {
                preamble_run = 0;
                pre_bins_n = 0;
                return;
            }
            const uint32_t d = (bin > last_bin) ? bin - last_bin : last_bin - bin;
            if (preamble_run > 0 && d <= bin_tolerance) {
                preamble_run++;
            } else {
                preamble_run = 1;
                pre_bins_n = 0;   /* fresh candidate: restart the history */
            }
            if (pre_bins_n < pre_bins_max)
                pre_bins[pre_bins_n++] = static_cast<uint8_t>(bin);
            cfo_sum += terr;   /* fractional offset of this preamble symbol */
            cfo_n++;
            last_bin = bin;
            if (preamble_run >= preamble_min) {
                emit(10, pre_bins, pre_bins_n);  /* preamble bins, for SFO */
                /* Always measured and logged, even when correction is off, so
                 * a hardware run reveals how much CFO the device actually has
                 * -- the one thing the HackRF-fed host harness cannot show. */
                cfo_est = (cfo_n > 0) ? (cfo_sum / static_cast<float>(cfo_n))
                                      : 0.0f;
                {
                    float q = (cfo_est + 0.5f) * 255.0f;
                    if (q < 0.0f) q = 0.0f;
                    if (q > 255.0f) q = 255.0f;
                    uint8_t qb = static_cast<uint8_t>(q);  /* 128 = zero CFO */
                    emit(11, &qb, 1);
                }
#if CFO_CORRECT
                for (size_t k = 0; k < fft_n; k++) {
                    const float ph = CFO_SIGN * 2.0f * 3.14159265358979f *
                                     cfo_est * static_cast<float>(k) /
                                     static_cast<float>(fft_n);
                    downchirp_corr[k] = downchirp[k] *
                        std::complex<float>(cosf(ph), sinf(ph));
                }
#endif
                detections++;
                uint8_t dbg[4] = {static_cast<uint8_t>(bin),
                                  static_cast<uint8_t>(preamble_run),
                                  static_cast<uint8_t>(detections),
                                  static_cast<uint8_t>(detections >> 8)};
                emit(1, dbg, 4);
                /* Align to the symbol boundary before looking for the SFD.
                 *
                 * The window was never aligned: the stream is chopped into
                 * fixed 128-sample frames, so each one straddles a symbol
                 * edge. The offset is constant within a packet -- which is why
                 * six consecutive symbols agree and the preamble locks at all
                 * -- but the peak bin *is* that offset. Field evidence: seven
                 * locks at bins 79, 20, 4, 29, 6, 74, 43, when a preamble is by
                 * definition a run of identical symbols. Scattered bins meant
                 * scattered alignment, and the SFD straddled windows too, so
                 * the up/down comparison could never resolve (6 of 7 timed out).
                 *
                 * A preamble symbol is 0, so its measured bin is pure offset.
                 * Dropping that many decimated samples snaps the frame onto the
                 * boundary the rest of the chain assumes.
                 */
                /* Adopt the best-aligned phase for the rest of the packet. */
                phase_offset = phase_best_idx;
                phase_skip = phase_offset;  /* realign the grid */
                preamble_bin = bin;
                sync_stage = 0;
                sfd_wait = 0;
                sym_count = 0;
                /* Hunt the sync word rather than correlate downchirps.
                 * Downchirp correlation timed out on 13 of 16 real packets;
                 * the sync symbols are unmistakable once the preamble bin is
                 * subtracted, and they appeared in every capture. */
                /* 1x, with the parameters measured from real captures:
                 * sync at +0x10/+0x58 relative to the preamble bin, header two
                 * symbols later, +N/4 rotation. 2x oversampling was tried and
                 * reverted -- it cut the coherent preamble run from 8 symbols
                 * to 2, losing lock stability for timing resolution we could
                 * not use. */
                state = State::Sync;
                /* Logged as well as shown: an empty log previously could not
                 * distinguish "no signal" from "signal seen, SFD missed". */
                emit(4, dbg, 4);
            }
            break;
        }

        case State::Sync: {
            const uint32_t rel = (bin + fft_n - (preamble_bin % fft_n)) % fft_n;
            const uint32_t d1 = (rel > sync_sym_1) ? rel - sync_sym_1 : sync_sym_1 - rel;
            const uint32_t d2 = (rel > sync_sym_2) ? rel - sync_sym_2 : sync_sym_2 - rel;
            if (sync_stage == 0) {
                if (d1 <= sync_tol) sync_stage = 1;
                else if (rel > preamble_tol && rel < fft_n - preamble_tol
                         && ++sfd_wait > 20) reset();  /* not preamble */
            } else {
                if (d2 <= sync_tol) {
                    sfd_wait = sfd_symbols;
                    state = State::Sfd;
                } else {
                    sync_stage = 0;
                    if (++sfd_wait > 16) reset();
                }
            }
            break;
        }

        case State::Sfd:
            if (sfd_wait > 0) { sfd_wait--; return; }
            sym_count = 0;
            state = State::Header;
            return;

        case State::Capture:
            if (sym_count < capture_len)
                symbols[sym_count++] = static_cast<uint16_t>(bin);
            if (sym_count >= capture_len) {
                uint8_t dbg[capture_len];
                for (size_t i = 0; i < capture_len; i++)
                    dbg[i] = static_cast<uint8_t>(symbols[i] & 0xFF);
                emit(5, dbg, static_cast<uint8_t>(capture_len));
                reset();
            }
            break;

        case State::Header:
            symbols[sym_count++] = static_cast<uint16_t>(
                (((bin + fft_n - (preamble_bin % fft_n) + bin_rotate) % fft_n)
                 / oversample) % symbol_max);
            if (sym_count >= 8 + hdr_search) on_header_complete();
            break;

        case State::Payload:
            if (sym_count < max_symbols) {
                last_peak_bin = bin;
#if SOFT_DECODE
                compute_bit_rel(sym_count);
#endif
                symbols[sym_count++] = static_cast<uint16_t>(
                    (((bin + fft_n - (preamble_bin % fft_n) + bin_rotate) % fft_n)
                     / oversample) % symbol_max);
            }
            if (sym_count >= payload_symbols_needed) on_payload_complete();
            break;

        default:
            break;
    }
}

void MeshtasticProcessor::execute(const buffer_c8_t& buffer) {
    /* 8-bit IQ at 2 Msps, decimated by 4 to the 500 kHz chip rate with a
     * boxcar: four adds per output, adequate because the analogue front end
     * has already limited the band. */
    /* Passive: count every delivered sample and report periodically. A
     * shortfall against 2 Msps means buffers are being dropped upstream,
     * which is invisible from inside execute() any other way. */
    samples_total += static_cast<uint32_t>(buffer.count);
    if (buffer.count >= samples_to_report) {
        samples_to_report = 2000000;
        uint8_t r[4] = {static_cast<uint8_t>(samples_total),
                        static_cast<uint8_t>(samples_total >> 8),
                        static_cast<uint8_t>(samples_total >> 16),
                        static_cast<uint8_t>(samples_total >> 24)};
        emit(9, r, 4);
    } else {
        samples_to_report -= static_cast<uint32_t>(buffer.count);
    }

    for (size_t i = 0; i < buffer.count; i++) {
        decim_acc += std::complex<float>(
            static_cast<float>(buffer.p[i].real()),
            static_cast<float>(buffer.p[i].imag()));

        if (++decim_phase < decim) continue;
        decim_phase = 0;
        if (phase_skip > 0) {  /* shift the decimation grid by whole samples */
            phase_skip--;
            decim_acc = {0.0f, 0.0f};
            continue;
        }
        if (timing_hold > 0) {
            /* Repeat this sample -- write it an *extra* time and then fall
             * through so the normal path writes it again.
             *
             * The first version wrote once and `continue`d past the normal
             * write, so it inserted nothing: the loop could only ever drop
             * samples, never restore them. Any standing bias then walked the
             * sampling point steadily out of alignment across a packet, which
             * is exactly what the corrupted symbols showed -- no offset in the
             * search window verified at all, offline or on-device. An
             * asymmetric control loop is worse than no loop. */
            timing_hold--;
            if (window_fill < fft_n) {
                window[window_fill++] = decim_acc;
                if (window_fill >= fft_n) { window_fill = 0; process_symbol(); }
            }
        }

        if (resync_skip > 0) {
            /* Discard whole decimated samples until the window is aligned. */
            resync_skip--;
            decim_acc = {0.0f, 0.0f};
            window_fill = 0;
            continue;
        }

        window[window_fill] = decim_acc;
        decim_acc = {0.0f, 0.0f};

        if (++window_fill >= fft_n) {
            window_fill = 0;
            process_symbol();
        }
    }
}

/* Every baseband image is a standalone executable with its own entry point;
 * the build gives each one -DBASEBAND_<name> and links it separately. Without
 * this the image fails to link with "undefined reference to `main'" from
 * ChibiOS's ResetHandler, which reads as a toolchain problem and is not one.
 */
int main() {
    EventDispatcher event_dispatcher{std::make_unique<MeshtasticProcessor>()};
    event_dispatcher.run();
    return 0;
}
