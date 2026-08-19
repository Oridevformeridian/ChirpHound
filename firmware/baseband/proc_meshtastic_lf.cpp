/*
 * Meshtastic LongFast (SF11 / BW250) receiver, baseband half (M4). GPLv2+.
 */
#include "portapack_shared_memory.hpp"
#include "proc_meshtastic_lf.hpp"
#include "event_m4.hpp"

#include <cmath>

MeshtasticLFProcessor::MeshtasticLFProcessor() {
    for (size_t k = 0; k < fft_n; k++) {
        const float phase = 3.14159265358979f * static_cast<float>(k * k) /
                            static_cast<float>(fft_n);
        dc_re_[k] = static_cast<int8_t>(lrintf(127.0f * cosf(-phase)));
        dc_im_[k] = static_cast<int8_t>(lrintf(127.0f * sinf(-phase)));
    }
    for (size_t p = 0; p < nphase; p++) {
#ifndef LF_CAND
        for (size_t k = 0; k < fft_n; k++) downchirp_corr[p][k] = down(k);
#endif
        reset(p);
    }
    /* Object fully built -- now it is safe for the baseband thread to call
     * execute(). (No-op in hostsim, which drives execute() directly.) */
    baseband_thread.start();
    rssi_thread.start();
}

void MeshtasticLFProcessor::reset(size_t p) {
    window_fill[p] = 0;
    decim_acc[p] = {0.0f, 0.0f};
    decim_cnt[p] = 0;
    last_bin[p] = 0;
    preamble_run[p] = 0;
    preamble_bin[p] = 0;
    sym_count[p] = 0;
    state[p] = State::Search;
#ifndef LF_CAND
    for (size_t k = 0; k < fft_n; k++) pre_mag[p][k] = 0.0f;
#endif
}

void MeshtasticLFProcessor::reset_all() {
    for (size_t p = 0; p < nphase; p++) reset(p);
}

/* Peak bin plus the strongest competitor and a 0..255 confidence
 * (255 = clean, 0 = a coin-flip between the two bins). */
uint32_t MeshtasticLFProcessor::peak_bin(
    std::array<std::complex<float>, fft_n>& src, float* sharpness,
    int8_t* frac) {
    fft_swap(src, scratch);
    fft_c_preswapped(scratch, 0, fft_k);
    float best = 0.0f, total = 0.0f;
    uint32_t bb = 0;
    for (size_t k = 0; k < fft_n; k++) {
        const float m = scratch[k].real() * scratch[k].real() +
                        scratch[k].imag() * scratch[k].imag();
        total += m;
        if (m > best) { best = m; bb = static_cast<uint32_t>(k); }
    }
    const float mean = total / static_cast<float>(fft_n);
    *sharpness = (mean > 0.0f) ? (best / mean) : 0.0f;
    if (frac) {
        /* parabolic sub-bin offset of the peak, using linear magnitude */
        auto mg = [&](size_t k) {
            return sqrtf(scratch[k].real() * scratch[k].real() +
                         scratch[k].imag() * scratch[k].imag());
        };
        const float a0 = mg((bb + fft_n - 1) % fft_n);
        const float a1 = mg(bb);
        const float a2 = mg((bb + 1) % fft_n);
        const float den = a0 - 2.0f * a1 + a2;
        float d = (fabsf(den) > 1e-9f) ? (0.5f * (a0 - a2) / den) : 0.0f;
        if (d > 0.5f) d = 0.5f; if (d < -0.5f) d = -0.5f;
        *frac = static_cast<int8_t>(d * 120.0f);
    }
    return bb;
}

void MeshtasticLFProcessor::on_lock(size_t p) {
    /* Fractional CFO from the accumulated preamble spectrum: parabolic
     * interpolation of the peak. Correcting it is what makes the beacon decode
     * byte-perfect on the host. */
#ifndef LF_CAND
    const size_t b = preamble_bin[p];
    const float a0 = pre_mag[p][(b + fft_n - 1) % fft_n];
    const float a1 = pre_mag[p][b];
    const float a2 = pre_mag[p][(b + 1) % fft_n];
    const float den = a0 - 2.0f * a1 + a2;
    const float delta = (fabsf(den) > 1e-9f) ? (0.5f * (a0 - a2) / den) : 0.0f;
    for (size_t k = 0; k < fft_n; k++) {
        const float ph = -2.0f * 3.14159265358979f * delta *
                         static_cast<float>(k) / static_cast<float>(fft_n);
        downchirp_corr[p][k] =
            down(k) * std::complex<float>(cosf(ph), sinf(ph));
    }
#endif
#ifdef LF_BATCH
    if (p == 0) lock_raw = raw_head;
#endif
#ifdef LF_CAND
    if (p == 0) { sfd_scan = 0; sfd_best = 0.0f; sfd_bin = 0;
                  state[0] = State::FindSFD; sym_count[0] = 0; return; }
#endif
    state[p] = State::Capture;
    sym_count[p] = 0;
}

/* Decode header+payload at a timing, optionally overriding chosen symbol
 * positions to their runner-up bin (flip[pos]!=0). Fills out with
 * payload_len+2 bytes (payload then CRC) and returns payload_len if the header
 * checksum verifies; 0 otherwise. The caller checks the LoRa CRC-16. */
static size_t decode_full(const uint16_t* sym, size_t nsym, uint32_t pre,
                          size_t hstart, uint32_t rot, int hadj, int padj,
                          uint8_t* out, size_t out_cap, lora::Header* hdr_out) {
    const uint8_t SFv = 11;
    const uint8_t sf_app_h = SFv - 2;
    if (hstart + 8 > nsym) return 0;
    const uint32_t MASK = (1u << SFv) - 1;

    uint16_t gray[8];
    for (size_t i = 0; i < 8; i++) {
        const uint32_t s = (sym[hstart + i] + 2u * (1u << SFv) - pre + rot + hadj) & MASK;
        gray[i] = lora::gray_demap(static_cast<uint16_t>(s >> 2));
    }
    uint8_t cw[9] = {0};
    lora::deinterleave(gray, cw, sf_app_h, 8);
    uint8_t nib[9];
    for (uint8_t i = 0; i < sf_app_h; i++) nib[i] = lora::hamming_decode(cw[i], 4, nullptr);

    lora::Header hdr;
    if (!lora::parse_header(nib, &hdr)) return 0;
    if (!hdr.crc_present) return 0;
    const uint8_t plen = hdr.payload_len;
    if (plen == 0 || static_cast<size_t>(plen) + 2 > out_cap) return 0;
    if (hdr_out) *hdr_out = hdr;

    const uint8_t sf_app_p = SFv;
    const uint8_t cw_len_p = hdr.cr + 4;
    static uint8_t nibs[256];
    size_t nc = 0;
    for (uint8_t i = 5; i < sf_app_h; i++) nibs[nc++] = nib[i];
    size_t base = hstart + 8;
    while (base + cw_len_p <= nsym && nc + sf_app_p < sizeof(nibs) &&
           nc < static_cast<size_t>(plen + 2) * 2 + 8) {
        uint16_t g[8];
        for (size_t i = 0; i < cw_len_p; i++) {
            const uint32_t s = (sym[base + i] + 2u * (1u << SFv) - pre + rot + padj) & MASK;
            g[i] = lora::gray_demap(static_cast<uint16_t>(s));
        }
        uint8_t cwp[11] = {0};
        lora::deinterleave(g, cwp, sf_app_p, cw_len_p);
        for (uint8_t i = 0; i < sf_app_p; i++) nibs[nc++] = lora::hamming_decode(cwp[i], hdr.cr, nullptr);
        base += cw_len_p;
    }
    lora::dewhiten(nibs, nc, out, plen, hdr.crc_present);
    return plen;
}

bool MeshtasticLFProcessor::decode(size_t p) {
    static uint8_t out[64];
    static uint8_t best[64]; uint8_t best_len = 0;

    static uint16_t eff[max_symbols];
    static const int adj[3] = {-1, 0, 1};
    /* Sweep the residual fractional CFO. A symbol's true bin is peak+/-1 when
     * its parabolic sub-bin offset plus the trial CFO crosses a bin edge; the
     * LoRa CRC-16 confirms the sweep value that lands every symbol right. This
     * subsumes per-symbol timing error, so no separate flip pass is needed. */
    for (int di = -8; di <= 8; di++) {
        const float dlt = static_cast<float>(di) * 0.06f;
        for (size_t i = 0; i < sym_count[p]; i++) {
            float f = static_cast<float>(symfrac[p][i]) / 120.0f + dlt;
            int sh = (f > 0.5f) ? 1 : (f < -0.5f) ? -1 : 0;
            eff[i] = static_cast<uint16_t>((symbols[p][i] + sh + fft_n) % fft_n);
        }
        for (uint32_t rot = 0; rot <= fft_n / 4; rot += fft_n / 4) {
            for (size_t hstart = 3; hstart + 8 <= sym_count[p] && hstart < 26; hstart++) {
                for (int hi = 0; hi < 3; hi++) {
                    lora::Header hdr;
                    const size_t plen0 = decode_full(eff, sym_count[p],
                        preamble_bin[p], hstart, rot, adj[hi], -1, out,
                        sizeof(out), &hdr);
                    if (plen0 < 12) continue;
                    if (di == 0 && best_len == 0) {   /* keep a best-effort copy */
                        best_len = static_cast<uint8_t>(plen0);
                        for (size_t z = 0; z < plen0 + 2u; z++) best[z] = out[z];
                    }
                    for (int pi = 0; pi < 3; pi++) {
                        const size_t plen = decode_full(eff, sym_count[p],
                            preamble_bin[p], hstart, rot, adj[hi], adj[pi],
                            out, sizeof(out), nullptr);
                        if (plen >= 12 && lora::crc16_ok(out, plen)) {
                            emit(2, out, static_cast<uint8_t>(plen));
                            return true;
                        }
                    }
                }
            }
        }
    }
    /* No timing made the whole payload CRC-clean (one marginal symbol on this
     * front end is enough to fail it). Ship the near-complete decode as
     * unverified so the header/addresses still reach the log. */
    if (best_len >= 12) emit(6, best, best_len);
    return false;
}

void MeshtasticLFProcessor::process_symbol(size_t p) {
#ifdef LF_CAND
#else
#endif
#ifdef LF_CAND
    for (size_t k = 0; k < fft_n; k++) dechirped[k] = win(p, k) * down(k);
#else
    for (size_t k = 0; k < fft_n; k++)
        dechirped[k] = win(p, k) *
            ((state[p] == State::Search) ? down(k) : downchirp_corr[p][k]);
#endif
    float sharp = 0.0f;
    int8_t frac = 0;
    const uint32_t bin = peak_bin(dechirped, &sharp, &frac);

    if (state[p] == State::Search) {
        if (sharp <= 6.0f) { preamble_run[p] = 0; return; }
        const uint32_t d = (bin > last_bin[p]) ? bin - last_bin[p] : last_bin[p] - bin;
        if (preamble_run[p] > 0 && d <= bin_tolerance) {
            preamble_run[p]++;
        } else {
            preamble_run[p] = 1;
#ifndef LF_CAND
            for (size_t k = 0; k < fft_n; k++) pre_mag[p][k] = 0.0f;
#endif
        }
#ifndef LF_CAND
        for (size_t k = 0; k < fft_n; k++)
            pre_mag[p][k] += sqrtf(scratch[k].real() * scratch[k].real() +
                                   scratch[k].imag() * scratch[k].imag());
#endif
        last_bin[p] = bin;
        preamble_bin[p] = bin;
        if (preamble_run[p] >= preamble_min) {
            if (p == 0) {
                uint8_t dbg[2] = {static_cast<uint8_t>(bin & 0xFF),
                                  static_cast<uint8_t>(bin >> 8)};
                emit(4, dbg, 2);
            }
            on_lock(p);
        }
        return;
    }

#ifdef LF_CAND
    if (p == 0 && state[0] == State::FindSFD) {
        /* SFD (down-chirps) peak under the up-chirp reference = conj(downchirp).
         * pre_bin (down) and sfd_bin (up) give STO = (pre - sfd)/2, which is
         * CFO-invariant, so toff = const - STO tracks any transmitter's CFO. */
        for (size_t k = 0; k < fft_n; k++)
            dechirped[k] = win(0, k) * std::conj(down(k));
        fft_swap(dechirped, scratch); fft_c_preswapped(scratch, 0, fft_k);
        float ub = 0.0f, ut = 0.0f; uint32_t ubin = 0;
        for (size_t k = 0; k < fft_n; k++) {
            const float m = scratch[k].real() * scratch[k].real() + scratch[k].imag() * scratch[k].imag();
            ut += m; if (m > ub) { ub = m; ubin = static_cast<uint32_t>(k); }
        }
        const float us = (ut > 0.0f) ? ub / (ut / fft_n) : 0.0f;
        if (us > sfd_best) {
            sfd_best = us; sfd_bin = ubin;
        }
        if (++sfd_scan >= LF_CAND_SFD_SCAN) {
            const int N = static_cast<int>(fft_n);
            int pre = static_cast<int>(preamble_bin[0]);
            int sfd = static_cast<int>(sfd_bin); if (sfd > N/2) sfd -= N;
            int sto = ((pre - sfd) / 2) % N; if (sto < 0) sto += N;
            int cst = LF_CAND_CONST; const char* e = getenv("LF_CONST"); if (e) cst = atoi(e);
            int center = (cst - sto) % N; if (center < 0) center += N;
            const char* co = getenv("LF_CFOC"); float cfo_center = co ? atof(co) : -0.55f;
            cand_init(center, cfo_center);
            cand_have_prev = false;
            state[0] = State::Capture; sym_count[0] = 0;
        }
        return;
    }
    if (p == 0) {
        if (!cand_have_prev) {
            for (size_t k = 0; k < fft_n; k++) {
                ring_re[k] = win_re_[0][k];
                ring_im[k] = win_im_[0][k];
            }
            cand_have_prev = true;
            return;
        }
        const size_t slot = sym_count[0];
        for (size_t c = 0; c < ncand; c++) {
            const size_t tf = cand_toff[c];
            const std::complex<float> w = cand_w[c];
            std::complex<float> ramp(1.0f, 0.0f);   /* exp(-j2pi cf k/N), recurrence */
            for (size_t k = 0; k < fft_n; k++) {
                const std::complex<float> sv = (tf + k < fft_n)
                    ? std::complex<float>(ring_re[tf + k], ring_im[tf + k])
                    : win(0, tf + k - fft_n);
                dechirped[k] = sv * down(k) * ramp;
                ramp *= w;
            }
            fft_swap(dechirped, scratch);
            fft_c_preswapped(scratch, 0, fft_k);
            float best = 0.0f; uint32_t bb = 0;
            for (size_t k = 0; k < fft_n; k++) {
                const float m = scratch[k].real() * scratch[k].real() +
                                scratch[k].imag() * scratch[k].imag();
                if (m > best) { best = m; bb = static_cast<uint32_t>(k); }
            }
            if (slot < max_symbols) cand_bins[c][slot] = static_cast<uint16_t>(bb);
        }
        for (size_t k = 0; k < fft_n; k++) {
            ring_re[k] = win_re_[0][k];
            ring_im[k] = win_im_[0][k];
        }
        if (slot < max_symbols) sym_count[0]++;
        if (sym_count[0] >= cap_after_lock) {
            const bool hit = cand_decode();
            if (hit) reset_all(); else reset(0);
        }
        return;
    }
#endif
    if (sym_count[p] < max_symbols) {
        symbols[p][sym_count[p]] = static_cast<uint16_t>(bin);
        symfrac[p][sym_count[p]] = frac;
        sym_count[p]++;
    }
    if (sym_count[p] >= cap_after_lock) {
#ifdef LF_BATCH
        bool hit = false;
        if (p == 0) hit = batch_decode();
        if (hit) reset_all(); else reset(p);
#else
        const bool hit = decode(p);
        if (hit) reset_all(); else reset(p);
#endif
    }
}

void MeshtasticLFProcessor::feed(size_t p) {
    if (++decim_cnt[p] < decim) return;
    decim_cnt[p] = 0;
    win_re_[p][window_fill[p]] = static_cast<int16_t>(lrintf(decim_acc[p].real()));
    win_im_[p][window_fill[p]] = static_cast<int16_t>(lrintf(decim_acc[p].imag()));
    decim_acc[p] = {0.0f, 0.0f};
    if (++window_fill[p] >= fft_n) { window_fill[p] = 0; process_symbol(p); }
}

void MeshtasticLFProcessor::execute(const buffer_c8_t& buffer) {
    for (size_t i = 0; i < buffer.count; i++) {
        const std::complex<float> s(static_cast<float>(buffer.p[i].real()),
                                    static_cast<float>(buffer.p[i].imag()));
#ifdef LF_BATCH
        rawbuf[raw_head & (raw_cap - 1)] = s;
        raw_head++;
#endif
        for (size_t p = 0; p < nphase; p++) {
            if (sample_index < p) continue;
            decim_acc[p] += s;
            feed(p);
        }
        sample_index++;
    }
}

void MeshtasticLFProcessor::emit(uint8_t kind, const uint8_t* data, uint8_t len) {
    if (len > 240) len = 240;
    message.msg_len = len;
    message.state = kind;
    for (uint8_t i = 0; i < len; i++) message.message[i] = static_cast<char>(data[i]);
    shared_memory.application_queue.push(message);
}


#ifdef LF_BATCH
/* One dechirped-FFT peak of the symbol whose first raw sample is sym0. Decimate
 * `decim` raw samples per point into a 2048 window, de-rotate by the trial CFO,
 * dechirp, FFT, argmax. */
uint16_t MeshtasticLFProcessor::batch_bin(uint64_t sym0, float cfo) {
    for (size_t k = 0; k < fft_n; k++) {
        std::complex<float> acc{0.0f, 0.0f};
        const uint64_t g = sym0 + static_cast<uint64_t>(k) * decim;
        for (size_t t = 0; t < decim; t++)
            acc += rawbuf[(g + t) & (raw_cap - 1)];
        const float ph = -2.0f * 3.14159265358979f * cfo *
                         static_cast<float>(k) / static_cast<float>(fft_n);
        dechirped[k] = acc * std::complex<float>(cosf(ph), sinf(ph)) * down(k);
    }
    fft_swap(dechirped, scratch);
    fft_c_preswapped(scratch, 0, fft_k);
    float best = 0.0f; uint32_t bb = 0;
    for (size_t k = 0; k < fft_n; k++) {
        const float m = scratch[k].real() * scratch[k].real() +
                        scratch[k].imag() * scratch[k].imag();
        if (m > best) { best = m; bb = static_cast<uint32_t>(k); }
    }
    return static_cast<uint16_t>(bb);
}

/* Fractional CFO from the averaged preamble spectrum (parabola on linear |FFT|),
 * mirroring the host reference. seg0 = raw index of the first frame's sample. */
float MeshtasticLFProcessor::batch_cfo(uint64_t seg0, size_t nsym) {
    const size_t use = nsym < 12 ? nsym : 12;
    if (use < 4) return 0.0f;
    static float mag[fft_n];
    for (size_t k = 0; k < fft_n; k++) mag[k] = 0.0f;
    for (size_t j = 0; j < use; j++) {
        const uint64_t sym0 = seg0 + static_cast<uint64_t>(j) * fft_n * decim;
        for (size_t k = 0; k < fft_n; k++) {
            std::complex<float> acc{0.0f, 0.0f};
            const uint64_t g = sym0 + static_cast<uint64_t>(k) * decim;
            for (size_t t = 0; t < decim; t++) acc += rawbuf[(g + t) & (raw_cap - 1)];
            dechirped[k] = acc * down(k);
        }
        fft_swap(dechirped, scratch);
        fft_c_preswapped(scratch, 0, fft_k);
        for (size_t k = 0; k < fft_n; k++)
            mag[k] += sqrtf(scratch[k].real() * scratch[k].real() +
                            scratch[k].imag() * scratch[k].imag());
    }
    uint32_t b = 0; float bm = 0.0f;
    for (size_t k = 0; k < fft_n; k++) if (mag[k] > bm) { bm = mag[k]; b = static_cast<uint32_t>(k); }
    const float a0 = mag[(b + fft_n - 1) % fft_n], a1 = mag[b], a2 = mag[(b + 1) % fft_n];
    const float den = a0 - 2.0f * a1 + a2;
    float d = (fabsf(den) > 1e-9f) ? (0.5f * (a0 - a2) / den) : 0.0f;
    if (d > 0.5f) d = 0.5f; if (d < -0.5f) d = -0.5f;
    return d;
}

bool MeshtasticLFProcessor::batch_decode() {
    static uint8_t out[64];
    static uint8_t best[64]; uint8_t best_len = 0;
    static uint16_t bins[128];
    static const int adj[3] = {-1, 0, 1};

    /* Re-window the burst from ~16 symbols before lock (covers the whole
     * preamble for CFO + a generous header-offset search). */
    const uint64_t PRE = static_cast<uint64_t>(fft_n) * decim * 16;
    if (lock_raw < PRE + decim) return false;
    const uint64_t region0 = lock_raw - PRE;
    const size_t nsym = (fft_n * 18 < 96) ? 96 : 96;   /* cap the frames we scan */

    for (size_t shift = 0; shift < decim; shift++) {
        const uint64_t seg0 = region0 + shift;
        /* how many whole symbols fit before we reach raw_head */
        const size_t avail = static_cast<size_t>((raw_head - seg0) / (fft_n * decim));
        const size_t ns = avail < nsym ? avail : nsym;
        if (ns < 24) continue;
        const float cfo0 = batch_cfo(seg0, ns);
        /* Frame-boundary (symbol-timing) sweep: our capture start is offset
         * from the true symbol boundary by an unknown number of decimated
         * samples. Normalisation hides that as a bin shift for clean symbols
         * but leaves ISI that flips a marginal one; the right boundary lands
         * every symbol and passes CRC. */
        for (int toff = 0; toff < static_cast<int>(fft_n); toff += 16) {
        const uint64_t tseg0 = seg0 + static_cast<uint64_t>(toff) * decim;
        for (int ci = -4; ci <= 4; ci++) {
            const float cfo = cfo0 + static_cast<float>(ci) * 0.08f;
            for (size_t j = 0; j < ns && j < 100; j++)
                bins[j] = batch_bin(tseg0 + static_cast<uint64_t>(j) * fft_n * decim, cfo);
            /* preamble bin = mode of the first 12 */
            uint16_t pre = bins[6];
            {
                int bc = 0;
                for (size_t a = 0; a < 12 && a < ns; a++) {
                    int c = 0;
                    for (size_t b = 0; b < 12 && b < ns; b++)
                        if (bins[b] == bins[a]) c++;
                    if (c > bc) { bc = c; pre = bins[a]; }
                }
            }
            for (uint32_t rot = 0; rot <= fft_n / 4; rot += fft_n / 4) {
                for (size_t hstart = 3; hstart + 8 <= ns && hstart < 30; hstart++) {
                    for (int hi = 0; hi < 3; hi++) {
                        lora::Header hdr;
                        const size_t plen0 = decode_full(bins, ns, pre, hstart,
                            rot, adj[hi], -1, out, sizeof(out), &hdr);
                        if (plen0 < 12) continue;
                        if (best_len == 0) {
                            best_len = static_cast<uint8_t>(plen0);
                            for (size_t z = 0; z < plen0 + 2u; z++) best[z] = out[z];
                        }
                        for (int pi = 0; pi < 3; pi++) {
                            const size_t plen = decode_full(bins, ns, pre, hstart,
                                rot, adj[hi], adj[pi], out, sizeof(out), nullptr);
                            if (plen >= 12 && lora::crc16_ok(out, plen)) {
                                emit(2, out, static_cast<uint8_t>(plen));
                                return true;
                            }
                        }
                    }
                }
            }
        }
        }
    }
    if (best_len >= 12) emit(6, best, best_len);
    return false;
}
#endif


#ifdef LF_CAND
void MeshtasticLFProcessor::cand_init(int toff_center, float cfo_center) {
    static const int toff_off[LF_CAND_NTOFF] = LF_CAND_TOFF_OFFS;
    static const int cfo_off_i[LF_CAND_NCFO] = LF_CAND_CFO_OFFS;   /* x0.01 */
    int psgn = 1; const char* ps = getenv("LF_PRE_SGN"); if (ps) psgn = atoi(ps);
    cand_n = 0;
    for (size_t ti = 0; ti < LF_CAND_NTOFF; ti++) {
        int t = (toff_center + toff_off[ti]) % static_cast<int>(fft_n);
        if (t < 0) t += fft_n;
        int pc = (static_cast<int>(preamble_bin[0]) + psgn * t) % static_cast<int>(fft_n);
        if (pc < 0) pc += fft_n;
        for (size_t ci = 0; ci < LF_CAND_NCFO; ci++) {
            const float cf = cfo_center + 0.01f * static_cast<float>(cfo_off_i[ci]);
            cand_toff[cand_n] = static_cast<uint16_t>(t);
            cand_pre[cand_n] = static_cast<uint16_t>(pc);
            cand_cfo[cand_n] = cf;
            const float wph = -2.0f * 3.14159265358979f * cf / static_cast<float>(fft_n);
            cand_w[cand_n] = std::complex<float>(cosf(wph), sinf(wph));
            cand_n++;
        }
    }
}

bool MeshtasticLFProcessor::cand_decode() {
    static uint8_t out[64];
    static const int adj[3] = {-1, 0, 1};
    for (size_t c = 0; c < ncand; c++) {
        const uint16_t* b = cand_bins[c];
        const uint16_t pre = cand_pre[c];
        for (uint32_t rot = 0; rot <= fft_n / 4; rot += fft_n / 4) {
            for (size_t hstart = 0; hstart + 8 <= sym_count[0] && hstart < 30; hstart++) {
                for (int hi = 0; hi < 3; hi++) {
                    lora::Header hdr;
                    const size_t plen0 = decode_full(b, sym_count[0], pre, hstart,
                                                     rot, adj[hi], -1, out, sizeof(out), &hdr);
                    if (plen0 < 12) continue;
                    for (int pi = 0; pi < 3; pi++) {
                        const size_t plen = decode_full(b, sym_count[0], pre, hstart,
                                                        rot, adj[hi], adj[pi], out, sizeof(out), nullptr);
                        if (plen >= 12 && lora::crc16_ok(out, plen)) {
                            emit(2, out, static_cast<uint8_t>(plen));
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}
#endif

int main() {
    EventDispatcher event_dispatcher{std::make_unique<MeshtasticLFProcessor>()};
    event_dispatcher.run();
    return 0;
}
