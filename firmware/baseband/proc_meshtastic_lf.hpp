/*
 * Meshtastic LongFast (SF11 / BW250) receiver, baseband half (M4).
 * This file is part of PortaPack. GPLv2+.
 */
#ifndef __PROC_MESHTASTIC_LF_H__
#define __PROC_MESHTASTIC_LF_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"
#include "message.hpp"
#include "dsp_fft.hpp"
#include "lora_decode.hpp"

#include <array>
#include <complex>
#include <cstdint>

/* LongFast = SF11, BW250, CR4/5, captured at 1 Msps (OS=4 -> 250 kHz chip
 * rate), so a 2^11 = 2048-point FFT (dsp_fft's twiddle table was extended to
 * K=11 for exactly this). SF11 is SLOW -- 122 symbols/s vs SHORT_TURBO's 3906
 * -- so the 2048-pt FFT (~0.5 ms) fits the 8.19 ms symbol budget with room to
 * run several decimation phases in parallel.
 *
 * The marginal front end rounds a few boundary symbols the wrong way; which
 * ones depends on the sub-chip sampling phase. So we run LF_NPHASE decimation
 * phases independently and accept the first whose payload passes the LoRa
 * CRC-16 -- an on-device, keyless correctness gate. Host (hostsim) uses 4;
 * the device build can trim LF_NPHASE to fit SRAM. Validated end to end on the
 * host (longfast_decode.py): the beacon decodes byte-perfect and decrypts. */
#ifndef LF_NPHASE
#define LF_NPHASE 4
#endif

class MeshtasticLFProcessor : public BasebandProcessor {
   public:
    MeshtasticLFProcessor();
    void execute(const buffer_c8_t& buffer) override;

   private:
    static constexpr size_t baseband_fs = 1'000'000;   /* 1 Msps for BW250 */
    static constexpr size_t decim = 4;
    static constexpr size_t sf = 11;
    static constexpr size_t fft_n = 1 << sf;            /* 2048 */
    static constexpr size_t fft_k = sf;                 /* 11 <= K_max */
    static constexpr size_t symbol_max = 1 << sf;
    static constexpr size_t nphase = LF_NPHASE;

    static constexpr size_t preamble_min = 14;
    static constexpr uint32_t bin_tolerance = 2;
    static constexpr size_t max_symbols = 96;           /* header+payload */
    static constexpr size_t cap_after_lock = 60;        /* symbols to buffer */

    void feed(size_t p);
    void process_symbol(size_t p);
    void reset(size_t p);
    void reset_all();
    uint32_t peak_bin(std::array<std::complex<float>, fft_n>& src,
                      float* sharpness, int8_t* frac);
    void on_lock(size_t p);
    bool decode(size_t p);   /* CRC-guided decode; emit on CRC pass */
    void emit(uint8_t kind, const uint8_t* data, uint8_t len);

    /* phase-independent reference downchirp */
    std::array<std::complex<float>, fft_n> downchirp{};
    /* scratch reused across phases (single-threaded execute) */
    std::array<std::complex<float>, fft_n> dechirped{};
    std::array<std::complex<float>, fft_n> scratch{};

    /* per decimation phase */
    std::array<std::complex<float>, fft_n> window[nphase]{};
    std::array<std::complex<float>, fft_n> downchirp_corr[nphase]{};
    std::array<float, fft_n> pre_mag[nphase]{};
    uint16_t symbols[nphase][max_symbols]{};   /* peak bin */
    int8_t   symfrac[nphase][max_symbols]{};   /* parabolic sub-bin offset */

    size_t window_fill[nphase]{};
    std::complex<float> decim_acc[nphase]{};
    size_t decim_cnt[nphase]{};

    enum class State { Search, Capture } state[nphase]{};
    uint32_t last_bin[nphase]{};
    size_t preamble_run[nphase]{};
    uint32_t preamble_bin[nphase]{};
    size_t sym_count[nphase]{};

    /* A few boundary symbols round to the wrong bin on this front end; each
     * is low-confidence (peak ~ runner-up). decode() flips the least-confident
     * payload symbols to their runner-up and lets the LoRa CRC-16 confirm the
     * fix -- keyless on-device error correction. */

    uint64_t sample_index{0};

    ACARSPacketMessage message{};

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif
