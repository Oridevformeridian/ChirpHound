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

#ifndef __PROC_MESHTASTIC_H__
#define __PROC_MESHTASTIC_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"
#include "message.hpp"
#include "dsp_fft.hpp"
#include "lora_decode.hpp"

#include <array>
#include <complex>
#include <cstdint>

/* LoRa chirp-spread-spectrum receiver, SF7 / BW500 (SHORT_TURBO).
 *
 * SF7 is not a placeholder choice: the firmware's dsp_fft carries twiddle
 * factors only to K=8 (256 points), so SF7's 128-point FFT is the largest that
 * needs no new DSP, and BW500 at 2 Msps is an exact 4x decimation. It is also
 * the preset the host pipeline already decodes, so a packet that fails here can
 * be compared against a known-good decode of the same air rather than guessed at.
 *
 * Frame structure being tracked:
 *
 *   8x upchirp (preamble)  ->  2x sync word  ->  2.25x downchirp (SFD)
 *   ->  8 header symbols (reduced rate, CR 4/8)  ->  payload symbols
 *
 * The SFD is what gives symbol alignment. It is found by dechirping with the
 * *up*chirp instead of the downchirp: a downchirp then collapses to a single
 * bin exactly as an upchirp does under the normal reference.
 */
class MeshtasticProcessor : public BasebandProcessor {
   public:
    MeshtasticProcessor();

    void execute(const buffer_c8_t& buffer) override;

   private:
    static constexpr size_t baseband_fs = 2'000'000;
    /* 2x oversampling: decimate to 1 Msps, two samples per chip.
     *
     * At the previous decim=4 the chain ran at exactly one sample per chip --
     * critically sampled, with no timing margin at all. That is why the SFD's
     * quarter symbol could only be approximated as a bin rotation rather than
     * corrected, and why residual frequency offset had nowhere to go.
     * gr-lora_sdr runs at 4x for exactly this reason.
     *
     * 2x is the most this hardware allows: symbols become 2^sf * 2 = 256
     * samples, needing a 256-point FFT, and the firmware's dsp_fft has
     * twiddle factors only to K=8. 4x would need 512-point tables that do not
     * exist here.
     *
     * Measured first: software tolerance (bin +-1, symbol offset +-1) lifted
     * yield only 8/14 -> 9/14 on real captures at 9x the trials, so retries
     * were not the answer. Timing resolution is.
     */
    static constexpr size_t decim = 4;
    static constexpr size_t sf = 7;
    static constexpr size_t oversample = 1;
    static constexpr size_t fft_n = (1 << sf) * oversample;  // 256
    static constexpr size_t fft_k = sf;                      // 7
    static constexpr size_t symbol_max = 1 << sf;            // 128 values

    static constexpr size_t preamble_min = 6;

    /* All four of these were measured from raw symbol captures off the air,
     * not derived from the spec, and each is checkable against those logs:
     *
     *  - Symbols are relative to the preamble bin. Subtracting it turns the
     *    two sync symbols into exactly 0x10 and 0x58 in every capture, which
     *    is LoRa's encoding of sync word 0x2B. Without it every symbol carries
     *    a constant bias and nothing downstream can decode.
     *  - The header sits 2 symbols past the sync word, and the SFD's trailing
     *    quarter symbol rotates every later bin by +N/4. 8 of 14 captures then
     *    decode to a valid CR-4/5 header with a passing checksum, six of them
     *    exactly 34 bytes -- the size of the test beacon.
     */
    /* Sync symbols in *bin* units, hence x oversample. A tolerance is
     * allowed because at 2x a half-chip error moves the peak one bin. */
    static constexpr uint32_t sync_sym_1 = 0x10 * oversample;
    static constexpr uint32_t sync_sym_2 = 0x58 * oversample;
    static constexpr uint32_t sync_tol = 3;  /* was 2: the phase_skip grid shift puts the sync word 2-3 bins off, so 2 missed it by one */
    /* How far a symbol may sit from 0 and still count as preamble.
     * Treating any non-zero value as 'not preamble' burned the reset
     * counter before the sync word arrived. */
    static constexpr uint32_t preamble_tol = 2;

    /* Decimation phase search: quarter-chip alignment for free.
     *
     * The radio delivers 2 Msps and the boxcar decimator collapses every 4
     * samples into one, discarding which of the 4 sample phases we landed on.
     * That is a quarter chip of timing information already paid for and then
     * thrown away -- and a quarter chip is exactly the SFD's trailing offset.
     *
     * Unlike true 2x oversampling (tried and reverted), this does not change
     * the coherent integration window: the FFT still sees 128 samples spanning
     * one whole symbol, so lock stability is unaffected. It only changes which
     * samples are summed. Cost is a few extra dechirps once per packet, during
     * the preamble, not per symbol. */
    static constexpr size_t phase_count = decim;
    /* Three, not two. The SFD is 2.25 symbols, which spans *three* whole
     * symbol windows -- skipping two started the header one symbol early.
     *
     * Determined, not guessed: with the reference receiver's decode of the
     * same beacon giving the exact bytes (ffffffff 2cab5843 ...), a search
     * over header offset, rotation and bit order reproduced all eight bytes
     * only at offset 15 from preamble lock, i.e. sync + 3. Rotation (+N/4)
     * and msb-first were already correct; this was the whole remaining bug. */
    static constexpr size_t sfd_symbols = 2;   /* nominal; the search covers slip */

    /* Search the header offset on-device instead of counting symbols to it.
     *
     * Counting failed repeatedly: the preamble run varies packet to packet
     * (2 to 10 symbols observed), so a fixed skip after the sync word lands
     * somewhere different each time. An offline solve found the header at a
     * specific offset for one capture and the same arithmetic missed on the
     * next.
     *
     * The header carries a 5-bit checksum, so trying a handful of offsets and
     * accepting the one that verifies is self-correcting and costs only a few
     * decodes per packet -- and a false accept needs both a checksum hit
     * (1/32) and a plausible length and coding rate. */
    /* Narrowed from 8. Each extra offset tried is another chance for the
     * 5-bit header checksum to pass by luck (~1/32), and the wide search
     * measurably traded quality for quantity: 14 frames at 11% header-byte
     * accuracy against 8 frames at 25% for a fixed offset. Trials without
     * tighter acceptance is a bad bargain. */
    /* Header sits at sync + 4 -- i.e. two symbols after the second sync
     * symbol, which is sfd_symbols=2. Measured directly: in every capture
     * containing a genuine 34-byte header, the offset was sync+4 exactly
     * (sync at 10 -> header 14, sync at 11 -> header 15).
     *
     * The search stays narrow because each extra offset is another ~1/32
     * chance of the checksum passing by luck. Searching 25 offsets offline
     * produced three false "headers" of 240, 198 and 174 bytes alongside the
     * four real ones -- a wide search manufactures frames rather than
     * finding them. */
    static constexpr size_t hdr_search = 3;

    /* Continuous timing recovery.
     *
     * Alignment was previously done once, at the preamble, and then the
     * stream was chopped into rigid windows for the whole packet -- roughly
     * 60 symbols. Two errors accumulate across that run:
     *
     *   - the preamble correction is whole-sample only, leaving up to half a
     *     chip of standing offset;
     *   - the radio and the transmitter run on independent crystals. At a
     *     typical +-10-20 ppm, a 44 ms SHORT_TURBO frame drifts ~0.9 samples,
     *     about half a chip, *during* the packet.
     *
     * That matches the symptom exactly: the 8 header symbols decode far
     * better than the ~50 payload symbols, and searching harder for the
     * header start never helped the payload -- the payload's problem is that
     * it walks out of alignment as it goes.
     *
     * A dechirped tone centred on a bin has symmetric neighbours, so the
     * imbalance between them is the signed fractional timing error. Accumulate
     * it and nudge the sample pointer when it crosses half a sample. Three
     * magnitudes and a divide per symbol; no extra FFT, and the coherent
     * window is untouched, so this cannot cost detection the way 2x did. */
    static constexpr float timing_gain = 0.35f;
    static constexpr uint32_t bin_rotate = fft_n / 4;  /* quarter symbol */
    static constexpr uint32_t bin_tolerance = 1;
    static constexpr size_t max_symbols = 200;
    static constexpr size_t max_payload = 255;
    /* Diagnostic capture length. Enough to span the rest of the preamble, the
     * sync word, the 2.25-symbol SFD and the whole 8-symbol header, so the
     * true alignment can be found offline instead of guessed at on hardware. */
    /* Long enough for a whole 34-byte frame: preamble tail + sync + SFD +
     * 8 header symbols + ~48 payload symbols at CR 4/5. With the true frame
     * bytes known from the reference receiver, a full symbol run turns the
     * remaining unknowns into a deterministic search instead of one
     * hypothesis per card swap. */
    static constexpr size_t capture_len = 80;

    enum class State : uint8_t {
        Search,   /* hunting for a run of identical upchirp symbols */
        Sync,     /* preamble locked; hunting the two sync-word symbols */
        Sfd,      /* sync seen; skipping the SFD symbols */
        Header,   /* collecting the 8 header symbols */
        Payload,  /* collecting payload symbols */
        Capture,  /* diagnostic: record the raw symbol stream */
    };

    void process_symbol();
    void compute_bit_rel(size_t slot);
    bool try_header_at(size_t off);
    void on_header_complete();
    void on_payload_complete();
    void emit(uint8_t kind, const uint8_t* data, uint8_t len);
    void reset();

    uint32_t peak_bin(const std::array<std::complex<float>, fft_n>& src,
                      float* sharpness, float* timing = nullptr);

    std::array<std::complex<float>, fft_n> downchirp{};
    /* Copy of downchirp with a per-packet phase ramp folded in to cancel the
     * fractional CFO/timing offset measured from the preamble. */
    std::array<std::complex<float>, fft_n> downchirp_corr{};
    float cfo_sum{0.0f};
    uint16_t cfo_n{0};
    float cfo_est{0.0f};
    std::array<std::complex<float>, fft_n> upchirp{};
    std::array<std::complex<float>, fft_n> window{};
    std::array<std::complex<float>, fft_n> scratch{};
    float mag_buf[fft_n]{};      /* |FFT|^2 per bin, for soft LLRs */
    float peak_mag{0.0f};
    uint32_t last_peak_bin{0};
    int8_t bit_rel[max_symbols][8]{};  /* per-bit reliability, 0..127 */
    /* Second buffer so peak_bin can fft_swap into scratch without its input
     * aliasing the destination. Replaces a 1 KB stack copy per symbol. */
    std::array<std::complex<float>, fft_n> dechirped{};

    /* Passive throughput counter: cumulative samples seen by execute(), and
     * how many are left before the next report. The log's RTC timestamps
     * supply the time base, so no clock is needed here. */
    uint32_t samples_total{0};
    uint32_t samples_to_report{2000000};

    /* Raw bin of each preamble symbol in the current candidate run, emitted at
     * lock. A flat sequence means no sample-clock drift; a ramp is SFO. */
    static constexpr size_t pre_bins_max = 12;
    uint8_t pre_bins[pre_bins_max]{};
    uint8_t pre_bins_n{0};

    std::array<uint16_t, max_symbols> symbols{};
    std::array<uint8_t, max_payload + 4> nibbles{};
    std::array<uint8_t, max_payload + 4> bytes{};

    size_t window_fill{0};
    size_t decim_phase{0};
    size_t phase_offset{0};   /* chosen decimation phase */
    size_t phase_probe{0};    /* phase under evaluation during preamble */
    float phase_best{0.0f};
    size_t phase_best_idx{0};
    size_t phase_tested{0};
    size_t phase_skip{0};
    std::complex<float> decim_acc{};

    State state{State::Search};
    uint32_t last_bin{0};
    uint32_t preamble_bin{0};
    size_t sync_stage{0};
    size_t preamble_run{0};
    size_t sym_count{0};
    size_t sfd_wait{0};
    /* Samples to drop to bring the window onto a symbol boundary. */
    size_t resync_skip{0};
    /* Whole symbols still to discard before the header begins. */
    size_t sfd_tail{0};
    uint32_t detections{0};
    uint32_t frames{0};

    lora::Header header{};
    size_t payload_symbols_needed{0};
    size_t hdr_offset{0};
    float timing_err{0.0f};   /* accumulated fractional sample error */
    size_t timing_hold{0};    /* samples to repeat, for negative drift */

    ACARSPacketMessage message{};

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_MESHTASTIC_H__*/
