/* Run the PortaPack's real LoRa DSP on the host, against captured IQ.
 *
 * Why this exists: every timing experiment so far has cost a flash, a card
 * swap and a walk upstairs, and the offline model that guided them was a
 * numpy *reimplementation* of proc_meshtastic.cpp. A reimplementation can
 * validate a fix the firmware does not reproduce -- it drifts the moment
 * either side is edited. This compiles the actual processor, the actual
 * dsp_fft and the actual lora_decode, so the thing under test is the thing
 * that ships.
 *
 * The output is deliberately byte-identical to the lines meshtastic_app.cpp
 * writes to /LOGS/MESHTAST.TXT, so score_log.py scores a host run and a
 * device run with the same code and the numbers are directly comparable.
 *
 *   ./hostsim ~/rf/mesh_iq2.bin | tee run.txt
 *   python3 ../score_log.py run.txt --against ../baseline_720549d3.txt
 *
 * The IQ format is what the radio delivers: interleaved signed 8-bit I/Q at
 * 2 Msps, which is exactly what buffer_c8_t carries.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dsp_types.hpp"
#include <memory>
#include "proc_meshtastic_lf.hpp"
#include "portapack_shared_memory.hpp"

SharedMemory shared_memory;

/* The device stamps each line with a clock it does not really have; the
 * scorer ignores the stamp and keys on the tag, but keeping the column means
 * host and device logs can be diffed directly. */
static std::string stamp(size_t sample, size_t rate) {
    const size_t ms = (sample * 1000) / rate;
    char buf[32];
    std::snprintf(buf, sizeof buf, "%014zu", ms);
    return buf;
}

static std::string hex1(uint8_t v) {
    static const char* d = "0123456789ABCDEF";
    return std::string(1, d[v & 0xF]);
}

static std::string hex2(uint8_t v) {
    static const char* d = "0123456789ABCDEF";
    return std::string(1, d[(v >> 4) & 0xF]) + std::string(1, d[v & 0xF]);
}

/* Mirror of meshtastic_app.cpp's on_packet switch. Kept in the same order and
 * with the same guards so a change there is easy to mirror here; if the two
 * drift, the scores stop being comparable and that is worth noticing. */
static void render(const ACARSPacketMessage& p, const std::string& ts) {
    switch (p.state) {
        case 1:  /* preamble detection: UI only on the device, no log line */
            break;

        case 2: {  /* decoded frame */
            std::string hex;
            for (uint8_t i = 0; i < p.msg_len; i++)
                hex += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s F %s\n", ts.c_str(), hex.c_str());
            break;
        }

        case 3: {  /* header decode failed, or SFD timeout */
            if (p.msg_len == 2) {
                std::printf("%s T\n", ts.c_str());
                return;
            }
            if (p.msg_len < 6) return;
            std::string n;
            for (uint8_t i = 1; i < 6; i++)
                n += hex1(static_cast<uint8_t>(p.message[i]));
            std::string syms;
            for (uint8_t i = 6; i < p.msg_len && i < 14; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s E %s S %s\n", ts.c_str(), n.c_str(), syms.c_str());
            break;
        }

        case 4: {  /* preamble locked */
            std::string n;
            /* All bytes now: the fifth carries preamble sharpness, so a
             * lock that never became a frame still reports its strength. */
            for (uint8_t i = 0; i < p.msg_len; i++)
                n += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s P %s\n", ts.c_str(), n.c_str());
            break;
        }

        case 5: {  /* raw symbol capture */
            std::string syms;
            for (uint8_t i = 0; i < p.msg_len; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s C %s\n", ts.c_str(), syms.c_str());
            break;
        }

        case 11: {  /* CFO estimate: byte, 128 = zero, +/-0.5 bin full-scale */
            std::printf("%s Q %u\n", ts.c_str(), (unsigned)(uint8_t)p.message[0]);
            break;
        }

        case 10: {  /* preamble bins (raw), for sample-clock drift */
            std::string syms;
            for (uint8_t i = 0; i < p.msg_len; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s B %s\n", ts.c_str(), syms.c_str());
            break;
        }

        case 9: {  /* passive: cumulative samples seen by execute() */
            uint32_t n = 0;
            for (int i = 3; i >= 0; i--)
                n = (n << 8) | static_cast<uint8_t>(p.message[i]);
            std::printf("%s R %u\n", ts.c_str(), n);
            break;
        }

        case 8: {  /* diagnostic: relative symbols seen while hunting sync */
            std::string syms;
            for (uint8_t i = 0; i < p.msg_len; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s S %s\n", ts.c_str(), syms.c_str());
            break;
        }

        case 7: {  /* diagnostic: fractional timing error for this frame */
            std::string syms;
            for (uint8_t i = 0; i < p.msg_len; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s D %s\n", ts.c_str(), syms.c_str());
            break;
        }

        case 6: {  /* header search failed */
            std::string syms;
            /* From i=1: byte 0 is the 0xE3 marker and meshtastic_app.cpp skips
             * it, so rendering it here made host and device H lines differ by
             * a byte -- which briefly looked like the device running the wrong
             * build. */
            for (uint8_t i = 1; i < p.msg_len; i++)
                syms += hex2(static_cast<uint8_t>(p.message[i]));
            std::printf("%s H %s\n", ts.c_str(), syms.c_str());
            break;
        }

        default:
            break;
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: %s <iq.bin> [max_seconds]\n"
                     "  iq.bin: interleaved int8 I/Q at 2 Msps\n"
                     "  max_seconds: 0 or omitted = whole file\n"
                     "  gain: integer scale, saturating (default 1)\n",
                     argv[0]);
        return 2;
    }
    const size_t rate = 1000000;
    double max_s = (argc > 2) ? std::atof(argv[2]) : 0.0;
    /* Optional gain. Captures made by other radios do not necessarily fill the
     * int8 range the PortaPack's ADC would: mesh_iq2.bin peaks around +/-3 of
     * a possible +/-127, so the dechirp runs on ~2 bits and quantisation, not
     * the algorithm, may be what fails. A scale factor makes that testable
     * instead of assumed. Saturating, because wrapping would manufacture
     * signal that was never there. */
    const int gain = (argc > 3) ? std::atoi(argv[3]) : 1;

    std::FILE* fh = std::fopen(argv[1], "rb");
    if (!fh) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }

    auto proc_ptr = std::make_unique<MeshtasticLFProcessor>();
    MeshtasticLFProcessor& proc = *proc_ptr;

    /* Buffer size is not the device's exact DMA size, and it does not need to
     * be: execute() is a pure function of the sample sequence, carrying its
     * decimation and window state across calls. Chunking only affects how
     * often the queue is drained. */
    const size_t CHUNK = 4096;
    std::vector<int8_t> raw(CHUNK * 2);
    std::vector<complex8_t> samples(CHUNK);

    size_t consumed = 0;
    const size_t limit = max_s > 0 ? static_cast<size_t>(max_s * rate) : SIZE_MAX;

    while (consumed < limit) {
        const size_t want = std::min(CHUNK, limit - consumed);
        const size_t got = std::fread(raw.data(), 2, want, fh);
        if (got == 0) break;

        for (size_t i = 0; i < got; i++) {
            int re = raw[i * 2] * gain;
            int im = raw[i * 2 + 1] * gain;
            if (re > 127) re = 127; if (re < -128) re = -128;
            if (im > 127) im = 127; if (im < -128) im = -128;
            samples[i] = complex8_t{static_cast<int8_t>(re),
                                    static_cast<int8_t>(im)};
        }

        buffer_c8_t buf{samples.data(), got, rate};
        proc.execute(buf);

        for (const auto& m : shared_memory.application_queue.items)
            render(m, stamp(consumed, rate));
        shared_memory.application_queue.items.clear();

        consumed += got;
    }

    std::fclose(fh);
    std::fprintf(stderr, "processed %.2f s (%zu samples)\n",
                 static_cast<double>(consumed) / rate, consumed);
    return 0;
}
