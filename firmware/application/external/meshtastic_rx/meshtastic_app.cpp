/*
 * Meshtastic / LoRa receiver, application half (M0).
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "meshtastic_app.hpp"
#include "baseband_api.hpp"
#include "file_path.hpp"
#include "portapack.hpp"
#include "string_format.hpp"

using namespace portapack;

namespace ui::external_app::meshtastic_rx {

MeshtasticRxView::MeshtasticRxView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_frequency,
                  &text_pre, &text_hdr, &text_dec, &text_addr,
                  &text_last, &text_status});

    receiver_model.enable();

    logger = std::make_unique<MeshtasticLogger>();
    if (logger)
        logger->append(logs_dir / u"MESHTAST.TXT");

    text_status.set("SF7 BW500 -> /LOGS/MESHTAST.TXT");
    refresh_stats();
}

MeshtasticRxView::~MeshtasticRxView() {
    receiver_model.disable();
    baseband::shutdown();
}

void MeshtasticRxView::focus() {
    field_frequency.focus();
}

/* Meshtastic over-the-air header, first 16 bytes, all plaintext:
 *   dest u32 LE | sender u32 LE | id u32 LE | flags | chan_hash | next | relay
 * Only what follows is AES-encrypted, so from/to are readable with no key.
 */
static uint32_t le32(const char* p) {
    return static_cast<uint32_t>(static_cast<uint8_t>(p[0])) |
           (static_cast<uint32_t>(static_cast<uint8_t>(p[1])) << 8) |
           (static_cast<uint32_t>(static_cast<uint8_t>(p[2])) << 16) |
           (static_cast<uint32_t>(static_cast<uint8_t>(p[3])) << 24);
}

void MeshtasticRxView::refresh_stats() {
    text_pre.set("PREAMBLE " + to_string_dec_uint(n_pre, 6));
    text_hdr.set("HEADER   " + to_string_dec_uint(n_hdr, 6));
    text_dec.set("DECODE   " + to_string_dec_uint(n_dec, 6));
    text_addr.set("ADDRS    " + to_string_dec_uint(n_addr, 6));
}

void MeshtasticRxView::note_addr(uint32_t a) {
    for (uint8_t i = 0; i < n_addr; i++)
        if (addrs[i] == a) return;
    if (n_addr < 64) addrs[n_addr++] = a;
}

void MeshtasticRxView::on_packet(const ACARSPacketMessage* packet) {
    switch (packet->state) {
        case 1: {  /* preamble detection */
            if (packet->msg_len < 4) return;
            const uint16_t count =
                static_cast<uint8_t>(packet->message[2]) |
                (static_cast<uint16_t>(static_cast<uint8_t>(packet->message[3])) << 8);
            detections = count;
            break;
        }

        case 2: {  /* decoded frame */
            if (packet->msg_len < 16) {
                return;
            }
            const uint32_t dest = le32(&packet->message[0]);
            const uint32_t sender = le32(&packet->message[4]);
            const uint8_t chan_hash = static_cast<uint8_t>(packet->message[13]);

            std::string to = (dest == 0xFFFFFFFF)
                                 ? std::string("bcast")
                                 : ("!" + to_string_hex(dest, 8));
            n_hdr++;
            n_dec++;
            note_addr(sender);
            text_last.set("LAST !" + to_string_hex(sender, 8) +
                          " >" + to);
            refresh_stats();
            (void)chan_hash;

            /* The whole frame as hex. Everything needed to re-derive the
             * decode offline lives in this one line -- addresses, channel
             * hash and the ciphertext -- so a replay can be verified against
             * the host implementation without the device present. */
            if (logging && logger) {
                std::string hex;
                for (uint8_t i = 0; i < packet->msg_len; i++)
                    hex += to_string_hex(
                        static_cast<uint8_t>(packet->message[i]), 2);
                logger->log_str("F " + hex);
                /* Tag each frame with the live RX gain config so a settings
                 * sweep can be scored per-config from the log alone. */
                logger->log_str(
                    "G " + to_string_dec_uint(receiver_model.rf_amp() ? 1 : 0) +
                    " " + to_string_dec_uint(receiver_model.lna()) +
                    " " + to_string_dec_uint(receiver_model.vga()));
            }
            break;
        }

        case 11: {  /* CFO estimate (byte; 128 = zero) */
            if (logging && logger)
                logger->log_str("Q " + to_string_dec_uint(
                    static_cast<uint8_t>(packet->message[0])));
            break;
        }

        case 10: {  /* preamble bins (raw) -- offline SFO analysis */
            std::string syms;
            for (uint8_t i = 0; i < packet->msg_len; i++)
                syms += to_string_hex(static_cast<uint8_t>(packet->message[i]), 2);
            if (logging && logger) logger->log_str("B " + syms);
            break;
        }

        case 9: {  /* cumulative samples seen by the baseband */
            uint32_t n = 0;
            for (int i = 3; i >= 0; i--)
                n = (n << 8) | static_cast<uint8_t>(packet->message[i]);
            /* Logged, not shown: the value only means anything as a delta
             * against the previous line's timestamp. */
            if (logging && logger) logger->log_str("R " + to_string_dec_uint(n));
            break;
        }

        case 6: {  /* header search failed -- the 16-symbol window */
            std::string syms;
            for (uint8_t i = 1; i < packet->msg_len; i++)
                syms += to_string_hex(
                    static_cast<uint8_t>(packet->message[i]), 2);
            n_hdr++;
            refresh_stats();
            if (logging && logger) logger->log_str("H " + syms);
            break;
        }

        case 5: {  /* diagnostic raw symbol capture */
            std::string syms;
            for (uint8_t i = 0; i < packet->msg_len; i++)
                syms += to_string_hex(
                    static_cast<uint8_t>(packet->message[i]), 2);
            if (logging && logger) logger->log_str("C " + syms);
            break;
        }

        case 4: {  /* preamble locked -- logged so an empty log is unambiguous */
            if (packet->msg_len < 4) return;
            n_pre++;
            refresh_stats();
            if (logging && logger) {
                std::string n;
                for (uint8_t i = 0; i < 2; i++)
                    n += to_string_hex(
                        static_cast<uint8_t>(packet->message[i]), 2);
                logger->log_str("P " + n);
            }
            break;
        }

        case 3: {  /* header decode failed, or SFD timeout */
            if (packet->msg_len == 2) {
                /* 0xE2: preamble locked but the SFD never arrived. Distinct
                 * from a header failure, and the distinction is the whole
                 * point -- one means alignment, the other means decode. */
                if (logging && logger) logger->log_str("T");
                return;
            }
            if (packet->msg_len < 6) return;
            std::string n;
            for (uint8_t i = 1; i < 6; i++)
                n += to_string_hex(static_cast<uint8_t>(packet->message[i]), 1);
            std::string syms;
            for (uint8_t i = 6; i < packet->msg_len && i < 14; i++)
                syms += to_string_hex(
                    static_cast<uint8_t>(packet->message[i]), 2);
            n_hdr++;
            refresh_stats();
            /* Nibbles *and* the symbols they came from: without the symbols an
             * all-zero result is ambiguous between "no signal" and "wrong
             * alignment", which need opposite fixes. */
            if (logging && logger) logger->log_str("E " + n + " S " + syms);
            break;
        }

        default:
            break;
    }
}

}  // namespace ui::external_app::meshtastic_rx
