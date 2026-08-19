/*
 * Meshtastic / LoRa receiver, application half (M0).
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MESHTASTIC_APP_H__
#define __MESHTASTIC_APP_H__

#include "app_settings.hpp"
#include "radio_state.hpp"
#include "ui_widget.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "ui_rssi.hpp"
#include "log_file.hpp"

namespace ui::external_app::meshtastic_lf_rx {

/* Meshtastic's channel grid: freq = start + bw/2 + n*bw.
 *
 * Verified against a real deployment: BW500 slot 30 gives 917.250 MHz, which
 * is where that site's traffic actually is. Exposed as a slot number rather
 * than raw Hz because "channel 30" is what a Meshtastic config shows, and
 * making the operator convert by hand invites off-by-one errors.
 */
constexpr uint32_t band_start_hz = 902'000'000;
constexpr uint32_t bandwidth_hz = 250'000;  // LongFast (BW250)
constexpr uint8_t default_slot = 31;

constexpr uint32_t slot_frequency(uint8_t slot) {
    return band_start_hz + (bandwidth_hz / 2) + (slot * bandwidth_hz);
}

/* Frames are logged as hex, one per line, so a capture can be replayed
 * offline and checked against the host decoder rather than taken on trust.
 * Hex rather than binary because the log has to survive being opened, copied
 * off a FAT card and pasted around; the size cost is irrelevant next to a
 * 32 GB card.
 */
class MeshtasticLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }
    void log_str(const std::string& msg) { log_file.write_entry(msg); }

   private:
    LogFile log_file{};
};

class MeshtasticLfRxView : public View {
   public:
    MeshtasticLfRxView(NavigationView& nav);
    ~MeshtasticLfRxView();

    void focus() override;

    std::string title() const override { return "MeshtasticLF"; };

   private:
    NavigationView& nav_;

    /* 1 Msps: 4x oversampling of the 250 kHz LongFast chip rate. */
    RxRadioState radio_state_{
        slot_frequency(default_slot),
        750'000 /* bandwidth */,
        1'000'000 /* sampling rate */
    };
    app_settings::SettingsManager settings_{
        "rx_meshtastic_lf", app_settings::Mode::RX};

    uint32_t detections{0};
    bool logging{true};
    std::unique_ptr<MeshtasticLogger> logger{};

    RFAmpField field_rf_amp{
        {13 * 8, 0 * 16}};
    LNAGainField field_lna{
        {15 * 8, 0 * 16}};
    VGAGainField field_vga{
        {18 * 8, 0 * 16}};
    RSSI rssi{
        {21 * 8, 0, 6 * 8, 4}};
    Channel channel{
        {21 * 8, 5, 6 * 8, 4}};

    RxFrequencyField field_frequency{
        {0 * 8, 0 * 16},
        nav_};

    /* Static detector panel -- counts stay put, they do not scroll. */
    Text text_pre{{0, 3 * 16, screen_width, 16}, ""};
    Text text_hdr{{0, 4 * 16, screen_width, 16}, ""};
    Text text_dec{{0, 5 * 16, screen_width, 16}, ""};
    Text text_addr{{0, 6 * 16, screen_width, 16}, ""};
    Text text_last{{0, 7 * 16, screen_width, 16}, ""};
    Text text_status{{0, 9 * 16, screen_width, 16}, ""};

    uint32_t n_pre{0};
    uint32_t n_hdr{0};
    uint32_t n_dec{0};
    uint32_t addrs[64]{};
    uint8_t n_addr{0};

    void refresh_stats();
    void note_addr(uint32_t a);

    void on_packet(const ACARSPacketMessage* packet);

    /* ACARSPacket is borrowed as the transport. Message IDs live in the
     * firmware's message.hpp, so defining a Meshtastic-specific one would
     * require rebuilding and reflashing the firmware -- and this app would
     * stop being an SD-card drop-in. An upstream PR should add a proper ID;
     * for a prototype that has to be installable without a reflash, reusing a
     * length-plus-buffer message is the right trade.
     */
    MessageHandlerRegistration message_handler_packet{
        Message::ID::ACARSPacket,
        [this](Message* const p) {
            const auto message = static_cast<const ACARSPacketMessage*>(p);
            this->on_packet(message);
        }};
};

}  // namespace ui::external_app::meshtastic_lf_rx

#endif /*__MESHTASTIC_APP_H__*/
