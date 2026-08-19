/*
 * Meshtastic / LoRa receiver, external app entry point.
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

#include "ui.hpp"
#include "meshtastic_app.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::meshtastic_lf_rx {
void initialize_app(ui::NavigationView& nav) {
    nav.push<MeshtasticLfRxView>();
}
}  // namespace ui::external_app::meshtastic_lf_rx

extern "C" {

__attribute__((section(".external_app.app_meshtastic_lf_rx.application_information"), used)) application_information_t _application_information_meshtastic_lf_rx = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::meshtastic_lf_rx::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "MeshtstcLF",
    /*.bitmap_data = */ {
        /* A mesh: three nodes joined by links. 16x16, 1bpp, two bytes a row. */
        0x00,
        0x00,
        0x18,
        0x18,
        0x3C,
        0x3C,
        0x18,
        0x18,
        0x00,
        0x00,
        0x81,
        0x81,
        0xC3,
        0xC3,
        0x66,
        0x66,
        0x3C,
        0x3C,
        0x18,
        0x18,
        0x3C,
        0x3C,
        0x66,
        0x66,
        0xC3,
        0xC3,
        0x81,
        0x81,
        0x00,
        0x00,
        0x00,
        0x00,
    },
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_meshtastic_lf */ {'P', 'M', 'L', 'F'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
