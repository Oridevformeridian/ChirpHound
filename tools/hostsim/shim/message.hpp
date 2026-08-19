/* Host shim for the firmware's message.hpp.
 *
 * Only the members proc_meshtastic actually touches are reproduced -- msg_len,
 * message[], state. Field-for-field with the real ACARSPacketMessage so the
 * processor compiles unmodified; the rest of the firmware's message zoo is
 * irrelevant to the DSP under test. */
#pragma once
#include <cstdint>

class Message {
   public:
    enum class ID { ACARSPacket = 0 };
    constexpr Message(ID id) : id_{id} {}
    ID id_;
};

class ACARSPacketMessage : public Message {
   public:
    constexpr ACARSPacketMessage() : Message{ID::ACARSPacket} {}
    uint8_t msg_len = 0;
    char message[250] = {0};
    uint8_t crc[2] = {0};
    uint8_t state = 0;
};
