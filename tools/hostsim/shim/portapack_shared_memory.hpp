/* Host shim: the M4->M0 queue becomes an in-process vector the harness drains
 * after every buffer, which is what lets main() render the same log lines the
 * app writes to MESHTAST.TXT. */
#pragma once
#include <vector>
#include "message.hpp"

struct AppQueue {
    std::vector<ACARSPacketMessage> items;
    void push(const ACARSPacketMessage& m) { items.push_back(m); }
};

struct SharedMemory {
    AppQueue application_queue;
};

extern SharedMemory shared_memory;
