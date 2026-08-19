/* Host shim: the baseband image's main() builds an EventDispatcher and runs
 * it forever. The harness never calls that main (it is renamed away at compile
 * time), so this only has to satisfy the compiler. */
#pragma once
#include <memory>

class BasebandProcessor;

class EventDispatcher {
   public:
    explicit EventDispatcher(std::unique_ptr<BasebandProcessor>&&) {}
    void run() {}
};
