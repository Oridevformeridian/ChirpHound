/* Host shim: on the device this owns a thread pulling buffers off the radio.
 * Here main() feeds buffers directly, so the type only has to exist and be
 * constructible with the same arguments. */
#pragma once
#include <cstddef>

class BasebandProcessor;

namespace baseband {
enum class Direction { Receive, Transmit };
}

class BasebandThread {
   public:
    BasebandThread(size_t, BasebandProcessor*, baseband::Direction) {}
};
