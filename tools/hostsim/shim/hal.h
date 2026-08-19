/* Host shim: the only thing dsp_fft.hpp wants from ChibiOS/HAL is __RBIT,
 * an ARM intrinsic that reverses the bits of a word. Everything else in
 * common/ is portable C++, so this one function is the whole port. */
#pragma once
#include <cstdint>

static inline uint32_t __RBIT(uint32_t v) {
    v = ((v & 0x55555555u) << 1) | ((v >> 1) & 0x55555555u);
    v = ((v & 0x33333333u) << 2) | ((v >> 2) & 0x33333333u);
    v = ((v & 0x0F0F0F0Fu) << 4) | ((v >> 4) & 0x0F0F0F0Fu);
    v = (v << 24) | ((v & 0xFF00u) << 8) | ((v >> 8) & 0xFF00u) | (v >> 24);
    return v;
}
