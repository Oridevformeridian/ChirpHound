#pragma once
#include "dsp_types.hpp"

class BasebandProcessor {
   public:
    virtual ~BasebandProcessor() = default;
    virtual void execute(const buffer_c8_t& buffer) = 0;
};
