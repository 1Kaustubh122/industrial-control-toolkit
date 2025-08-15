#pragma once

#include <cstdint>

namespace ictk{
    enum class Status : std::uint8_t{
        kOK = 0,
        kInvalidArg,
        kPreconditionFail,
        kNotReady,
        kDeadlineMiss,
        kNoMem,
    };
    
} // namespace ictk
