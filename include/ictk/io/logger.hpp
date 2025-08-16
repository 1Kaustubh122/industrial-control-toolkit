#pragma once

#include "ictk/core/time.hpp"

namespace ictk{
    enum class LogLevel {
        kTrace,
        kDebug,
        kInfo,
        kWarn,
        kError
    };

    struct LoggerSink{
        virtual ~LoggerSink() = default;
        virtual void log(LogLevel lvl, const char *msg, t_ns t) noexcept = 0;
    };
    
} // namespace ictk
