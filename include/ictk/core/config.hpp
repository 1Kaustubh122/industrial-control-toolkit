#pragma once

namespace ictk{
    #if defined(ICTK_NO_EXCEPTIONS)
    inline constexpr bool kNoExceptions = true;
    #else
    inline constexpr bool kNoExceptions = false;
    #endif

    #if defined(ICTK_NO_RTTI)
    inline constexpr bool kNoRTTI = true;
    #else
    inline constexpr bool kNoRTTI = false;
    #endif

} // namespace ictk