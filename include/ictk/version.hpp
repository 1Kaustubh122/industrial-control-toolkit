#pragma once

#define ICTK_VERSION_MAJOR 0
#define ICTK_VERSION_MINOR 0
#define ICTK_VERSION_PATCH 2
#define ICTK_VERSION_STR   "0.0.2"
#define ICTK_C_ABI_VER     1


namespace ictk {
    constexpr int  kVersionMajor = ICTK_VERSION_MAJOR;
    constexpr int  kVersionMinor = ICTK_VERSION_MINOR;
    constexpr int  kVersionPatch = ICTK_VERSION_PATCH;
    constexpr char kVersionStr[] = ICTK_VERSION_STR;
    constexpr int  kCAbiVersion  = ICTK_C_ABI_VER;
} // namespace ictk

#if defined(__cplusplus)
extern "C"{
#endif
    const char* ictk_version_string() noexcept;
    int         ictk_c_abi_version() noexcept;
#if defined(__cplusplus)
}
#endif