#include "ictk/version.hpp"
#include "ictk/visibility.hpp"

extern "C"{
    ICTK_API const char* ictk_version_string() noexcept {
        return ictk::kVersionStr;
    }

    ICTK_API int ictk_c_abi_version() noexcept{
        return ictk::kCAbiVersion;
    }
}
