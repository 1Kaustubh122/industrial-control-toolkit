#pragma once
#include "ictk/version.hpp"

#define ICTK_ACR_VERSION_MAJOR ICTK_VERSION_MAJOR
#define ICTK_ACR_VERSION_MINOR 1
#define ICTK_ACR_VERSION_PATCH 0
#define ICTK_ACR_VERSION_STR   "0.1.0"

namespace ictk::tools::acr{
    constexpr int kVersionMajor = ICTK_ACR_VERSION_MAJOR;
    constexpr int kVersionMinor = ICTK_ACR_VERSION_MINOR;
    constexpr int kVersionPatch = ICTK_ACR_VERSION_PATCH;
    constexpr char kVersionStr[] = ICTK_ACR_VERSION_STR;
} // ictk::tools::acr