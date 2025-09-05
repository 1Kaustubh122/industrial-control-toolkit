#pragma once
#include "ictk/core/types.hpp"

namespace ictk::safety{
struct Clip{
    Scalar val;
    bool   hit;
    Scalar mag;
};
} // namespace ictk::safety
