#pragma once

#include <span>
#include <cstdint>
#include "ictk/core/time.hpp"
#include "ictk/core/types.hpp"

namespace ictk{
    struct PlantState{

        // // View over size, no copy
        std::span<const Scalar> y;
        std::span<const Scalar> xhat;

        // // From time.hpp nanoseconds 
        t_ns t;

        // default: all channels valid
        std::uint64_t valid_bits{~0ull};
    };

    struct Setpoint{
        std::span<const Scalar> r;
        std::uint16_t preview_horizon_len{0};
    };

    struct UpdateContext{
        PlantState plant;
        Setpoint sp;
    };
    
    
} // namespace ictk
