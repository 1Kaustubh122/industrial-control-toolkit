#pragma once

#include <span>
#include "ictk/core/types.hpp"
#include "ictk/core/health.hpp"

namespace ictk{
    struct  Result{

        // // Output command buffer (length mu) <- By caller | u -> actual control signal | No Copy
        std::span<Scalar> u;

        // // Status report of the controller after producing u
        ControllerHealth health;

        // // diag fields can be extended, after adding more controller mode, or condition numbers of MPC
    };
    
} // namespace ictk
