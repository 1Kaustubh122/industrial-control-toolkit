#pragma once
#include <span>
#include <algorithm>
#include "ictk/core/types.hpp"

namespace ictk::safety{
    
    // // back calculation: e_aw = (u_sat - u_unsat) * Kt; caller integrates e_aw into integrator state
    inline void anti_windup_backcalc(
        std::span<const Scalar> u_unsat,
        std::span<const Scalar> u_sat,
        Scalar Kt,
        std::span<Scalar> e_aw_out
    ) noexcept{

        const std::size_t n = std::min({
            u_unsat.size(),
            u_sat.size(),
            e_aw_out.size()
            }        
        );
        
        for (std::size_t i=0; i<n; i++){
            e_aw_out[i] = (u_sat[i] - u_unsat[i]) * Kt;
        }
    }

    // // condition integration helper: zero e_aw when not saturated
    inline void anti_windup_conditional(
        std::span<const Scalar> u_unsat,
        std::span<const Scalar> u_sat,
        Scalar Kt,
        std::span<Scalar> e_aw_out
    ) noexcept{
        const std::size_t n = std::min({
            u_unsat.size(),
            u_sat.size(),
            e_aw_out.size()
        });

        for (std::size_t i=0; i<n; ++i){
            const bool sat = (u_sat[i] != u_unsat[i]);
            e_aw_out[i] = sat ? (u_sat[i] - u_unsat[i]) * Kt : 0;
        
        }
    }

} // namespace ictk::safety
