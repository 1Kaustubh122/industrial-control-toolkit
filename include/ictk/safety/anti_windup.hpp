#pragma once
#include <span>
#include <algorithm>
#include "ictk/core/types.hpp"

// // anti-windup: damps or zeroes the integrator while the output is saturated
namespace ictk::safety{
    
    // single source for AW modes
    enum class AWMode : std::uint8_t{
        kBackCalc,
        kConditional,
        kOff
    };

    // helpers
    inline Scalar aw_backcalc_term(Scalar u_unsat, Scalar u_sat, Scalar Kt) noexcept{
        return (u_sat - u_unsat) * Kt;
    }

    inline Scalar aw_conditional_term(Scalar u_unsat, Scalar u_sat, Scalar Kt) noexcept{
        return (u_sat != u_unsat) ? (u_sat - u_unsat) * Kt : Scalar(0);
    }

    // // back calculation: e_aw = (u_sat - u_unsat) * Kt; caller integrates e_aw into integrator state
    inline void anti_windup_backcalc(
        std::span<const Scalar> u_unsat,  // // vector of raw controller output before safety limits (raw math results)
        std::span<const Scalar> u_sat,    // // vector of saturated output: post-limit output after safety clamps 
        Scalar Kt,                        // // tracking gain, tunes how aggressively to unwind
        std::span<Scalar> e_aw_out        // // vector to feed to the integrator as an anti-windup correction term 
    ) noexcept{

        const std::size_t n = std::min({
            u_unsat.size(),
            u_sat.size(),
            e_aw_out.size()
            }        
        );
        
        // // compute e_aw for each channel
        /*
        if u_sat < u_unsat -> negative correction -> push integrator down  (clamped high)
        if u_sat > u_unsat -> positive correction -> push integrator up    (clamped low)
        */
        for (std::size_t i=0; i<n; i++){
            aw_backcalc_term(u_unsat[i], u_sat[i], Kt);
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
            // // if saturated
            // // when not saturated sets correction to 0
            e_aw_out[i] = aw_conditional_term(u_unsat[i], u_sat[i], Kt);
        }
    }

} // namespace ictk::safety
