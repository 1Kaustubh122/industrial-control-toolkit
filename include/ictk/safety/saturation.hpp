#pragma once

#include <span>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cassert>

#include "ictk/core/types.hpp"

/*
Goal: 
    Protect actuators: torque/voltage/current/position bounds
    Keep commands in the feasible set before rate/jerk limit and before sending to the hardware
    KPIs: hits and saturation+pct : health metrics and alerts
*/
// // clamp stage -> enforces actuator limits 
namespace ictk::safety{
    struct SatReport{
        std::uint64_t hits{0};      // how many elements were clamped
        double saturation_pct{0.0}; // hits / u.size() * 100
    };

    class Saturation{
        public:
            Saturation(std::span<const Scalar> umin, std::span<const Scalar> umax) noexcept : umin_(umin), umax_(umax) {}

            explicit Saturation(Scalar umin, Scalar umax) noexcept : umin_s_(umin), umax_s_(umax) {}
            
            SatReport apply(std::span<Scalar> u) const noexcept{
                SatReport rep{};
                const bool per = (!umin_.empty() && !umax_.empty());

                #ifndef NDEBUG
                    if (per){
                        assert(umin_.size() >= u.size() && umax_.size() >= u.size());
                    }
                #endif

                // // choose limits per elements
                for (std::size_t i=0; i<u.size(); ++i){
                    // // clamp each u[i] to [low, high]
                    const Scalar lo = per ? umin_[i] : umin_s_;
                    const Scalar hi = per ? umax_[i] : umax_s_;

                    // // counts each elements that was clamped
                    if (u[i] < lo){
                        u[i] = lo;
                        rep.hits++;
                    }else if (u[i] > hi){
                        u[i] = hi;
                        rep.hits++;
                    }

                }

                // // computes percent of channels that hit a clamp this tick 
                if (!u.empty()){
                    rep.saturation_pct = 100.0 * double(rep.hits) / double(u.size());
                }
                return rep;

                // // cost: O(n)
            }
        
        private:
            std::span<const Scalar> umin_{}, umax_{};
            Scalar umin_s_{0}, umax_s_{0};
    };
    
} // namespace ictk::safety
