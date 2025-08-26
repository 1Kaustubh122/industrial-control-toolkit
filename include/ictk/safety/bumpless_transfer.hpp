#pragma once
#include <span>
#include <cstddef>
#include <cassert>
#include <algorithm>

#include "ictk/core/types.hpp"

/*
Goal: Smooth handover between controllers
    handover controller might cause spikes, saturation or transient 
Fix: Instead of u_out = u_hold, slowly increase alpha toward 1.
    at alpha = 0,   u_out = u_hold 
    at alpha = 0.2, u_out = u_hold (80%) + u_new (20%)
    .
    .
    at alpha = 1 ,  u_out = u_new (fully new)

    smooth crossfade instead of jump.
    
    u_out = (1-alpha)u_hold + alpha*u_new : where, alpha belongs to [0, 1] 
*/
namespace ictk::safety{
    class BumplessMixer{
        public:
            explicit BumplessMixer(Scalar alpha = 0.2) noexcept : alpha_(alpha) {}

            static void mix(
                std::span<const Scalar> u_hold, // last actuator output
                std::span<const Scalar> u_new,  // incoming controller's output
                std::span<Scalar>       u_out,  // final commands that will go to the actuators
                Scalar alpha) noexcept{

                    
                    // [0,1]
                    const Scalar a = std::clamp(alpha, Scalar(0), Scalar(1)); 
                    const Scalar b = Scalar(1) - a;
                    
                    #ifndef NDEBUG
                        assert(u_hold.size() == u_new.size());
                        assert(u_out.size() == u_new.size());
                    #endif

                    // // safe length, uses the smalleset span to avoid OOB writes
                    const std::size_t n = std::min({
                        u_hold.size(),
                        u_new.size(),
                        u_out.size()
                    });
                                                        // (1-alpha)*u_hold + alpha*u_new 
                    for (std::size_t i=0; i<n; ++i) u_out[i] = b*u_hold[i] + a*u_new[i];
                }

                void setup(
                    std::span<const Scalar> u_hold,
                    std::span<const Scalar> u_new,
                    std::span<Scalar>       u_out) 
                    const noexcept{
                        mix(u_hold, u_new, u_out, alpha_);
                    }
                
                // // helper functions

                // increase alpha value
                void step_alpha(Scalar delta) noexcept{
                    alpha_ = std::clamp(alpha_ + std::max<Scalar>(0, delta), Scalar(0), Scalar(1));
                }

                // get alpha value 
                Scalar alpha() const noexcept{
                    return alpha_;
                }; 

                // set alpha value
                void set_alpha(Scalar a) noexcept{
                    alpha_ = std::clamp(a, Scalar(0), Scalar(1));
                }
        
        private:
            Scalar alpha_{0.2};
    };
} // namespace ictk::safety

