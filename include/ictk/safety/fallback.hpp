#pragma once
#include <span>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cassert>

#include "ictk/core/time.hpp" 
#include "ictk/core/types.hpp" 
#include "ictk/core/memory_arena.hpp" 

/*
Goal: To create safety trips -> fallback controller.
    When the main controller can't be trusted -> then drive the plant toward a safe command at a bounded rate -> prevents actuator spikes
How: by replacing the controller's output with a safe vector safe_u and approch it smoothly. 
    
*/

namespace ictk::safety{
    class FallbackPolicy{
        public: 
            FallbackPolicy(
                std::span<const Scalar> safe_u, // // target safe command
                Scalar rmax,                    // // max change magnitude
                dt_ns dt,                       // // ticks
                MemoryArena& arena,             // // memory allocation
                std::size_t nu                  
            ) noexcept: safe_(safe_u), rmax_(rmax), dt_(dt){
                // // internal working copy stored in arena -> holds fallback last output
                u_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));  
                nu_ = nu; // // no of output channel
                // // seeds u[i] = safe[i] if present else 0
                if (u_) for (std::size_t i=0; i<nu; ++i) u_[i] = (i < safe_u.size() ? safe_u[i] : Scalar(0));
            }

            // // toggle fallback on
            void engage() noexcept{
                engaged_ = true;
            }

            // // toggle fallback off
            void disengage() noexcept{
                engaged_ = false;
            }

            // // check current status
            bool isengaged() const noexcept{
                return engaged_;
            }

            void apply(std::span<Scalar> u_out) noexcept{
                // // Guard
                if (!engaged_ || !u_) return;

                // // debug
                #ifndef NDEBUG
                    assert(u_out.size() >= nu_);
                #endif

                const Scalar step = rmax_ * Scalar(dt_) * 1e-9;  // per tick (in seconds) max move
                /*
                Each tick -> move each channel towards its safe target by at most rmax * dt
                write the new value to u_out and store it in u_ for next tick
                If already a target, du=0 and output holds
                
                Cost: O(n)
                */
                for (std::size_t i=0; i<nu_; ++i){
                    const Scalar target = (i < safe_.size() ? safe_[i] : Scalar(0));
                    const Scalar diff = target - u_[i];
                    const Scalar du = std::clamp(diff, -step, step);    // rate limit toward safe
                    u_[i] += du;
                    u_out[i] = u_[i];
                }
            }

            // // seeds u to current actuator output for bumpless entry
            void reset_to(std::span<const Scalar> u_now) noexcept{
                if (!u_) return;
                for (std::size_t i=0; i<nu_; ++i) u_[i] = (i<u_now.size() ? u_now[i] : Scalar(0));
            }

            // // helper functions
            void set_safe(std::span<const Scalar> s) noexcept{
                safe_ = s;
            } 
            void set_rmax(Scalar r) noexcept{
                rmax_ = r;
            }

        private:
            std::span<const Scalar> safe_{};
            Scalar* u_{nullptr};
            std::size_t nu_{0};
            Scalar rmax_{0};
            dt_ns dt_{0};
            bool engaged_{false};
    };
} // namespace ictk::safety
