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
Goal: To cap both rate and jerk of the command vector u for each tick.
    By storing last output and the last step in arena memory, then clam the desired step (jerk and rate band).
    rate limit: controls how fast the command move -> prevent spikes
    jerk limit: controls how fast the rate changes -> reduces mechanical stress 
    together: make it smooth and actuator friendly profile

    jerk band: [last_step(dprev) - jmax*dt, last_step(dprev) + jmax*dt]
    rate band: [-rmax*dt, +rmax*dt]
*/
namespace ictk::safety{
    class JerkLimiter{
        public:
            explicit JerkLimiter(Scalar rmax, Scalar jmax, dt_ns dt, MemoryArena &arena, std::size_t nu) noexcept
            :   rmax_(rmax),    // max rate (unit of u per second)
                jmax_(jmax),    // max jerk (unit rate per second = u/s^2) -> but del rate = (du - dprev) / dt_s
                dt_(dt)         // tick period in nano seconds -> later converted into seconds 
            {
                prev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));   // last output -> u[i] at k-1
                dprev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));  // last step   -> u[k-1] - u[k-2]
                nu_ = nu;

                // // seeds both to 0
                if (prev_) for (std::size_t i=0; i<nu;++i) prev_[i]=0;
                if (dprev_) for (std::size_t i=0; i<nu; ++i) dprev_[i]=0;
            }

            std::uint64_t apply(std::span<Scalar> u) noexcept{
                if (!prev_ || !dprev_) return 0;
                
                const Scalar dt_s = Scalar(dt_) * 1e-9; // ns to sec conv
                const Scalar rstep = rmax_ * dt_s;      // max |du| this tick
                const Scalar jstep = jmax_ * dt_s;      // max |du - dprev| this tick

                std::uint64_t hits = 0;

                // // O(n) cost
                for (std::size_t i=0; i<u.size(); ++i){
                    const Scalar du_des = u[i] - prev_[i]; // desired step
                    const Scalar lo_j = dprev_[i] - jstep;
                    const Scalar hi_j = dprev_[i] + jstep;

                    Scalar du = std::clamp(du_des, lo_j, hi_j); // // allowed step + jerk limit
                    du = std::clamp(du, -rstep, rstep);         // // adding rate limit
                    const Scalar u_new = prev_[i] + du;
                    if (u_new != u[i]) hits++;  // // count hits if clamped
                    dprev_[i] = du;     // // store step as new prev step
                    prev_[i] = u_new;   // // store output as new prev out
                    u[i] = u_new;       // // write back
                }
                return hits;
            }

            void reset(std::span<const Scalar> u0) noexcept{
                if (!prev_ || !dprev_) return;
                for (std::size_t i=0; i<nu_; i++){
                    prev_[i] = (i < u0.size() ? u0[i] : 0);
                    dprev_[i] = 0;
                }
            }

        private:
            Scalar rmax_{0}, jmax_{0};
            dt_ns dt_{0};
            Scalar* prev_{nullptr};
            Scalar* dprev_{nullptr};
            std::size_t nu_{0};
    };
} // namespace ictk::safety
