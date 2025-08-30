#pragma once
#include <span>
#include <cmath>
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

    // Avoid duplicate definition if other safety headers also define Clip.
    #ifndef ICTK_SAFETY_CLIP_DEFINED
    #define ICTK_SAFETY_CLIP_DEFINED
    struct Clip{
        Scalar val; // final clamp value
        bool hit;   // was clamped?
        Scalar mag; // how far outside the limit is
    };

    static inline Clip jerk_limit_scalar(
        Scalar u_now,
        Scalar u_prev,
        Scalar du_prev,
        Scalar ddu_max
    ) noexcept{
        const Scalar lo = du_prev - ddu_max;
        const Scalar hi = du_prev + ddu_max;

        Scalar du = u_now - u_prev; // desired step this tick
        Scalar mag = Scalar(0);
        bool hit = false;

        if (du < lo){
            mag = std::abs(lo - du);
            du = lo; 
            hit =true;
        }else if(du > hi){
            mag = std::abs(du - hi);
            du = hi;
            hit = true;
        }

        return {u_prev + du, hit, mag};
    }

    #endif  
    class JerkLimiter{
        public:
            explicit JerkLimiter(Scalar rmax, Scalar jmax, dt_ns dt, MemoryArena &arena, std::size_t nu) noexcept
            :   rmax_(rmax),    // max rate (unit of u per second)
                jmax_(jmax),    // max jerk (unit rate per second = u/s^2) -> but del rate = (du - dprev) / dt_s
                dt_(dt)         // tick period in nano seconds -> later converted into seconds 
            {
                #ifndef NDEBUG
                    assert(rmax_ >= Scalar(0) && jmax_ >= Scalar(0));
                    assert(nu > 0);
                #endif

                prev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));   // last output -> u[i] at k-1
                dprev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));  // last step   -> u[k-1] - u[k-2]
                nu_ = nu;

                // // seeds both to 0
                if (prev_) for (std::size_t i=0; i<nu;++i) prev_[i]=Scalar(0);
                if (dprev_) for (std::size_t i=0; i<nu; ++i) dprev_[i]=Scalar(0);
            }

            std::uint64_t apply(std::span<Scalar> u) noexcept{
                if (!prev_ || !dprev_) return 0;

                std::uint64_t hits = 0;
                last_mag_ = Scalar(0);
                
                const Scalar dt_s = Scalar(dt_) * 1e-9; // ns to sec conv
                const Scalar rstep = rmax_ * dt_s;      // max |du| this tick
                const Scalar jstep = jmax_ * dt_s;      // max |du - dprev| this tick

                #ifndef NDEBUG
                    assert(std::isfinite(dt_s) && dt_s > Scalar(0));
                #endif

                // // O(n) cost
                 for (std::size_t i = 0; i < u.size(); ++i){
                    const Scalar lo_r = prev_[i] - rstep;
                    const Scalar hi_r = prev_[i] + rstep;

                    // Rate clamp of output.
                    const Scalar u_rate = std::clamp(u[i], lo_r, hi_r);

                    // Jerk clamp of step.
                    const Clip c = jerk_limit_scalar(u_rate, prev_[i], dprev_[i], jstep);

                    u[i] = c.val;
                    dprev_[i] = u[i] - prev_[i];
                    prev_[i] = u[i];

                    if (c.hit){     // if clamped -> increment hit
                        ++hits;
                        if (c.mag > last_mag_) last_mag_ = c.mag;
                    }
                }
                return hits;
            }

            void reset(std::span<const Scalar> u0) noexcept{
                if (!prev_ || !dprev_) return;

                for (std::size_t i = 0; i < nu_; ++i) {
                    prev_[i]  = (i < u0.size() ? u0[i] : Scalar(0));
                    dprev_[i] = Scalar(0);
                }
                last_mag_ = Scalar(0);
            }

            // // helper functions
            bool valid() const noexcept{ 
                return prev_ != nullptr && dprev_ != nullptr;
            }
            Scalar last_clip_mag() const noexcept{
                return last_mag_; 
            }

        private:
            Scalar rmax_{0}, jmax_{0};
            dt_ns dt_{0};

            Scalar* prev_{nullptr};
            Scalar* dprev_{nullptr};
            std::size_t nu_{0};

            Scalar last_mag_{0};
    };
} // namespace ictk::safety
