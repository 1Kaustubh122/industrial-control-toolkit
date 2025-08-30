#pragma once
#include <span>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cassert>
#include <algorithm>

#include "ictk/core/time.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/memory_arena.hpp"

/*
Goal: To bound, how fast each command element can change per tick
    
    It supports per channel tick or one uniform limit, keeps the previous output in arena backend memory for RT safety,
    and returns how many elements were clamped this tick
*/
namespace ictk::safety{
    struct Clip{
        Scalar val; //final clamp value
        bool hit;   // was clamped?
        Scalar mag; // how far outside the limit is
    };

    // helper function
    static inline Clip rate_limiter_scalar(Scalar u_now, Scalar u_prev, Scalar du_max){
        const Scalar lo=u_prev - du_max, hi = u_prev + du_max;
        if (u_now < lo) return {lo, true, std::abs(lo - u_now)};
        if (u_now > hi) return {hi, true, std::abs(u_now - hi)};
        return {u_now, false, Scalar(0)};
    }
    
    class RateLimiter{
        public:
            /*
                rmax:       rate limit per channel 
                rmax_s_:    uniform max rate
                dt_:        tick length in ns (nano seconds)
                prev_  :    pointer to prev output
                last_mag:   biggest clip mag in last tick
            */ 
            // // Per channel limit
            RateLimiter(std::span<const Scalar> rmax, dt_ns dt, MemoryArena &arena, std::size_t nu) noexcept
            : rmax_(rmax), dt_(dt){
                prev_ = static_cast<Scalar*>(arena.allocate(nu * sizeof(Scalar), alignof(Scalar))); // // last emitted ouput of each channel
                nu_ = nu;
                if (prev_) for (std::size_t i=0; i<nu; ++i) prev_[i] = 0;                           // // zero init
            }

            // // uniform rmax for all channel  || No implicit conversions || Same limit for all channel
            explicit RateLimiter (Scalar rmax_uniform, dt_ns dt, MemoryArena& arena, std::size_t nu) noexcept
            : dt_(dt), rmax_s_(rmax_uniform){
                prev_ = static_cast<Scalar*>(arena.allocate(nu * sizeof(Scalar), alignof(Scalar)));
                nu_ = nu;
                if (prev_) for (std::size_t i=0; i<nu; ++i) prev_[i] = 0;
            }


            std::uint64_t apply(std::span<Scalar> u) noexcept{
                if (!prev_) return 0;                   // // inert if no storage
                std::uint64_t hits=0;
                last_mag_ = 0;
                const Scalar dts = Scalar(dt_) * 1e-9;  // // convert ns to seconds 
                const bool per = !rmax_.empty();
            
                #ifndef NDEBUG
                    if (per) assert(rmax_.size() >= u.size());      // shape guard for DEBUG
                #endif
                    for (std::size_t i=0; i<u.size(); ++i){
                        const Scalar r = per ? rmax_[i] : rmax_s_;  // pick limit r
                        const auto c = rate_limiter_scalar(u[i], prev_[i], r*dts);
                        u[i] = c.val;
                        prev_[i] = u[i];
                        if (c.hit){     // if clamped -> increment hit
                            ++hits;
                            last_mag_ = std::max(last_mag_, c.mag);
                        }
                    }

                    return hits;
            }

            // reset to given initial vector 
            void reset(std::span<const Scalar> u0) noexcept{
                if (!prev_) return;
                for (std::size_t i=0; i<nu_; ++i) prev_[i] = (i < u0.size() ? u0[i] : 0);
            }

            // chck alloc
            bool valid() const noexcept{
                return prev_!=nullptr;
            }

            // biggest mag clipped
            Scalar last_clip_mag() const noexcept{
                return last_mag_;
            }

            bool valid() const noexcept{
                return prev_ != nullptr;
            }

        private:
            std::span <const Scalar> rmax_{};
            dt_ns dt_{0};
            Scalar* prev_{nullptr};
            std::size_t  nu_{0};
            Scalar rmax_s_{0};
            Scalar last_mag_{0};
    };
} // namespace ictk::safety
