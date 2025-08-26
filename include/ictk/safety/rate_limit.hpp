#pragma once
#include <span>
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
    class RateLimiter{
        public:
            // rmax: rate limit per channel 
            RateLimiter(std::span<const Scalar> rmax, dt_ns dt, MemoryArena &arena, std::size_t nu) noexcept
            : rmax_(rmax), dt_(dt){
                prev_ = static_cast<Scalar*>(arena.allocate(nu * sizeof(Scalar), alignof(Scalar))); // // last emitted ouput of each channel
                nu_ = nu;
                if (prev_) for (std::size_t i=0; i<nu; ++i) prev_[i] = 0;                           // // zero init
            }

            // uniform rmax for all channel  || No implicit conversions
            explicit RateLimiter (Scalar rmax_uniform, dt_ns dt, MemoryArena& arena, std::size_t nu) noexcept
            : dt_(dt), rmax_s_(rmax_uniform){
                prev_ = static_cast<Scalar*>(arena.allocate(nu * sizeof(Scalar), alignof(Scalar)));
                nu_ = nu;
                if (prev_) for (std::size_t i=0; i<nu; ++i) prev_[i] = 0;
            }


            std::uint64_t apply(std::span<Scalar> u) noexcept{
                if (!prev_) return 0;                   // // inert if no storage
                const Scalar dts = Scalar(dt_) * 1e-9;  // // convert ns to seconds 
                std::uint64_t hits=0;
                const bool per = !rmax_.empty();
            
                #ifndef NDEBUG
                    if (per) assert(rmax_.size() >= u.size());      // shape guard for DEBUG
                #endif
                    for (std::size_t i=0; i<u.size(); ++i){
                        const Scalar r = per ? rmax_[i] : rmax_s_;
                        const Scalar step = r * dts;                // max allowed delta this tick
                        const Scalar lo = prev_[i] - step;
                        const Scalar hi = prev_[i] + step;

                        // // count of hit whenever clamped
                        if (u[i] < lo){
                            u[i] = lo;
                            hits++;
                        }else if (u[i] > hi){
                            u[i] = hi;
                            hits++;
                        }
                        prev_[i] = u[i];        // for next tick
                    }

                    return hits;
            }

            void reset(std::span<const Scalar> u0) noexcept{
                if (!prev_) return;
                for (std::size_t i=0; i<nu_; ++i) prev_[i] = (i < u0.size() ? u0[i] : 0);
            }

        private:
            std::span <const Scalar> rmax_{};
            dt_ns dt_{0};
            Scalar* prev_{nullptr};
            std::size_t  nu_{0};
            Scalar rmax_s_{0};
    };
} // namespace ictk::safety
