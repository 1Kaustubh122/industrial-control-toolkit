#pragma once
#include <span>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cassert>

#include "ictk/core/time.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/memory_arena.hpp"


namespace ictk::safety{
    class JerkLimiter{
        public:
            explicit JerkLimiter(Scalar rmax, Scalar jmax, dt_ns dt, MemoryArena &arena, std::size_t nu) noexcept
            : rmax_(rmax), jmax_(jmax), dt_(dt){
                prev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));
                dprev_ = static_cast<Scalar*>(arena.allocate(nu*sizeof(Scalar), alignof(Scalar)));
                nu_ = nu;
                if (prev_) for (std::size_t i=0; i<nu;++i) prev_[i]=0;
                if (dprev_) for (std::size_t i=0; i<nu; ++i) dprev_[i]=0;
            }

            std::uint64_t apply(std::span<Scalar> u) noexcept{
                if (!prev_ || !dprev_) return 0;
                
                const Scalar dt_s = Scalar(dt_) * 1e-9;
                const Scalar rstep = rmax_ * dt_s;
                const Scalar jstep = jmax_ * dt_s;

                std::uint64_t hits = 0;

                for (std::size_t i=0; i<u.size(); ++i){
                    const Scalar du_des = u[i] - prev_[i];
                    const Scalar lo_j = dprev_[i] - jstep;
                    const Scalar hi_j = dprev_[i] + jstep;

                    Scalar du = std::clamp(du_des, lo_j, hi_j);
                    du = std::clamp(du, -rstep, rstep);
                    const Scalar u_new = prev_[i] + du;
                    if (u_new != u[i]) hits++;
                    dprev_[i] = du;
                    prev_[i] = u_new;
                    u[i] = u_new;
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
