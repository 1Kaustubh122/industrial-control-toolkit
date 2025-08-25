#pragma once

#include <span>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cassert>

#include "ictk/core/types.hpp"

namespace ictk::safety{
    struct SatReport{
        std::uint64_t hits{0};
        double saturation_pct{0.0};
    };

    class Saturation{
        public:
            Saturation(std::span<const Scalar> umin, std::span<const Scalar> umax) noexcept : umin_(umin), umax_(umax) {}

            explicit Saturation(Scalar umin, Scalar umax) noexcept : umin_s_(umin), umax_s_(umax) {}
            
            SatReport apply(std::span<Scalar> u) const noexcept{
                SatReport rep{};
                const bool per = (!umin_.empty() && !umax_.empty());

                #ifndef NODEBUG
                    if (per){
                        assert(umin_.size() >= u.size() && umax_.size() >= u.size());
                    }
                #endif

                for (std::size_t i=0; i<u.size(); ++i){
                    const Scalar lo = per ? umin_[i] : umin_s_;
                    const Scalar hi = per ? umax_[i] : umax_s_;

                    if (u[i] < lo){
                        u[i] = lo;
                        rep.hits++;
                    }else if (u[i] > hi){
                        u[i] = hi;
                        rep.hits++;
                    }

                    if (!u.empty()){
                        rep.saturation_pct = 100.0 * double(rep.hits) / double(u.size());
                        return rep;
                    }
                }
            }
        
        private:
            std::span<const Scalar> umin_{}, umax_{};
            Scalar umin_s_{0}, umax_s_{0};
    };
    
} // namespace ictk::safety
