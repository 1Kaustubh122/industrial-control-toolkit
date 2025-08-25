#pragma once
#include <cstdint>
#include <algorithm>
#include "ictk/core/time.hpp"

namespace ictk::safety {

    class Watchdog {
        public:
            Watchdog(dt_ns dt_expected, std::uint32_t miss_threshold, dt_ns slack = 0) noexcept
            : dt_(dt_expected), slack_(std::max<dt_ns>(0, slack)), miss_thr_(miss_threshold) {}

            void reset(t_ns t0) noexcept{
                last_t_ = t0;
                misses_ = 0;
                tripped_=false;
            }

            // // Returns true when tripped
            bool tick(t_ns t_now) noexcept{
                if (last_t_ >= 0){
                    const auto d = t_now - last_t_;
                    if (d < dt_ - slack_ || d > dt_ + slack_){
                        if (++misses_ >= miss_thr_) tripped_ = true;
                    }
                }

                last_t_ = t_now;
                return tripped_;
            }

            bool tripped() const noexcept{
                return tripped_;
            }

            std::uint32_t misses() const noexcept{
                return misses_;
            }

        private:
            dt_ns dt_{0}, slack_{0};
            t_ns last_t_{-1};
            std::uint32_t misses_{0}, miss_thr_{1};
            bool tripped_{false};
    };
} // namespace ictk::safety
