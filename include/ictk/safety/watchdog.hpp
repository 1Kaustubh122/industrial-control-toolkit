#pragma once
#include <cstdint>
#include <algorithm>

#include "ictk/core/time.hpp"
 
/*
    Goal: To check the control loop ticks at the expected period

    tolerates small jitter(slack) -> counts deadline misses and trips after a threshold

    use: to decide when to enter fallback
 */
namespace ictk::safety {
    // // Deadline watch dog (tick-driven) -> counts misses and trips after threshold
    class Watchdog {
        public:
            /*
                dt_expected: target timeperiod 
                miss_threshold: count of violations
                slack: allowed jutter around dt_expected
            */
            Watchdog(dt_ns dt_expected, std::uint32_t miss_threshold, dt_ns slack = 0) noexcept
            : dt_(dt_expected), slack_(std::max<dt_ns>(0, slack)), miss_thr_(miss_threshold) {}

            // // set the last seen time to t0, clears counter and trip states
            void reset(t_ns t0) noexcept{
                last_t_ = t0;   // establishes baseline
                misses_ = 0;    // cumulative miss counter
                tripped_=false; // clear trip
            }

            // // Returns true when tripped -> call once per loop with the current time
            bool tick(t_ns t_now) noexcept{
                if (dt_ <= 0 || miss_thr_ == 0){  // misconfigured
                    last_t_ = t_now;
                    return tripped_;
                }
                if (last_t_ >= 0){                  // ignore first tick after construction
                    const auto d = t_now - last_t_;
                    if (d < dt_ - slack_ || d > dt_ + slack_){      // outside tolerance window
                        if (++misses_ >= miss_thr_) tripped_ = true; // increment misses and trip = true
                    }
                }

                last_t_ = t_now;
                return tripped_;
            }

            // // helper functions
            bool tripped() const noexcept{
                return tripped_;
            }
            std::uint32_t misses() const noexcept{
                return misses_;
            }

        private:
            dt_ns dt_{0}, slack_{0};
            t_ns last_t_{-1};       // sentinel: no prior tick
            std::uint32_t misses_{0}, miss_thr_{1};
            bool tripped_{false};
    };
} // namespace ictk::safety
