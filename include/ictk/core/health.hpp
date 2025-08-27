# pragma once
#include <cstdint>

namespace ictk{
    struct ControllerHealth{

        // // count of times the controller failed to produce output within the allowed deadline
        std::uint64_t deadline_miss_count{0};

        // // percentage of channels at saturatrion (clamped at actuator limits) in the last tick
        double saturation_pct{0.0};

        // // Number of times output changes were clamped by the rate limiter
        std::uint64_t rate_limit_hits{0};

        // // Number of times output changes were clamped by the jerk limiter
        std::uint64_t jerk_limit_hits{0};

        // // true if current controller is not trusted -> fallback mode toggle
        bool fallback_active{false};

        // // marker if inputs/plant state triggered an out of distribution
        bool novelty_flag{false};

        // // magniture of the anti windup correction term applied at the last tick
        double aw_term_mag{0.0};

        // // magnitude of last saturation clamp
        double last_clamp_mag{0.0};

        // //magnitude of last rate clip
        double last_rate_clip_mag{0.0};

        // // mag of last jerk clip
        double last_jerk_clip_mag{0.0};

        
        // // Reset per tick runtime counter to 0
        void clear_runtime(){
            saturation_pct = 0.0;
            rate_limit_hits = 0;
            jerk_limit_hits = 0;
            aw_term_mag = 0.0;
            last_clamp_mag= 0.0;
            last_rate_clip_mag = 0.0;
            last_jerk_clip_mag = 0.0;
        }
    };

    static_assert(sizeof(ControllerHealth) <= 128, "keep health small/fixed");
    
} // namespace ictk
