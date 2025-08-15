# pragma once
#include <cstdint>

namespace ictk{
    struct ControllerHealth{
        std::uint64_t deadline_miss_count{0};
        double saturation_pct{0.0};
        std::uint64_t rate_limit_hits{0};
        bool fallback_active{false};
        bool novelty_flag{false};


        void clear_runtime(){
            saturation_pct = 0.0;
            rate_limit_hits = 0;
        }
    };
    
    
} // namespace ictk
