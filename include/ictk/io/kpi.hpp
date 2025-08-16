#pragma once

#include <cstdint>


namespace ictk{

    struct KpiCounters{
        // // Updates will increase monotonically (proves that the controller is alive)
        std::uint64_t updates{0};

        // // watchdog_trips > 0, means violating your real time contract
        std::uint64_t watchdog_trips{0};

        // // fallback_entries > 0, means your fancy controller failed -> Shift to baseline for safety
        std::uint64_t fallback_entries{0};

        // //  Too high means, commands are beyond pyhsical capability -> poor tuning, wrong model or unsafe request
        std::uint64_t limit_hits{0};
    };
    

} // namespace ictk
