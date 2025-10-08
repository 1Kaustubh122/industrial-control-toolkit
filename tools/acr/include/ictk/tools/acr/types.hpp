#pragma once

#include <vector>
#include <limits>
#include <string>
#include <cstdint>

namespace ictk::tools::acr{
    enum class ClockDomain: uint8_t{
        // monotonic clock
        MONO = 0,

        // wall real clock timer
        WALL = 1,

        // non identified 
        UNKNOWN = 244
    };

    // // Per tick canonical row 
    struct CanonicalRow{
        // MCAP timestamp
        std::int64_t    t_ns;

        // per file sequence
        std::uint64_t   seq{0};

        // index in mcap_path
        std::uint16_t   file_idx{0};

        // keep n-byte alignment
        std::uint16_t _pad{0};

        // // Signals
        // measured value
        double y0{0.0};

        // setpoint/reference point
        double r0{0.0};

        // controller output pre safety
        double u_pre{0.0};

        // command after safety
        double u_post{0.0};

        // bit mask for extra conditions
        std::uint32_t flags{0};

        // Control mode
        std::uint32_t mode{0};

        // saturation pct
        double sat_pct{0.0};
    };

    // // Stream/session meta
    struct CanonicalMeta{
        // base cycle time
        std::int64_t dt_ns{0};

        std::uint32_t tick_decimation{1};
    
        ClockDomain clock{ClockDomain::UNKNOWN};

        // identity
        std::string controller_id;
        std::string asset_id;

        // host timing context
        std::string kernel_clocksource;

        // schema hashed from sidecar and MCAP
        std::vector<std::string> bfbs_sidecar_sha_256;
        std::vector<std::string> bfbs_mcap_snapshot;

        // helpers
        bool    in_zone{false};
        double  v_safe{
            std::numeric_limits<double>::infinity()
        };
    };

    // events probe
    struct EventsProbe{
        bool            present{false};
        std::uint64_t   lines_total{0};
        std::uint64_t   lines_bad{0};
        std::uint64_t   estop_true_count{0};
    };
} // namespace ictk::tools::acr