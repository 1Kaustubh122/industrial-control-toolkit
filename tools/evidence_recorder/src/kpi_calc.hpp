#pragma once

#include <cstddef>
#include <cstdint>

namespace ictk::tools::detail{
    struct KpiAcc{
        // // Running sums
        double iae{0.0};
        double itae{0.0};
        double tvu{0.0};
        double last_u{0.0};
        bool have_u{false};

        // Latency reservoir 
        static constexpr std::size_t kLatCap = 2048;
        double lat_us[kLatCap]; // ring buffer

        // total seen 
        std::size_t lat_count{0};

        // next write index
        std::size_t lat_head{0};

        // // Derived percentiles 
        double p50_lat_us{0.0}, p95_lat_us{0.0}, p99_lat_us{0.0};

        // // Health tracking
        std::uint64_t health_gap_frames{0};
        bool health_written_since_last_tick{false};

        // tick update
        void on_tick(double t_s, double r0, double y0, double u_post0) noexcept;

        // record a latency sample in ms
        void on_latency_us(double s) noexcept;

        // signal health record was written
        void on_health_written() noexcept;

        // final current tick
        void on_tick_commit() noexcept;

        // compute percentiles from reservoir
        void finalize_latency_percentiles() noexcept;

        // reset accum
        void reset() noexcept;
    };
} // namespace ictk::tools::detail