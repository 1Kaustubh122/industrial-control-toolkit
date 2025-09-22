#include <cmath>
#include <algorithm>

#include "kpi_calc.hpp"

namespace ictk::tools::detail{
    void KpiAcc::on_tick(double t_s, double r0, double y0, double u_post0) noexcept{
        const double e = std::abs(r0 - y0);
        iae += e;
        itae += t_s * e;

        if (have_u) tvu += std::abs(u_post0 - last_u);
        last_u = u_post0;
        have_u = true;

        health_written_since_last_tick = false;
    }

    void KpiAcc::on_latency_us(double s) noexcept{
        if (!std::isfinite(s) || s < 0.0) return;
        lat_us[lat_head] = s;
        lat_head = (lat_head + 1) % kLatCap;
        if (lat_count < kLatCap) ++lat_count;
    }

    void KpiAcc::on_health_written() noexcept{
        health_written_since_last_tick = true;
    }
    
    void KpiAcc::on_tick_commit() noexcept{
        if (!health_written_since_last_tick) ++health_gap_frames;
    }

    void KpiAcc::finalize_latency_percentiles() noexcept{
        // Guard
        if (lat_count == 0){
            p50_lat_us = p95_lat_us = p99_lat_us = 0.0;
            return;
        }

        // copy to local buffer and partial sort; probably O(n log n) worst case with cap = 2048
        double buf[kLatCap];
        const std::size_t n = lat_count;

        // Reconstruct linear order from ring 
        const std::size_t start = (lat_count == kLatCap) ? lat_head : 0;

        for (std::size_t i=0; i<n; ++i){
            const std::size_t idx = (start + i) % kLatCap;
            buf[i] = lat_us[idx];
        }

        std::sort(buf, buf + n);

        auto q = [&](double q)->double{
            const double pos = q * (n-1);
            const std::size_t lo = static_cast<std::size_t>(pos);
            const std::size_t hi = std::min(lo + 1, n-1);
            const double frac = pos - lo;
            return buf[lo] + (buf[hi] - buf[lo]) * frac;
        };

        p50_lat_us = q(0.50);
        p95_lat_us = q(0.95);
        p99_lat_us = q(0.99);
    }

    void KpiAcc::reset() noexcept{
        iae = itae = tvu = 0.0;
        last_u = 0.0;
        have_u = false;

        lat_count = 0;
        lat_head = 0;
        p50_lat_us = p95_lat_us = p99_lat_us = 0.0;

        health_gap_frames = 0;
        health_written_since_last_tick = false;
    }
} // namespace ictk::tools::detail
