#pragma once

#include <memory>
#include <cstdint>

#include "ictk/io/kpi.hpp"
#include "ictk/core/time.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/health.hpp"

#ifndef ICTK_FB_SCHEMA_DIR
#define ICTK_FB_SCHEMA_DIR "tools/evidence_recorder/schemas"
#endif

namespace ictk::tools{
    struct TickSample{
        /*
        t = ns timestamp (monotonic) -> cast to uint64_t
        y0 = measured output
        r0 = reference
        u_pre0 = controller output before clamp
        u_post0 = after clamp
        */
        t_ns t{};
        double y0{};
        double r0{};
        double u_pre0{};
        double u_post0{};
        ControllerHealth h{};
    };
    
    struct RecorderOptions{
        const char* out_dir{"evidence"}; // output directory for logs
        const char* schema_dir{ICTK_FB_SCHEMA_DIR}; // FlatBuffer schema location
        std::size_t segment_max_mb{256};    // rotation size threshold
        std::size_t fsync_n_mb{16}; // between fsync calls
        int tick_decimation{1}; // every Nth tick (load reduction)
        enum FsyncPolicy{EverySegment, EveryNMB} fsync_policy{EveryNMB}; // choose fsync on segment boundary or rolling
        long long dt_ns_hint{0};    // loop period hint
        const char* controller_id{""}; // eg: ictk_pid_v1 or ictk_mpc_v2
        const char* asset_id{""};   // id of assets
        ictk::CommandMode fixed_mode{ictk::kPrimary};
    };
    
    class Recorder{
        public:
            [[nodiscard]] static std::unique_ptr<Recorder> open(const RecorderOptions& opt);

            virtual ~Recorder() = default;

            // logs compiler/git/version metadata
            virtual void write_buildinfo() = 0;

            // maps monotonic clock to UTC for audit correlation
            virtual void write_time_anchor(std::int64_t epoch_mono_ns,
                                           std::int64_t epoch_utc_ns) = 0;

            // core per cycle evidence 
            virtual void write_tick(const TickSample& s) = 0;

            // dumps aggregated KPI counters
            virtual void write_kpi(const ictk::KpiCounters& kpi) = 0;

            // roll output files based on size/limits
            virtual void rotate_if_needed() = 0;

            // force fsync
            virtual void flush() = 0;
    };
} // namespace ictk::tools