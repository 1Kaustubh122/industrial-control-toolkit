#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <optional>

#include "ictk/tools/acr/types.hpp"

namespace ictk::tools::acr{
    /// @brief per file summary (Per MCAP segment)
    struct FileEntry{
        // file to ingest
        std::string     path;

        // file size
        std::uint64_t   size_bytes{0};

        // all records in that file
        std::uint64_t   message_total{0};

        // per topic counts
        std::uint64_t   tick{0};
        std::uint64_t   health{0};
        std::uint64_t   kpi{0};

        // time span covered by this file
        std::int64_t    first_t_ns{0};
        std::int64_t    last_t_ns{0};

        // hex BLAKE3 of the file payload 
        std::string     payload_blake3;

        // null if no sidecar -> true/false if verified
        std::optional<bool> payload_hash_ok;

        // FB roots matched
        bool schema_ok{false};
    }; // FileEntry


    /// @brief total after stiching all files
    struct MergedEntry{
        // merged counts
        std::uint64_t tick{0};
        std::uint64_t health{0};
        std::uint64_t kpi{0};

        // global span after merge
        std::int64_t first_t_ns{0};
        std::int64_t last_t_ns{0};
    }; // MergedEntry


    /// @brief Quality counter
    struct Anomalies{
        // time went backward
        std::uint64_t non_monotonic_ticks{0};

        // duplicate timestamps across segments
        std::uint64_t overlap_msgs{0};

        // out of order across the total order
        std::uint64_t ooo_msgs{0};

        // total missing time
        std::uint64_t gaps_ns{0};

        // mixed MONO/WALL
        bool timebase_mixed{false};
    }; // Anomalies


    /// @brief Reason for Anomalies
    enum class GapReason : std::uint8_t{
        Duplicate = 1,
        Backward = 2,
        Missing = 3,
        Unknown = 9
    }; // GapReason

    /// @brief Reason for Anomalies
    struct GapSpan{
        std::int64_t start_t_ns{0};
        std::int64_t end_t_ns{0};
        std::uint64_t missing_ticks{0};
        GapReason reason{GapReason::Unknown};
    }; // GapSpan


    /// @brief build block
    struct BuildInfoBlock{
        // config cycle and downsampling
        std::int64_t    dt_ns{0};
        std::uint32_t   tick_decimation{1};

        // identity
        std::string     controller_id;
        std::string     asset_id;

        // clock domain: MONO/WALL
        std::string     clock_domain{"MONO"};

        // kernel clock
        std::string     kernel_clocksource{"unknown"};

        // source
        std::string     dt_source{"buildinfo"};

        // timing stability
        std::int64_t    dt_p50_est_ns{0};
        double          dt_p95_over_p50{0.0};
    };
    

    /// @brief version attestation for tools and schema
    struct Manifest{
        std::string acr_version;
        std::string ictk_version;
        std::string git_sha;
        std::string shcema_id;
    };
    
    
    /// @brief detailed report
    struct Report{
        Manifest                manifest{};
        std::vector<FileEntry>  files;
        MergedEntry             merged{};
        Anomalies               anomalies{};
        BuildInfoBlock          buildinfo{};
        bool                    unstable_dt{false};

        struct Conflict{
            std::string field;
            std::vector<std::string> values;
        };

        std::vector<Conflict> buildinfo_conflicts;

        struct RequiredPresent{
            bool speed{false};
            bool u_pre{false};
            bool u_cmd{false};
            bool set_point{false};
        } required_fields_present;

        // list of absent items
        std::vector<std::string> missing_fields;

        // topic roots and schema hash 
        struct SchemaBlock{
            std::string tick_root{"ictk.metrics.Tick"};
            std::string health_root{"ictk.metrics.Health"};
            std::string kpi_root{"ictk.metrics.Kpi"};
            std::vector<std::string> bfbs_sidecar_sha256;
            std::vector<std::string> bfbs_mcap_snapshot;
            std::vector<std::string> bfbs_mismatch;
        } schema;

        std::string                 notes{"stream ingest"};
        std::vector<std::string>    warnings;
        std::uint64_t               monotonic_nudges{0};
        std::vector<GapSpan>        gaps;

        EventsProbe events_probe{};
    };
} // namespace ictk::tools::acr