#pragma once

#include <string>       // for ids
#include <vector>       // for list of MCAP paths
#include <utility>      // for time range start end
#include <cstddef>      // memory safe unsigned counter
#include <cstdint>      // fixed width integers -> Stable ABI
#include <optional>     // for values might be absent (no event file/no filter)
#include <filesystem>   

namespace ictk::tools::acr{
    /// @brief tells ACR how to treat the .sidecar.json files that sit beside each MCAP log
    enum class SidecarPolicy : uint8_t{
        // only read sidecars -> normal audit
        kReadonly = 0,

        // generate sidecar JSONs for MCAPs that don't have them -> for first time data ingestion
        kCreate = 1,

        // overwrite existing sidecars with new hashes/schema -> to maintain or re index mode
        kUpdate = 2
    };

    /// @brief Configuration options: settings -> tells how ACR reads and checks data
    struct IngestConfig{
        // // Inputs
        // list of MCAP log files to ingest
        std::vector<std::filesystem::path> mcap_paths;

        /// (optional -> can run without these)

        // directory containing all the sidecars 
        std::optional<std::filesystem::path> sidecar_dir;

        // path to events log 
        std::optional<std::filesystem::path> events_path;

        // // filters : restrict which data to ingests or how strictly it checks

        // enforce each BuildInfo message in the MCAP
        bool require_tick_decim_1{true};

        // clause
        std::optional<std::string> asset_id_filter;
        std::optional<std::string> controller_id_filter;

        // time window in nano seconds
        std::optional<std::pair<std::int64_t, std::int64_t>> time_range_ns;

        // //  Integrity / schema behavior : while validating
        // check that each topic in MCAP matched the expected FlatBuffers
        bool strict_schema{true};

        // if multi seg claims differet id or timing, ACR aborts with exitcode
        bool fail_on_buildinfo_conflict{true};

        // blake3 for each MCAP 
        bool per_file_hash_verify{true};

        SidecarPolicy sidecar_policy{SidecarPolicy::kReadonly};

        // // streaming
        // soft cap on how many rows to keep in memory
        std::size_t max_rows_hint{0};

        // size of memory I/O (8MB)
        std::size_t stream_buffer_bytes{8u * 1024u * 1024u};

        // reserve space for future
        std::uint32_t _reserved0{0};
    };
    
} // namespace ictk::tools::acr
