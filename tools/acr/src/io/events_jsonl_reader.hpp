#pragma once

#include <string>
#include <vector> 
#include <utility>
#include <cstdint>
#include <filesystem>

namespace ictk::tools::acr::events{
    /// @brief JSONL events
    struct Event{
        // timestamp of the event in ns
        std::int64_t t_ns{0};

        // Sequence number (monotonic counter across the file)
        std::uint64_t seq{0};

        // line of JSONL file this event came from
        std::uint64_t line_no{0};

        // Estop activation
        bool estop_rise{false};
    };

    /// @brief summary for the event 
    struct Probe{
        // whehter the file exists
        bool present{false};

        // total no of lines read (total events)
        std::uint64_t lines_total{0};

        // lines couldn't be parsed
        std::uint64_t lines_bad{0};

        // count of estop activation
        std::uint64_t estop_true{0};
    };

    /// @brief Read all estop event line from JSONL file
    /// @param p Path to JSONL file
    /// @param probe Summary structure filled with counts
    /// @return vector of events entries
    [[nodiscard]] std::vector<Event> read_events(const std::filesystem::path& p, Probe& probe);

    /// @brief change in transitions are logged as zone change event
    struct ZoneChange{
        // time of zone changed
        std::int64_t t_ns{0};

        // sequence number
        std::uint64_t seq{0};

        // JSONL file line number
        std::uint64_t line_no{0};

        // which zone entered
        int     zone_id{0};

        // max safety param for that zone
        double  v_safe{0.0};
    };

    /// @brief Read zone change events from a JSONL file
    /// @param p path to JSONL file
    /// @return vector of of ZoneChnage entries -> logs of zone transition
    [[nodiscard]] std::vector<ZoneChange> read_zone_changes(const std::filesystem::path& p);

} // namespace ictk::tools::acr::events