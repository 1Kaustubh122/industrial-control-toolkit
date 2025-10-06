#pragma once
#include <cstdint>

namespace ictk::tools::acr{
    // // explicit type: enum size is stable across ABIs

    // no implicit int conversions
    enum class ExitCode: std::int32_t{
        // success
        kOk = 0,

        // Could not open MCAP/sidecar/events file
        kOpenFail = 1,

        // Sidecar requested but not found
        kSidecarMissing = 2,

        // BLAKE3 (or schema) mismatch
        kHashMismatch = 3,

        // topic -> FlatBuffers root mismatch
        kBadSchema = 4,

        // MCAP parse error ot truncated stram
        kStreamCorrupt = 5,

        // Required channels/fields absent
        kMissingRequired =6,

        // Out of memory 
        kOOM = 7,

        // BuildInfo fields disagree across segments 
        kBuildInfoConflict = 16
    };

    // // helper functions

    // safe cast for returning from main
    [[nodiscard]] inline constexpr int to_int(ExitCode e) noexcept{
        return static_cast<int>(e);
    }

    // Canonical success check
    [[nodiscard]] inline constexpr bool is_ok(ExitCode e) noexcept{
        return e == ExitCode::kOk;
    }
} // namespace ictk::tools::acr
