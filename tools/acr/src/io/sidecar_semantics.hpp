#pragma once

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <filesystem>
#include <string_view>

namespace ictk::tools::acr::sidecar{
    // both of size 32 bytes => 64 hex chars
    inline constexpr std::size_t kBlake3_256_HexLen = 64;
    inline constexpr std::size_t kSha256_HexLen     = 64;

    struct SidecarData{
        // blake3 or sha256
        std::string payload_alg;

        // 64 hex len
        std::string payload_hex;

        // FlatBuffers schema sha256 len = 64
        std::vector<std::string> bfbs_sha256;
    }; // SidecarData

    // return SidecarData on sucess else null  
    [[nodiscard]] std::optional<SidecarData> parse_sidecar(const std::filesystem::path& p);

    // // Utilities
    [[nodiscard]] bool is_hex_64(std::string_view s);
    [[nodiscard]] bool ieq_hex(std::string_view a, std::string_view b);

} // namespace ictk::tools::acr::sidecar
