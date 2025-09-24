#pragma once

#include <span>
#include <array>
#include <string>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ictk::tools::hash{

    //API for BLAKE3 -> 32 byte digest -> data, len for memory, *_file for streaming a file
    // // BLAKE3-256
    std::array<std::uint8_t, 32> blake3_256(const void* data, size_t len);
    std::array<std::uint8_t, 32> blake3_256_file(const char * path);

    // Self contained SHA-256 
    // // SHA-256 (portable, for BFBS digest)
    std::array<std::uint8_t, 32> sha256(const void* data, size_t len);
    std::array<std::uint8_t, 32> sha256_file(const char* path);

    // // Hex Helpers
    std::string to_hex(std::span<const std::uint8_t> bytes);
} // namespace ictk::tools::hash
