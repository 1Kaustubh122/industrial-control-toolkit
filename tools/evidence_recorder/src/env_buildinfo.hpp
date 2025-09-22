#pragma once

#include <cstdint>
#include <string>

namespace ictk::tools::detail{

    struct BuildInfoPack{
        std::string ictk_version;
        std::string git_sha;
        std::string compiler;
        std::string flags;
        std::string scalar_type;
        std::uint64_t dt_ns{0};
        std::string controller_id;
        std::string asset_id;
        std::uint64_t tick_decimation{0};
    };

    BuildInfoPack make_buildinfo(std::uint64_t dt_ns, const char* controller_id, const char* asset_id, std::uint32_t tick_decimation);
    
} // namespace ictk::tools::detail
