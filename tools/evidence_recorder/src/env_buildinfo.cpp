#include <vector>
#include <sstream>
#include <cstddef>

#include "env_buildinfo.hpp"
#include "ictk/version.hpp"
#include "ictk/core/types.hpp"

#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

namespace ictk::tools::detail{
    static const char* compiler_string(){
        #if defined(__clang__)
            return "clang " __clang_version__;
        #elif defined(__GNUC__)
            return "gcc " __VERSION__;
        #elif defined(_MSC_VER)
            #define ICTK_MSVC_TOSTR2(x) #x
            #define ICTK_MSVC_TOSTR(x) ICTK_MSVC_TOSTR2(x)
            static const char* s = "msvc " ICTK_MSVC_TOSTR(_MSC_VER);
            return s;
        #else
            return "unknown";
        #endif
    }

    static const char* scalar_string(){
        #if defined(ICTK_SCALAR_FLOAT)
            return "float";
        #else
            return "double";
        #endif
    }

    static std::string flags_string(){
        std::vector<const char*> f;

        #if defined(ICTK_NO_EXCEPTIONS)
            f.push_back("no-exceptions");
        #endif
        
        #if defined(ICTK_NO_RTTI)
            f.push_back("no-rtti");
        #endif

        #if defined(ICTK_ENABLE_LTO)
            f.push_back("lto");
        #endif

        #if defined(__FAST_MATH__)
            f.push_back("fast-math-ON");
        #else
            f.push_back("fast-math-OFF");
        #endif

        #ifdef NDEBUG
            f.push_back("release");
        #else
            f.push_back("debug");
        #endif

        std::ostringstream oss;

        for (size_t i=0; i<f.size(); ++i){
            if (i) oss << ' ';
            oss << f[i];
        }
        return oss.str();
    }

    BuildInfoPack make_buildinfo(std::uint64_t dt_ns, const char* controller_id, const char * asset_id, std::uint32_t tick_decimation) { 
        BuildInfoPack bi;
        bi.ictk_version = ictk::kVersionStr;
        bi.git_sha = GIT_SHA;
        bi.compiler = compiler_string();
        bi.flags = flags_string();
        bi.scalar_type = scalar_string();
        bi.dt_ns = dt_ns;
        bi.controller_id = controller_id ? controller_id : "";
        bi.asset_id = asset_id ? asset_id : "";
        bi.tick_decimation = tick_decimation;
        return bi;
    }
} // namespace ictk::tools::detail

