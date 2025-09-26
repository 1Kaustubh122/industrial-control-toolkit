#include <atomic>
#include <vector>
#include <chrono>
#include <cstdio>
#include <string>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string_view>

#include "kpi_calc.hpp"
#include "ictk/io/kpi.hpp"
#include "env_buildinfo.hpp"
#include "ictk/core/health.hpp"
#include "ictk/tools/recorder.hpp"

#if defined(ICTK_RECORDER_BACKEND_MCAP)
#include <mcap/writer.hpp>
#include <flatbuffers/flatbuffers.h>

#include "hash.hpp"
#include "ictk_metrics_generated.h"
#endif

#if defined(_WIN32)
  #include <io.h>
  #include <windows.h>
#else
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace ictk::tools{
    // #if defined(ICTK_RECORDER_BACKEND_MCAP)
        using namespace std::chrono;

        static inline std::string utc_ns_string(){
            auto now = std::chrono::system_clock::now();
            auto ns = duration_cast<nanoseconds>(now.time_since_epoch()).count();
            return std::to_string((long long)ns);
        }

        static inline std::string kernel_clocksource(){
            #if defined(__linux__)
                const char* p="/sys/devices/system/clocksource/clocksource0/current_clocksource";
                std::FILE* f = std::fopen(p, "rb");
                if (!f) return "unknown";
                
                char buf[128];
                size_t n = feed(buf, 1, sizeof(buf) - 1, f);
                buf[n] = 0;
                std::fclose(f);

                for (size_t i=0; i<n; i++){
                    if (buf[i] == '\n' || buf[i] == '\r'){
                        buf[i] = 0;
                        break;
                    }
                }
                return std::string(buf);
            
            #elif defined(_WIN32)
                return "QPC";
            #elif defined(__APPLE__)
                return "mach_absolute_time";
            #else
                return "unknown";
            #endif
        }

        static inline std::vector<uint8_t> read_file_bin(const std::string &p){
            std::FILE* f = std::fopen(p.c_str(), "rb");
            if (!f) return {};
            std::vector<uint8_t> b;
            b.reserve(4096);
            uint8_t buf[4096];
            size_t n;

            while ((n=fread(buf, 1, sizeof(buf), f)) > 0) b.insert(b.end(), buf, buf + n);

            std::fclose(f);
            return b;
        }

        class RecorderMcap final : public Recorder{
            public:
                explicit RecorderMcap(const RecorderOptions& opt) : cfg_{}, builder_(1024){
                    
                }
            
            private:
                struct RecorderConfig{
                    std::string out_dir;
                    std::string schema_dir;
                    std::size_t segment_max_mb{256};
                    std::size_t fsync_n_mb{16};
                    bool fsync_every_segment{false};
                    int tick_decimation{1};
                    long long dt_ns_hint{0};
                };

                static std::string make_filename_(const std::string& dir, const char* ext){
                    static std::atomic<uint64_t> seq{0};
                    const std::string utcns = utc_ns_string();
                    const uint64_t s = seq.fetch_add(1, std::memory_order_relaxed);
                    return (fs::path(dir) / ("ictk_" + std::string(GIT_SHA) + "_" + utcns + "_" + std::to_string(s) + ext)).string();
                }
            private:
                RecorderConfig cfg_{}
        };
    
    // #endif

} // namespace ictk::tool
