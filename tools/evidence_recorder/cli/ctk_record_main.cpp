#include <ctime>
#include <chrono>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <cstring>
#include <filesystem>

#include "ictk/io/kpi.hpp"
#include "ictk/core/time.hpp"
#include "ictk/tools/recorder.hpp"

namespace fs = std::filesystem;
using namespace ictk;
using namespace ictk::tools;

// // Single stderr print flags and CSV 
static void usage(){
    std::fprintf(
        stderr,
        "ictk_record --out <dir> --schema-dir <dir> --tick-decim N "
        "--segment-max-mb 256 --fsync-policy {every_segment|every_n_mb} --fsync-n-mb 16 "
        "--dt-ns <n> --controller-id <str> --asset-id <str> "
        "--mode {primary|residual|shadow|cooperative} --stdin-csv\n"
        "CSV (if --stdin-csv): t_ns,y0,r0,u_pre0,u_post0\n"
    );
}

// system clock -> epoch ns
static inline long long now_utc_ns(){
    using namespace std::chrono;
    return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

// monotonic source for TimeAnchor
#if defined(_WIN32)
    #include <windows.h>
    static inline long long now_mono_ns(){
        LARGE_INTEGER f, c;
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&c);
        return (long long)((1e9 * (long double)c.QuadPart) / (long double)f.QuadPart);
    }
#else
    #include <time.h>
    static inline long long now_mono_ns(){
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (long long) ts.tv_sec*1000000000ll + (long long)ts.tv_nsec;
    }
#endif

int main(int argc, char** argv){
    // // Configs
    const char* out_dir = "evidence";
    const char* schema_dir = ICTK_FB_SCHEMA_DIR;
    int tick_decim = 1;
    int segment_mb = 256;
    const char* fsync_policy = "every_n_mb";
    int fsync_n_mb = 16;
    bool stdin_csv = false;
    long long dt_ns_hint = 0;
    const char* controller_id = "";
    const char* asset_id = "";
    const char* mode_str = "primary"; // default

    for (int i=1; i<argc; i++){
         if (!std::strcmp(argv[i], "--out") && i+1<argc) out_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--schema-dir") && i+1<argc) schema_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--tick-decim") && i+1<argc) tick_decim = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--segment-max-mb") && i+1<argc) segment_mb = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--fsync-policy") && i+1<argc) fsync_policy = argv[++i];
        else if (!std::strcmp(argv[i], "--fsync-n-mb") && i+1<argc) fsync_n_mb = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--dt-ns") && i+1<argc) dt_ns_hint = std::atoll(argv[++i]);
        else if (!std::strcmp(argv[i], "--controller-id") && i+1<argc) controller_id = argv[++i];
        else if (!std::strcmp(argv[i], "--asset-id") && i+1<argc) asset_id = argv[++i];
        else if (!std::strcmp(argv[i], "--mode") && i+1<argc) mode_str = argv[++i];
        else if (!std::strcmp(argv[i], "--stdin-csv")) stdin_csv = true;
        else{ 
            usage();
            return 2;
        }
    }

    // exceptions
    {
        std::error_code ec;
        fs::create_directories(out_dir, ec);
        if (ec) std::fprintf(stderr, "ictk_record: warn: create_directories(%s): %s\n",
                            out_dir, ec.message().c_str());
    }

    // assigns dirs, sizes with clamping, seg size, fsync policy, fsync N MiB, stdin mode off, dt hint, id and mode
    RecorderOptions opt;
    opt.out_dir = out_dir;
    opt.schema_dir = schema_dir;
    opt.segment_max_mb = (segment_mb>0) ? (std::size_t)segment_mb : 256;
    opt.fsync_n_mb = (fsync_n_mb > 0) ? (std::size_t)fsync_n_mb : 16;
    opt.tick_decimation = (tick_decim > 0) ? tick_decim : 1;
    opt.fsync_policy = (
        std::strcmp(fsync_policy, "every_segment") == 0 ?
        RecorderOptions::EverySegment : RecorderOptions::EveryNMB
    );
    opt.dt_ns_hint = dt_ns_hint;
    opt.controller_id = controller_id;
    opt.asset_id = asset_id;

    // Choose mode, default primary
    if      (!std::strcmp(mode_str, "primary"))     opt.fixed_mode = ictk::kPrimary;
    else if (!std::strcmp(mode_str, "residual"))    opt.fixed_mode = ictk::kResidual;
    else if (!std::strcmp(mode_str, "shadow"))      opt.fixed_mode = ictk::kShadow;
    else if (!std::strcmp(mode_str, "cooperative")) opt.fixed_mode = ictk::kCooperative;

    auto rec = Recorder::open(opt);
    rec->write_buildinfo();

    // compute anchor
    const long long epoch_mono_ns = now_mono_ns();
    const long long epoch_utc_ns = now_utc_ns();

    // write time anchor
    rec->write_time_anchor(epoch_mono_ns, epoch_utc_ns);

    KpiCounters kpi{};
    if (stdin_csv){
        char line[256];
        while (std::fgets(line, sizeof(line), stdin)){
            line[strcspn(line, "\r\n")] = 0;

            long long t_ns=0;
            double y0=0, r0=0, upre=0, upost=0;
            const int n = std::sscanf(
                line, " %lld , %lf , %lf , %lf , %lf",
                &t_ns, &y0, &r0, &upre, &upost
            );
            if (n != 5) continue;

            TickSample s;
            s.t = t_ns;
            s.y0 = y0;
            s.r0 = r0;
            s.u_pre0 = upre;
            s.u_post0 = upost;

            s.h.deadline_miss_count = 0;
            s.h.saturation_pct = 0.0;
            s.h.rate_limit_hits = 0;
            s.h.jerk_limit_hits = 0;
            s.h.fallback_active = false;
            s.h.novelty_flag = false;
            s.h.aw_term_mag = 0.0;
            s.h.last_clamp_mag = 0.0;
            s.h.last_rate_clip_mag = 0.0;
            s.h.last_jerk_clip_mag = 0.0;

            rec->write_tick(s);
            ++kpi.updates;
            rec->rotate_if_needed();
        }
    }

    rec->write_kpi(kpi);
    rec->flush();
    return 0;
}