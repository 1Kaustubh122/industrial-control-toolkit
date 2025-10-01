#include <chrono>
#include "ictk/tools/recorder.hpp"

int main(){
    #if ICTK_RECORDER_BACKEND_MCAP
        return 0;
    #else

    using namespace std::chrono;

    ictk::tools::RecorderOptions opt;
    opt.out_dir="evidence_tp";
    opt.segment_max_mb=32;
    opt.dt_ns_hint=1'000'000;

    auto rec = ictk::tools::Recorder::open(opt);

    const auto t0m = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    const auto t0u = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

    rec->write_time_anchor(t0m, t0u);
    rec->write_buildinfo();

    const int hz=1000, secs=5;
    const long long dt = 1'000'000'000ll/hz;

    long long t = t0m;

    for(int i=0;i<hz*secs;i++){
        ictk::tools::TickSample s{}; s.t=t;
        rec->write_tick(s);
        t+=dt;
        rec->rotate_if_needed();
    }

    rec->write_kpi({});
    rec->flush();
    return 0;
    
    #endif
}
