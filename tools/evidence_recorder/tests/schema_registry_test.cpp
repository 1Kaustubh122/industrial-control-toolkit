#include <filesystem>
#include <cassert>
#include "ictk/tools/recorder.hpp"

int main(){
    #if !ICTK_RECORDER_BACKEND_MCAP
        return 0;
    #else

    namespace fs = std::filesystem;

    fs::create_directories("evidence_schema");

    ictk::tools::RecorderOptions opt;
    opt.out_dir="evidence_schema";
    opt.dt_ns_hint=1000000;

    auto rec = ictk::tools::Recorder::open(opt);
    rec->write_time_anchor(1,2);
    rec->write_buildinfo();

    ictk::tools::TickSample s{};

    s.t=10;
    rec->write_tick(s);
    rec->write_kpi({});
    rec->flush();

    fs::path latest;

    for (auto& e: fs::directory_iterator("evidence_schema"))

    if (e.path().extension()==".mcap") latest=e.path();
    assert(!latest.empty());
    
    return 0;
#endif
}