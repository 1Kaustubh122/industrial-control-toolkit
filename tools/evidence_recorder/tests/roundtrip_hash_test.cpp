#include <cstdio>
#include <string>
#include <cassert>
#include <filesystem>

#include "hash.hpp" 
#include "ictk/tools/recorder.hpp"

int main(){
    #if ICTK_RECORDER_BACKEND_MCAP
        return 0;
    #else

    namespace fs = std::filesystem;

    fs::create_directories("evidence_rt");

    ictk::tools::RecorderOptions opt;
    opt.out_dir="evidence_rt";
    opt.dt_ns_hint=1000000;

    auto rec = ictk::tools::Recorder::open(opt);
    rec->write_time_anchor(111,222);
    rec->write_buildinfo();

    for(int i=0;i<10;i++){
        ictk::tools::TickSample s{};
        s.t = 1000 + i;
        rec->write_tick(s);
    }

    rec->write_kpi({}); rec->flush();

    fs::path mcap, sidecar;
    for (auto& e: fs::directory_iterator("evidence_rt")){
        if (e.path().extension()==".mcap") mcap=e.path();
        if (e.path().extension()==".json") sidecar=e.path();
    }

    assert(!mcap.empty() && !sidecar.empty());

    auto bl = ictk::tools::hash::blake3_256_file(mcap.string().c_str());
    auto bl_hex = ictk::tools::hash::to_hex({bl.data(), bl.size()});

    std::FILE* f = std::fopen(sidecar.string().c_str(),"rb");
    assert(f);

    std::string s;
    char buf[4096];
    size_t n;
    while((n=fread(buf,1,sizeof(buf),f))>0) s.append(buf,n);

    std::fclose(f);
    assert(s.find(bl_hex)!=std::string::npos);
    return 0;
    #endif
}
