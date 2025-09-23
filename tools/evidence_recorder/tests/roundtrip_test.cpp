#include <cstdio>
#include <string>
#include <vector>
#include <cassert>
#include <cstdlib>
#include <filesystem>

#include "ictk/tools/recorder.hpp"

namespace fs = std::filesystem;
using namespace ictk::tools;

static std::string readall(const fs::path& p){
    const auto sz = static_cast<std::size_t>(fs::file_size(p));
    std::FILE* f = std::fopen(p.string().c_str(), "rb");
    assert(f);
    std::string s(sz, '\0');
    std::fread(s.data(), 1, sz, f);
    std::fclose(f);
    return s;
}


int main(){
    const char* out_dir = "evidence_test";
    fs::remove_all(out_dir);
    fs::create_directories(out_dir);

    RecorderOptions opt;

    opt.out_dir = out_dir; 
    opt.dt_ns_hint = 1000000;

    auto rec = Recorder::open(opt);
    rec->write_buildinfo();
    rec->write_time_anchor(123456789, 987654321);

    TickSample s{};

    s.t = 1000; 
    s.y0 = 1.0; 
    s.r0 = 1.5; 
    s.u_pre0 = 0.2; 
    s.u_post0 = 0.18;

    rec->write_tick(s);
    rec->flush();

    fs::path latest;
    fs::file_time_type newest{};

    for (auto& e : fs::directory_iterator(out_dir)){
        if (!e.is_regular_file()) continue;
        auto t = fs::last_write_time(e);
        if (latest.empty() || t > newest) {newest = t; latest = e.path();}
    }
    assert(!latest.empty());

    auto a = readall(latest);
    fs::path copy = latest.string() + ".copy";
    std::FILE* g = std::fopen(copy.string().c_str(), "wb");
    assert(g);
    std::fwrite(a.data(), 1, a.size(), g);
    std::fclose(g);
    auto b = readall(copy);
    assert(a == b);
    return 0;
}
