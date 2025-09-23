#include <filesystem>
#include <string>
#include <vector>
#include <cassert>
#include <cstdio>
#include "ictk/tools/recorder.hpp"

namespace fs = std::filesystem;
using namespace ictk::tools;

static std::string readall(const fs::path& p){
    std::FILE* f = std::fopen(p.string().c_str(), "rb");
    assert(f);
    const auto sz = static_cast<std::size_t>(fs::file_size(p));
    std::string s(sz, '\0');
    const std::size_t n = std::fread(s.data(), 1, sz, f);
    std::fclose(f);
    if (n != sz) s.resize(n);
    assert(n == sz);
    return s;
}


int main(){
    const char* out_dir = "evidence_tb";
    fs::remove_all(out_dir);
    fs::create_directories(out_dir);

    RecorderOptions opt; 
    opt.out_dir = out_dir; 
    opt.dt_ns_hint = 1000000;

    auto rec = Recorder::open(opt);

    rec->write_buildinfo();
    rec->write_time_anchor(111, 222);
    rec->flush();

    fs::path latest;
    fs::file_time_type newest{};
    for (auto& e : fs::directory_iterator(out_dir)){
        if (!e.is_regular_file()) continue;
        auto t = fs::last_write_time(e);
        if (latest.empty() || t > newest) { newest = t; latest = e.path(); }
    }

    assert(!latest.empty());
    std::string s = readall(latest);

    assert(s.find("\"clock_domain\":\"MONO\"") != std::string::npos);
    assert(s.find("\"epoch_mono_ns\":111") != std::string::npos);
    assert(s.find("\"epoch_utc_ns\":222") != std::string::npos);

    return 0;
}
