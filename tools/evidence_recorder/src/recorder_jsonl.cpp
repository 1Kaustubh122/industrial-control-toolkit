#include <atomic>   // for thread safe counters for filenames
#include <cstdio>   // FILE*, for open, write and flush work
#include <ctime>    // for conversion of time format
#include <vector>   // dynamic arrays (NO RT HERE)
#include <string>
#include <chrono>   // for time util
#include <cstdlib>  // for size_t
#include <cstring>  // for string ops
#include <filesystem>   // for file handling 
#include <string_view>  // for now owning string slice

#include "ictk/io/kpi.hpp"          // for KpiCounters struct
#include "ictk/version.hpp"         // Version strings
#include "ictk/core/time.hpp"       // time type
#include "ictk/core/health.hpp"     // controller health fields
#include "ictk/tools/recorder.hpp"  // Recorder interface and options

#include "kpi_calc.hpp"         // for KPI accum
#include "env_buildinfo.hpp"    // for BuilInfoPack

// // for windows
#if defined(_WIN32)
    #include <io.h>     // _commit
#else
    #include <fcntl.h>
    #include <unistd.h>     // fsync
    #include <sys/stat.h>
    #include <sys/types.h>
#endif

// Default value
#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

/*
goal: JSONL backend for Recorder
Writes BuildInfo, per-tick Tick + Health and KPI
Rotates files bu size
Periodically fsync.
tracks per segment seq
*/

namespace fs = std::filesystem;
namespace ictk::tools{
    #if ICTK_RECORDER_BACKEND_MCAP
        std::unique_ptr<Recorder> make_mcap_recorder(const RecorderOptions& opt);
    #endif
    // // Helper functions

    /*
    produce YYYYMMDD_HHMMSS in UTC format
    for filenaming -> time stamped segments
    */
    static inline std::string utc_timestamp_filename(){
        /*
        TO DO:
        If system clock is wrong then
        wrong filenames still sort lexically by whatever the clock says.
        Evidence still writes
        */
        using namespace std::chrono;
        auto now = system_clock::now();                 // wall clock now
        std::time_t t = system_clock::to_time_t(now);   // to time_t (calender time)
        std::tm tm{};   // calender time
        #if defined(_WIN32)
            gmtime_s(&tm, &t);  // thread-safe UTC convert
        #else
            gmtime_r(&t, &tm);  // POSIX thread safe UTC convert
        #endif
        char buf[32];
    
        std::snprintf(
            buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec
        );

        return std::string(buf);
    }

    static inline std::string jesc(std::string_view s){
        /*
        To DO:
        Upgrade it to handle more escape control like \n, \r etc
        */
        std::string o;
        o.reserve(s.size() + 8);
        for (char c : s){
            if (c == '"' || c == '\\') o.push_back('\\');       // escape control 
            o.push_back(c);
        }
        return o;
    }

    // // JSONL backend
    class RecorderJsonl final : public Recorder{
        public:
            // ctor -> initialize as internal config POD
            explicit RecorderJsonl(const RecorderOptions& opt) : cfg_{}{

                // // NULL GUARD C string
                cfg_.out_dir = opt.out_dir ? opt.out_dir : "evidence";
                cfg_.schema_dir = opt.schema_dir ? opt.schema_dir : ICTK_FB_SCHEMA_DIR;

                cfg_.segment_max_mb = opt.segment_max_mb;
                cfg_.fsync_policy = (opt.fsync_policy == RecorderOptions::EverySegment) ? FsyncPolicy::EverySegment : FsyncPolicy::EveryNMB;
                cfg_.fsync_n_mb = opt.fsync_n_mb;
                cfg_.tick_decimation = opt.tick_decimation;

                // identity
                cfg_.controller_id = opt.controller_id ? opt.controller_id : "";
                cfg_.asset_id = opt.asset_id ? opt.asset_id : "";
                cfg_.fixed_mode = opt.fixed_mode;

                // loop period
                dt_ns_hint_ = opt.dt_ns_hint;
                std::error_code ec;
                fs::create_directories(cfg_.out_dir, ec);
            }

            // // Flushes and Fsyncs then close the file
            ~RecorderJsonl() override{
                close_current_();
            }

            /*
            Open the segment
            pack BuildIndo with dt, IDs, decimation.
            if dt_ns <= 0 then 0 signal 
            */
            void write_buildinfo() override{
                ensure_open_();
                using detail::make_buildinfo;
                auto bi = make_buildinfo(
                    static_cast<std::uint64_t>(dt_ns_hint_ > 0 ? dt_ns_hint_ : 0),
                    cfg_.controller_id.c_str(),
                    cfg_.asset_id.c_str(),
                    static_cast<std::uint32_t>(cfg_.tick_decimation)
                );

                std::string line;
                line.reserve(512);

                // // JSONL record with jesc escapes quote
                line += R"({"ch":"/ictk/buildinfo", "body":{)";
                line += R"("ictk_version":")"   + jesc(bi.ictk_version)       + R"(",)";
                line += R"("git_sha":")"        + jesc(bi.git_sha)            + R"(",)";
                line += R"("compiler":")"       + jesc(bi.compiler)           + R"(",)";
                line += R"("flags":")"          + jesc(bi.flags)              + R"(",)";
                line += R"("scalar_type":")"    + jesc(bi.scalar_type)        + R"(",)";
                line += R"("dt_ns":)"           + std::to_string(bi.dt_ns)    + ",";
                line += R"("controller_id":")"  + jesc(bi.controller_id)      + R"(",)";
                line += R"("asset_id":")"       + jesc(bi.asset_id)           + R"(",)";
                line += R"("tick_decimation":)" + std::to_string(static_cast<unsigned>(bi.tick_decimation));
                line += R"(}})";

                write_line_(line);
            }

            // // Cache anchors for potential downstream use
            void write_time_anchor(std::int64_t epoch_mono_ns, std::int64_t epoch_utc_ns) override{
                ensure_open_();
                mono_anchor_ns_ = epoch_mono_ns;
                utc_anchor_ns_  = epoch_utc_ns;
                std::string line;
                line.reserve(192);
                line += R"({"ch":"/ictk/time_anchor","body":{"clock_domain":"MONO","epoch_mono_ns":)";
                line += std::to_string(static_cast<unsigned long long>(epoch_mono_ns));
                line += R"(,"epoch_utc_ns":)";
                line += std::to_string(static_cast<unsigned long long>(epoch_utc_ns));
                line += R"(}})";
                write_line_(line);
            }

            void write_tick(const TickSample& s) override{
                // open -> apply decimation -> skips N-1 ticks by modulo counter
                ensure_open_();
                if (decim_skip_(s.t)) return;

                // define t=0 as first tick
                if (first_t_ < 0) first_t_ = s.t;
                // conv ns to sec
                const double t_s = static_cast<double>(s.t - first_t_) * 1e-9;

                // infer loop period once from first positive delta
                if (dt_ns_hint_ == 0 && prev_t_ >= 0){
                    const long long d = static_cast<long long>(s.t - prev_t_);
                    if (d > 0) dt_ns_hint_ = d;
                }
                prev_t_ = s.t;

                // update KPI accum
                acc_.on_tick(t_s, s.r0, s.y0, s.u_post0);

                std::string tick;
                tick.reserve(192);

                // Write tick record
                tick += R"({"ch":"/ictk/tick","body":{"seq":)";
                tick += std::to_string(static_cast<unsigned long long>(++seq_));
                tick += R"(,"t_ns":)";
                tick += std::to_string(static_cast<unsigned long long>(s.t));
                tick += R"(,"y0":)";         tick += to_fix_(s.y0);
                tick += R"(,"r0":)";         tick += to_fix_(s.r0);
                tick += R"(,"u_pre0":)";     tick += to_fix_(s.u_pre0);
                tick += R"(,"u_post0":)";    tick += to_fix_(s.u_post0);
                tick += R"(}})";
                write_line_(tick);

                std::string h;
                h.reserve(320);
                h += R"({"ch":"/ictk/health","body":{)";
                h += R"("deadline_miss_count":)" + std::to_string(static_cast<unsigned long long>(s.h.deadline_miss_count)) + ",";
                h += R"("saturation_pct":)"     + to_fix_(s.h.saturation_pct) + ",";
                h += R"("rate_hits":)"          + std::to_string(static_cast<unsigned long long>(s.h.rate_limit_hits)) + ",";
                h += R"("jerk_hits":)"          + std::to_string(static_cast<unsigned long long>(s.h.jerk_limit_hits)) + ",";

                h += R"("fallback_active":)";
                h += (s.h.fallback_active ? "true" : "false");
                h += ",";

                h += R"("novelty_flag":)";
                h += (s.h.novelty_flag ? "true" : "false");
                h += ",";

                h += R"("aw_term_mag":)"        + to_fix_(s.h.aw_term_mag) + ",";
                h += R"("last_clamp_mag":)"     + to_fix_(s.h.last_clamp_mag) + ",";
                h += R"("last_rate_clip_mag":)" + to_fix_(s.h.last_rate_clip_mag) + ",";
                h += R"("last_jerk_clip_mag":)" + to_fix_(s.h.last_jerk_clip_mag) + ",";
                h += R"("mode":)";
                switch (cfg_.fixed_mode)
                {
                case ictk::CommandMode::Primary:
                    h += "0";
                    break;
                
                case ictk::CommandMode::Residual:
                    h += "1";
                    break;
                case ictk::CommandMode::Shadow:
                    h += "2";
                    break;
                case ictk::CommandMode::Cooperative:
                    h += "3";
                    break;
                default:
                    h += "0";
                    break;
                }
                h+= R"(}})";
                write_line_(h);

                // health mark true and increment 
                acc_.on_health_written();
                acc_.on_tick_commit();
            }

            void write_kpi(const ictk::KpiCounters& k) override{
                ensure_open_();

                // // Compute latency 
                acc_.finalize_latency_percentiles();

                std::string line;
                line.reserve(320);

                // // Update KPIs
                line += R"({"ch":"/ictk/kpi_report","body":{)";
                line += R"("updates":)"           + std::to_string(static_cast<unsigned long long>(k.updates)) + ",";
                line += R"("watchdog_trips":)"    + std::to_string(static_cast<unsigned long long>(k.watchdog_trips)) + ",";
                line += R"("fallback_entries":)"  + std::to_string(static_cast<unsigned long long>(k.fallback_entries)) + ",";
                line += R"("limit_hits":)"        + std::to_string(static_cast<unsigned long long>(k.limit_hits)) + ",";
                line += R"("iae":)"               + to_fix_(acc_.iae) + ",";
                line += R"("itae":)"              + to_fix_(acc_.itae) + ",";
                line += R"("tvu":)"               + to_fix_(acc_.tvu) + ",";
                line += R"("p50_lat_us":)"        + to_fix_(acc_.p50_lat_us) + ",";
                line += R"("p95_lat_us":)"        + to_fix_(acc_.p95_lat_us) + ",";
                line += R"("p99_lat_us":)"        + to_fix_(acc_.p99_lat_us) + ",";
                line += R"("health_gap_frames":)" + std::to_string(static_cast<unsigned long long>(acc_.health_gap_frames));
                line += R"(}})";
                write_line_(line);
            }

            /*
            Rotate when file grows beyond limit
            If not rotates, do rolling fsync every N MiB to bound loss on crash
            */
            void rotate_if_needed() override{
                if (!fp_) return;
                const std::size_t max_bytes = cfg_.segment_max_mb * 1024ull * 1024ull;
                if (written_bytes_ >= max_bytes){
                    rotate_segment_();
                } else if (cfg_.fsync_policy == FsyncPolicy::EveryNMB){
                    const std::size_t nbyte = cfg_.fsync_n_mb * 1024ull * 1024ull;
                    if ((written_bytes_ - last_fsync_mark_) >= nbyte){
                        ::fflush(fp_);
                        #if defined(_WIN32)
                            _commit(_fileno(fp_));
                        #else
                            ::fsync(fileno(fp_));
                        #endif
                        last_fsync_mark_ = written_bytes_;
                    }
                }
            }

            // flush 
            void flush() override{
                if (!fp_) return;
                ::fflush(fp_);
                #if defined(_WIN32)
                    _commit(_fileno(fp_));
                #else
                    ::fsync(fileno(fp_));
                #endif
            }

        private:
            // Public policy
            enum class FsyncPolicy{EverySegment, EveryNMB};

            // configs
            struct RecorderConfig{
                std::string out_dir;
                std::string schema_dir;
                std::size_t segment_max_mb{256};
                FsyncPolicy fsync_policy{FsyncPolicy::EveryNMB};
                std::size_t fsync_n_mb{16};
                int tick_decimation{1};
                std::string controller_id;
                std::string asset_id;
                ictk::CommandMode fixed_mode{ictk::kPrimary};
            };

            // append a newline stream to file
            void write_line_(const std::string &line){
                /// @todo Check for short writes -> under disk errors
                if (!fp_) return;
                const std::string out = line + "\n";
                const size_t n = std::fwrite(out.data(), 1, out.size(), fp_);
                written_bytes_ += n;    // update byte counter
            }

            // to generate unique file name with prefix if defined GIT_SHA + time
            static std::string make_filename_(const std::string& dir){
                static std::atomic<uint64_t> seq{0};
                const std::string ts = utc_timestamp_filename();
                const uint64_t s = seq.fetch_add(1, std::memory_order_relaxed);
                return (fs::path(dir) / ("ictk_" + std::string(GIT_SHA) + "_" + ts + "_" + std::to_string(s) + ".jsonl")).string();
            }

            void ensure_open_(){
                if (fp_) return;
                open_new_file_();
            }

            // Open new seg then write a meta line first
            void open_new_file_(){
                current_path_ = make_filename_(cfg_.out_dir);
                fp_ = std::fopen(current_path_.c_str(), "wb");
                written_bytes_ = 0;
                last_fsync_mark_ = 0;

                if (!fp_){
                    // // no evidence -> no audit
                    std::fprintf(stderr, "ictk_recorder: failed to open '%s'\n", current_path_.c_str());
                    return;
                }

                std::string meta;
                meta.reserve(256);
                meta += R"({"meta":{"schema_backend":"jsonl","dt_ns":)";
                meta += std::to_string(static_cast<long long>(dt_ns_hint_));
                meta += R"(,"ictk_version":")" + std::string(ictk::kVersionStr) + R"(",)";
                meta += R"("git_sha":")" + std::string(GIT_SHA) + R"(","schema_registry_snapshot":[]}})";
                write_line_(meta);

                seq_ = 0; // reset per seg
            }

            void close_current_(){
                if (!fp_) return;
                ::fflush(fp_);
                #if defined(_WIN32)
                    _commit(_fileno(fp_));
                #else
                    ::fsync(fileno(fp_));
                #endif
                std::fclose(fp_);
                fp_ = nullptr;
            }

            void rotate_segment_(){
                close_current_();
                open_new_file_();
            }

            // floor formatting
            static inline std::string to_fix_(double v){
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.9g", v);
                return std::string(buf);
            }

            // wite only Nth tick
            bool decim_skip_(t_ns /*t*/){
                if (cfg_.tick_decimation <= 1) return false;
                return (tick_index_ ++ % static_cast<std::uint64_t>(cfg_.tick_decimation)) != 0;
            }

            
        private:
            RecorderConfig cfg_;        // params
            std::FILE* fp_{nullptr};    // seg handle
            std::string current_path_{};// active path

            // for rotating and rolling fsync
            std::size_t written_bytes_{0};
            std::size_t last_fsync_mark_{0};

            long long dt_ns_hint_{0};
            long long utc_anchor_ns_{0};
            long long mono_anchor_ns_{0};

            long long prev_t_{-1};
            long long first_t_{-1};

            std::uint64_t seq_{0};
            std::uint64_t tick_index_{0};

            detail::KpiAcc acc_{};
    };

    // // Factory
    std::unique_ptr<Recorder> Recorder::open(const RecorderOptions& opt){
        #if ICTK_RECORDER_BACKEND_MCAP
            return make_mcap_recorder(opt);
        #else
            return std::unique_ptr<Recorder>(new RecorderJsonl(opt));
        #endif
    }

} // namespace ictk::tools