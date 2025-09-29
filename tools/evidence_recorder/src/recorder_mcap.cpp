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

#include "kpi_calc.hpp"         // Kpi accum 
#include "env_buildinfo.hpp"    // build BuildIinfo struct from flags

#include "ictk/io/kpi.hpp"
#include "ictk/core/health.hpp"
#include "ictk/tools/recorder.hpp"

// // value check gate -> compile mcap + flatbuffer
#if ICTK_RECORDER_BACKEND_MCAP
    #include <mcap/writer.hpp>
    #include <flatbuffers/flatbuffers.h>

    // hashing 
    #include "hash.hpp"

    // auto generated schema
    #include "ictk_metrics_generated.h"
#endif

// For windows
#if defined(_WIN32)
  #include <io.h>
  #include <windows.h>
#else
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <unistd.h>
#endif

// if GIT_SHA else use unknown
#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

// // alias for std::filesystem 
namespace fs = std::filesystem;

namespace ictk::tools{
    #if ICTK_RECORDER_BACKEND_MCAP
        using namespace std::chrono;

        /*
        Gets current UTC wall clock time in nanosecs since Unix epoch
        returns it as string
        */
        static inline std::string utc_ns_string(){
            auto now = system_clock::now();
            auto ns = duration_cast<nanoseconds>(now.time_since_epoch()).count();
            return std::to_string((long long)ns);
        } // std::string utc_ns_string

        /*
        Record which kernel monotonic clock is in use
        this is compile evidence -> tells which time source system trusted
        returns string -> with the clock source label
        */
        static inline std::string kernel_clocksource(){
            // compile witht he linux path if compiler defines __linux__
            #if defined(__linux__)
                // // abs sysfs path that exposes the current system timekeeping clock source -> tsc / hpet / acpi_pm  
                const char* p="/sys/devices/system/clocksource/clocksource0/current_clocksource";

                // // bindary read 
                if (std::FILE* f = std::fopen(p, "rb")){
                    
                    // fixed stack buffer for contents
                    char buf[128];
    
                    // read upto 127 bytes into buf -> Returns byte reas -> leave 1 byte to place a terminator
                    size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
                    const bool err = std::ferror(f);
                    std::fclose(f); // close file

                    if (err) return "unknown";

                    buf[n] = 0;
    
                    /*
                    trims trailing newline or CRFL by scanning only the bytes read
                    convert to 0 and stops
                    */
                    for (size_t i=0; i<n; ++i){
                        if (buf[i] == '\n' || buf[i] == '\r'){
                            buf[i] = 0;
                            break;
                        }
                    }
    
                    // C string into std::string if *buf
                    return *buf ? std::string(buf) : std::string("unknown");

                }

                return "unknown"
            
            // for windows -> QPC -> query Performance Counter
            #elif defined(_WIN32)
                return "QPC";
            // for macOS 
            #elif defined(__APPLE__)
                return "mach_absolute_time";
            #else
            // other platform
                return "unknown";
            #endif
        } // std::string kernel_clocksource

        // // Read entire file into a memory buffer 
        static inline std::vector<uint8_t> read_file_bin(const std::string &p){
            std::FILE* f = std::fopen(p.c_str(), "rb");
            if (!f) return {};

            // output buffer that will hold an empty vector
            std::vector<uint8_t> b;

            // pre allocate 4096 bytes capacity to avoid repeated reallocation
            b.reserve(4096);

            // stack scratch buffer for chunked reads
            uint8_t buf[4096];

            // to hold number of bytes read
            size_t n;

            /*
            Loop till EOF
            tries to read up to 4096 bytes into buf
            return actual bytes read stored in n
            stops when 0 (EOF or err)
            */
            while ((n=std::fread(buf, 1, sizeof(buf), f)) > 0) b.insert(b.end(), buf, buf + n);

            std::fclose(f);
            return b;
        } // std::vector<uint8_t> read_file_bin

        /*
        called in write tick when recording Health message
        ensures the logged health record carries a mode 
        */ 
        static inline ictk::metrics::Mode fb_mode(ictk::CommandMode m){

            // switch between different command mode
            switch (m)
            {
            case ictk::CommandMode::Residual:
                return ictk::metrics::Mode::Residual;

            case ictk::CommandMode::Shadow:
                return ictk::metrics::Mode::Shadow;

            case ictk::CommandMode::Cooperative:
                return ictk::metrics::Mode::Cooperative; 

            default:
                return ictk::metrics::Mode::Primary;
            }
        } // fb_mode
        
        class RecorderMcap final : public Recorder{
            public:                                      // zero init cfg and pre alloc FlatBufferBuilder with 1KB
                explicit RecorderMcap(const RecorderOptions& opt) : cfg_{}, builder_(1024){

                    // // Configs
                    cfg_.out_dir = opt.out_dir ? opt.out_dir : "evidence";
                    cfg_.schema_dir = opt.schema_dir ? opt.schema_dir : ICTK_FB_SCHEMA_DIR;
                    cfg_.segment_max_mb = opt.segment_max_mb;
                    cfg_.fsync_n_mb = opt.fsync_n_mb;
                    cfg_.tick_decimation = opt.tick_decimation;
                    cfg_.dt_ns_hint = opt.dt_ns_hint;
                    cfg_.fsync_every_segment = (opt.fsync_policy == RecorderOptions::EverySegment);
                    cfg_.controller_id = opt.controller_id ? opt.controller_id : "";
                    cfg_.asset_id = opt.asset_id ? opt.asset_id : "";
                    cfg_.fixed_mode = opt.fixed_mode;

                    // ensures output dir exists 
                    std::error_code ec;
                    fs::create_directories(cfg_.out_dir, ec);

                    // // Write mcap header once
                    open_new_file_();
                    register_schemas_channels_();
                } // Ctor RecorderMcap
                
                // // Destructor 
                ~RecorderMcap() override{
                    close_current_();
                }

                // // emits one BuildInfo message on /ictk/buildinfo
                void write_buildinfo() override{

                    // init
                    auto bi = detail::make_buildinfo(
                        (std::uint64_t)(cfg_.dt_ns_hint > 0 ? cfg_.dt_ns_hint : 0),
                        cfg_.controller_id.c_str(),
                        cfg_.asset_id.c_str(),
                        (std::uint32_t) cfg_.tick_decimation
                    );

                    // Reuse flat buffer
                    builder_.Reset();

                    // Serializes BuildInfo per FlatBuffers schema
                    auto fb = ictk::metrics::CreateBuildInfo(
                        builder_,
                        builder_.CreateString(bi.ictk_version),
                        builder_.CreateString(bi.git_sha),
                        builder_.CreateString(bi.compiler),
                        builder_.CreateString(bi.flags),
                        builder_.CreateString(bi.scalar_type),
                        (uint64_t)bi.dt_ns,
                        builder_.CreateString(bi.controller_id),
                        builder_.CreateString(bi.asset_id),
                        (uint32_t)bi.tick_decimation
                    );

                    // finalise the buffer
                    builder_.Finish(fb);

                    // mcap msg envelope to publish this record
                    mcap::Message msg;

                    // route to /ictk/buildinfo channel
                    msg.channelId = ch_build_;

                    // monotonic per file
                    msg.sequence = seq_++;

                    // Timestamps the Bildinfo message -> uses monotonic anchor if set, else zero
                    const uint64_t t = mono_anchor_ns_ ? (uint64_t) mono_anchor_ns_ : 0ull;
                    msg.logTime = t;
                    msg.publishTime = t;

                    // transfer ownership of FlatBuffer bytes out of the builder without copying 
                    auto buf = builder_.Release();

                    // copy FlatBuffer into MCAP message payload -> MCAP msg owns its buffer
                    msg.data.resize(buf.size());
                    std::memcpy(msg.data.data(), buf.data(), buf.size());

                    // write to file 
                    writer_.write(msg);
                } // void write_buildinfo

                // // Bind Monotonic to UTC and stamp provenance
                void write_time_anchor(std::int64_t epoch_mono_ns, std::int64_t epoch_utc_ns) override{
                    // cache anchors for later timestamps and KPI time base (Internal state only)
                    mono_anchor_ns_ = epoch_mono_ns;
                    utc_anchor_ns_ = epoch_utc_ns;

                    // MCAP metadata
                    std::vector<mcap::KeyValue> meta;

                    // metadata
                    meta.push_back({"schema_backend", "flatbuffers"});
                    meta.push_back({"clock_domain", "MONO"});
                    meta.push_back({
                        "monotonic_to_utc_ns",
                        std::string("{\"epoch_mono_ns\":")+std::to_string(epoch_mono_ns)+
                        ",\"epoch_utc_ns\":"+std::to_string(epoch_utc_ns)+"}"
                    });
                    meta.push_back({"kernel_clocksource", kernel_clocksource()});

                    // Rebuilds BuildInfo to reuse normalized values
                    auto bi = detail::make_buildinfo(
                        static_cast<std::uint64_t> (cfg_.dt_ns_hint > 0 ? cfg_.dt_ns_hint : 0),
                        cfg_.controller_id.c_str(),
                        cfg_.asset_id.c_str(),
                        static_cast<std::uint32_t> (cfg_.tick_decimation)

                    );

                    // Inject loop period and provenance triplet for quick reads without parsing a message
                    meta.push_back({"dt_ns", std::to_string(bi.dt_ns)});
                    meta.push_back({"ictk_version", bi.ictk_version});
                    meta.push_back({"git_sha", bi.git_sha});

                    // if BFBA file exists -> compute SHA-256 and embed one-time registry snapshot JSON array
                    const auto bfbs_path = (fs::path(cfg_.schema_dir)/"ictk_metrics.bfbs").string();
                    std::error_code fec;
                    if (fs::exists(bfbs_path, fec)){
                        const auto bfbs_sha = hash::sha256_file(bfbs_path.c_str());
                        const auto bfbs_hex = hash::to_hex({bfbs_sha.data(), bfbs_sha.size()});
                        meta.push_back({
                            "schema_registry_snapshot",
                            std::string("[{\"name\":\"ictk_metrics.bfbs\",\"version\":\"1\",\"sha256\":\"")+bfbs_hex+"\"}]"
                        });
                    }

                    // build and writes an MCAP metadata
                    mcap::Metadata md;

                    md.name = "ictk";
                    md.metadata = std::move(meta);
                    writer_.writeMetadata(md);
                } // void write_time_anchor
                
                // // emit Tick and Health, updates KPIs, rotate if needed
                void write_tick(const TickSample& s) override{
                    // Guard -> Decimation gate -> only every Nth tick passes
                    if (cfg_.tick_decimation > 1 && (tick_index_++ % cfg_.tick_decimation) != 0) return;

                    // capture start time once for relative time KPI integration
                    if (first_t_ < 0) first_t_ = s.t;

                    // enforce strictly increaing timestamps for MCAP 
                    if (prev_t_ >= 0 && s.t <= prev_t_) prev_t_ += 1;
                    else prev_t_ = s.t;

                    // cast 
                    const uint64_t t =(uint64_t) prev_t_;
                    
                    // // Tick -> reuse FlatBufferBuilder buffer -> Zero alloc
                    builder_.Reset();

                    // Serialize tick per schema
                    auto tk = ictk::metrics::CreateTick(
                        builder_,
                        (uint64_t)(++tick_seq_),
                        t,
                        s.y0,
                        s.r0,
                        s.u_pre0,
                        s.u_post0
                    );
                    // Finish -> prepare buffer
                    builder_.Finish(tk);
                    
                    // mcap envelope
                    mcap::Message msg;

                    msg.channelId = ch_tick_;
                    msg.sequence = seq_++;
                    msg.logTime = t;
                    msg.publishTime = t;

                    // Payload copy into MCAP message and write out
                    auto buf = builder_.Release();
                    msg.data.resize(buf.size());
                    std::memcpy(msg.data.data(), buf.data(), buf.size());
                    writer_.write(msg);

                    // // Health
                    msg.channelId = ch_health_;

                    // reset for next record
                    builder_.Reset();

                    // Serialize Health
                    auto hl = ictk::metrics::CreateHealth(
                        builder_,
                        (uint64_t)s.h.deadline_miss_count,
                        (double)s.h.saturation_pct,
                        (uint64_t) s.h.rate_limit_hits,
                        (uint64_t) s.h.jerk_limit_hits,
                        (bool) s.h.fallback_active,
                        (bool) s.h.novelty_flag,
                        (double) s.h.aw_term_mag,
                        (double) s.h.last_clamp_mag,
                        (double) s.h.last_rate_clip_mag,
                        (double) s.h.last_jerk_clip_mag,
                        fb_mode(cfg_.fixed_mode)
                    );

                    // finish health
                    builder_.Finish(hl);

                    // Timestamp Health same as tick
                    msg.logTime = t;
                    msg.publishTime = t;
                    msg.sequence = seq_++;

                    // Write health payload
                    buf = builder_.Release();
                    msg.data.resize(buf.size());
                    std::memcpy(msg.data.data(), buf.data(), buf.size());
                    writer_.write(msg);

                    // compute absolute seconds since start of ITAE
                    const double t_s = double(t - (uint64_t)first_t_) / 1e9;

                    // update KPI accum from r/y/u
                    acc_.on_tick(t_s, s.r0, s.y0, s.u_post0);

                    // mark health record was written
                    acc_.on_health_written();

                    // commit tick
                    acc_.on_tick_commit();

                    // check segment size and fsync policy
                    rotate_if_needed();
                } // void write_tick

                // // Emit KPI summary
                void write_kpi(const ictk::KpiCounters& k) override{
                    // pick timestamp -> prefer last tick (prev_t)
                    const uint64_t t = prev_t_ >= 0 ? (uint64_t)prev_t_ : (uint64_t)mono_anchor_ns_;

                    // reuse buffer 
                    builder_.Reset();

                    // compute p50/p95/p99 
                    acc_.finalize_latency_percentiles();

                    //  Serialize KPI
                    auto kp = ictk::metrics::CreateKpi(
                        builder_,

                        (uint64_t)k.updates,
                        (uint64_t)k.watchdog_trips,
                        (uint64_t)k.fallback_entries,
                        (uint64_t)k.limit_hits,

                        acc_.iae, acc_.itae, acc_.tvu,
                        acc_.p50_lat_us, acc_.p95_lat_us, acc_.p99_lat_us,

                        (uint64_t) acc_.health_gap_frames
                    );

                    // finish the buffer
                    builder_.Finish(kp);

                    // for /ictk/kpi_report 
                    mcap::Message msg;
                    msg.channelId = ch_kpi_;
                    msg.sequence = seq_++;
                    msg.logTime = t;
                    msg.publishTime = t;

                    // copy flat buffer into MCAP message 
                    auto buf = builder_.Release();
                    msg.data.resize(buf.size());
                    std::memcpy(msg.data.data(), buf.data(), buf.size());
                    writer_.write(msg);
                } // void write_kpi
                
                // seg roll and periodic fsync
                void rotate_if_needed() override{
                    // conv MB limit to bytes
                    const std::size_t max_bytes = cfg_.segment_max_mb * 1024ull * 1024ull;

                    // get current file size without exceptions
                    std::error_code ec;
                    const auto sz = fs::file_size(mcap_path_, ec);

                    // Hard rotate when size threshold reached
                    if (!ec && sz >= max_bytes){
                        rotate_segment_(); 
                        return; 
                    }

                    // rolling -> if policy is every N MB
                    if (!cfg_.fsync_every_segment) {
                        const std::size_t nbytes = cfg_.fsync_n_mb * 1024ull * 1024ull;
                        if (!ec && (sz - last_fsync_mark_) >= nbytes){ 
                            flush(); 
                            last_fsync_mark_ = sz; 
                        }
                    }
                } // void rotate_if_needed

                // // Force flush
                void flush() override{
                    // Guard -> No file -> no Flush
                    if (!fp_) return;

                    // Push libc buffers to OS kernel
                    std::fflush(fp_);

                    // for different OS
                    #if defined(_WIN32)
                        _commit(_fileno(fp_));
                    #else
                        ::fsync(fileno(fp_));
                    #endif
                } // void flush

            private:
                // configs -> runtime knobs copied from RecorderOptions at construction 
                struct RecorderConfig{
                    std::string out_dir;                // file destination
                    std::string schema_dir;             // file destination
                    std::size_t segment_max_mb{256};    // size threshold for rotation
                    std::size_t fsync_n_mb{16};         // rolling fsync casdence if not per segment
                    bool fsync_every_segment{false};    // policy toggele
                    int tick_decimation{1};             // emit every Nth tick
                    long long dt_ns_hint{0};            // nominal loop period hint for metadata
                    std::string controller_id;          // provenance
                    std::string asset_id;               // provenance
                    ictk::CommandMode fixed_mode{       // runtime flag 
                        ictk::kPrimary
                    };
                }; //struct RecorderConfig

                static std::string make_filename_(const std::string& dir, const char* ext){
                    // TU local counter -> ensure uniqueness if multiple files open within same nanoseconds
                    static std::atomic<uint64_t> seq{0};

                    // UTC nano secs for time stamped name
                    const std::string utcns = utc_ns_string();
                    const uint64_t s = seq.fetch_add(1, std::memory_order_relaxed);

                    return (fs::path(dir) / ("ictk_" + std::string(GIT_SHA) + "_" + utcns + "_" + std::to_string(s) + ext)).string();
                } // std::string make_filename_

                void open_new_file_(){
                    /*
                    Picks the file path
                    Sidecar is same stem with .sidecar.json
                    open MCAP -> read and write mode
                    abort if fail
                    */
                    mcap_path_ = make_filename_(cfg_.out_dir, ".mcap");
                    sidecar_path_ = mcap_path_;
                    sidecar_path_.replace_extension(".sidecar.json");
                    fp_ = std::fopen(mcap_path_.c_str(), "wb+");
                    if (!fp_){
                        std::fprintf(stderr,"ictk_mcap: fopen failed: %s\n", mcap_path_.c_str());
                        std::abort();
                    }

                    // Configure writer
                    mcap::McapWriterOptions wopt;
                    wopt.compression = mcap::Compression::None;
                    wopt.enableStatistics = false;
                    writer_.open(fp_, wopt);

                    // reset rotation fsync 
                    last_fsync_mark_ = 0;
                    seq_ = 1;
                    tick_seq_ = 0;
                }// void open_new_file_

                void close_current_(){
                    // close MCAP stream 
                    if (!fp_) return;
                    writer_.close();
                    #if defined(_WIN32)
                        _commit(_fileno(fp_));
                    #else 
                        ::fsync(fileno(fp_));
                    #endif

                    // force kernel to write disk
                    std::fclose(fp_);
                    fp_ = nullptr;

                    // compute payload BLAKE3-256 over the final MCAP file and SHA 256 over the schema bfbs
                    const auto blake = hash::blake3_256_file(mcap_path_.c_str());
                    const auto blake_hex = hash::to_hex({blake.data(), blake.size()});
                    const auto bfbs_sha = hash::sha256_file((fs::path(cfg_.schema_dir)/"ictk_metrics.bfbs").string().c_str());

                    // produce hex string
                    const auto bfbs_hex = hash::to_hex({bfbs_sha.data(), bfbs_sha.size()});

                    std::FILE* sc = std::fopen(sidecar_path_.c_str(), "wb");

                    // write compact JSON sidecar with payload hash and schema hash list
                    if(sc){
                        std::string j;
                        j += R"({"payload_hash":{"alg":"BLAKE3-256","value":")" + blake_hex + R"("},)";
                        j += R"("bfbs_hashes":[{"name":"ictk_metrics.bfbs","alg":"SHA-256","value":")" + bfbs_hex + R"("}]})";
                        std::fwrite(j.data(), 1, j.size(), sc);
                        std::fclose(sc);
                    }
                }// void close_current_

                void register_schemas_channels_(){
                    // load FlatBuffers binday schema bytes
                    const auto bfbs_path = (fs::path(cfg_.schema_dir)/"ictk_metrics.bfbs").string();
                    const auto bfbs = read_file_bin(bfbs_path);

                    // Register these four schema records with the writer -> return IDs
                    auto sidBuild = writer_.addSchema({"ictk.metrics.BuildInfo", "flatbuffer", bfbs});
                    auto sidTick = writer_.addSchema({"ictk.metrics.Tick", "flatbuffer", bfbs});
                    auto sidHealth = writer_.addSchema({"ictk.metrics.Health", "flatbuffer", bfbs});
                    auto sidKpi = writer_.addSchema({"ictk.metrics.Kpi", "flatbuffer", bfbs});

                    // template channel descriptor
                    mcap::Channel ch;
                    ch.messageEncoding = "flatbuffer";

                    // create these channels bounds to their schemas
                    ch.schemaId = sidBuild;
                    ch.topic = "/ictk/buildinfo";
                    ch_build_ = writer_.addChannel(ch);

                    ch.schemaId = sidTick;
                    ch.topic = "/ictk/tick";
                    ch_tick_ = writer_.addChannel(ch);
                    
                    ch.schemaId = sidHealth;
                    ch.topic = "/ictk/health";
                    ch_health_ = writer_.addChannel(ch);

                    ch.schemaId = sidKpi;
                    ch.topic = "/ictk/kpi_report";
                    ch_kpi_ = writer_.addChannel(ch);
                }// void register_schemas_channels_

                // // rotate_if_needed
                void rotate_segment_(){
                    // close current file
                    close_current_();

                    // open new file and re register schemas for the new container
                    open_new_file_();
                    register_schemas_channels_();

                    // reset KPI accum  
                    acc_.reset();

                    // reset time
                    first_t_ = -1;
                    prev_t_ = -1;
                } // void rotate_segment_

            private:
                // holds all configuration knobs
                RecorderConfig cfg_{};
                
                // raw file handle for the open .mcap file
                std::FILE* fp_{nullptr};

                // store mcap file path
                std::string mcap_path_{};

                // store json path .sidecar.json
                std::string sidecar_path_{};

                // tracks the file size at the last fsync 
                std::size_t last_fsync_mark_{0};

                // store time anchor
                long long mono_anchor_ns_{0};   // monotonic clock epoch (stable, non adjustable)
                long long utc_anchor_ns_{0};    // corresponding wall clock epoch
                long long first_t_{-1};         // nano sec timestamp
                long long prev_t_{-1};          // timestamp of last tick
                uint64_t seq_{1};               // Global message sequence number 
                uint64_t tick_seq_{0};          // Sequence counter for tick message 
                uint64_t tick_index_{0};        // raw counter for all incoming ticks before decimation

                // writer object 
                mcap::McapWriter writer_;

                flatbuffers::FlatBufferBuilder builder_;
                detail::KpiAcc acc_{};

                uint16_t ch_build_{0}, ch_tick_{0}, ch_health_{0}, ch_kpi_{0};
        };
    
    #endif

} // namespace ictk::tools