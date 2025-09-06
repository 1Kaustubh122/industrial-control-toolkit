#include <chrono> // to measure time
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cinttypes>
#include <algorithm>
#include <cassert>
#include <cstring>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

// platform specific includes
#if defined(_WIN32)
    #include <windows.h>
#elif defined(__linux__)
    #include <pthread.h>
    #include <sched.h>
    #include <unistd.h>
    #include <sys/mman.h> 
#endif

using namespace ictk;
using namespace ictk::control::pid;


/*
On Linux: forces this thread to run only on CPU core 0
On Windows: same idea with SetThreadAffinityMask
Aim: To reduce jitter from OS Scheduling (thread hops cores, caches flush -> noisy timings)
*/
// CPU pinning (best effort)
static void pin_thread_best_effort(){
#if defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#elif defined(_WIN32)
    SetThreadAffinityMask(GetCurrentThread(), 1);
#endif
}

// // Latency percentiles + min/max
struct Stats {
    /*
    Latency Profile:

    p50 = median (50th Percentile). Half runs were faster, half were slower
    p95 = 95th Percentile, 95% faster rest slower
    p99 = 99th Percentile, Only 1% slower 
    p999 = 99.9th percentile, only 0.1% slower than this

    jmin = minimum latency
    jmax = maximum latency
    */
    double p50, p95, p99, p999, jmin, jmax;
};

// // sort timing
static Stats summarize(std::vector<double>& ns){
    // ns = vector of all latencies  -> sorting them

    std::sort(ns.begin(), ns.end());
    const std::size_t n = ns.size();
    assert(n > 0);

    auto q = [&](double p) -> double {
        const double dnm = static_cast<double>(n - 1u);
        const double pos = p * dnm;
        const std::size_t i = static_cast<std::size_t>(pos);
        return ns[i];
    };

    return {
        q(0.50),
        q(0.95),
        q(0.99),
        q(0.999),
        ns.front(),
        ns.back()
    };
}

int main(int argc, char** argv){
    // Defaults
    int nu_i = 1;
    int iters = 200000;
    long long dt_arg_ns = 1'000'000; // 1 ms

                    // str to int
    if (argc > 1) nu_i = std::atoi(argv[1]);
    if (argc > 2) iters = std::atoi(argv[2]);
    if (argc > 3) dt_arg_ns = std::strtoll(argv[3], nullptr, 10);

    bool opt_sat=false, opt_rate=false, opt_jerk=false, opt_no_header=false;
    for (int i = 4; i < argc; ++i){
        if (std::strcmp(argv[i], "--sat") == 0)       opt_sat = true; 
        else if (std::strcmp(argv[i], "--rate") == 0) opt_rate = true;
        else if (std::strcmp(argv[i], "--jerk") == 0) opt_jerk = true;
        else if (std::strcmp(argv[i], "--no-header") == 0) opt_no_header = true;
    }

    const std::size_t nu = static_cast<std::size_t>(nu_i);
    dt_ns dt = static_cast<dt_ns>(dt_arg_ns);

    pin_thread_best_effort();

    Dims d{
        .ny = nu,
        .nu = nu,
        .nx = 0
    };

            // 1 shift left by 20 buts 1 << 20 => 2^20 = 1,048,576 / 1024 => 1024/1024 = 1 MB
    std::vector<std::byte> storage(1 << 20);
    MemoryArena arena(storage.data(), storage.size());

    PIDCore pid;
    if (pid.init(d, dt, arena, {}) != Status::kOK) return 2;

    PIDConfig c{};

    // PID configs:
    std::vector<Scalar> Kp(nu, 1.5), Ki(nu, 0.5), Kd(nu, 0.1);
    std::vector<Scalar> beta(nu, 1.0), gamma(nu, 0.0);
    std::vector<Scalar> tf(nu, 0.01), bias(nu, 0.0);

    c.Kp = {Kp.data(), nu};
    c.Ki = {Ki.data(), nu};
    c.Kd = {Kd.data(), nu};
    c.beta = {beta.data(), nu};
    c.gamma = {gamma.data(), nu};
    c.tau_f = {tf.data(), nu};
    c.u_ff_bias = {bias.data(), nu};

    // Safety toggles
    static Scalar umin[]{-1.0}, umax[]{1.0};
    static Scalar du[]{5.0};
    static Scalar ddu[]{50.0};

    if (opt_sat){
        c.umin = {umin, 1};
        c.umax = {umax, 1}; 
    }
    if (opt_rate){
        c.du_max = {du, 1};
    } 
    if (opt_jerk){
        c.du_max = {du, 1}; 
        c.ddu_max = {ddu, 1}; 
    }

    if (pid.configure(c) != Status::kOK) return 3;
    if (pid.start() != Status::kOK) return 4;

    // buffer
    std::vector<Scalar> y(nu, 0), r(nu, 1.0), u(nu, 0);

    PlantState ps{
        .y = std::span<const Scalar>(y.data(), nu),
        .xhat = {},
        .t = 0,
        .valid_bits = (nu >= 64 ? ~0ull : ((1ull << nu) - 1ull))
    };

    Setpoint sp{
        .r = std::span<const Scalar>(r.data(), nu),
        .preview_horizon_len = 0
    };

    Result res{
        .u = std::span<Scalar>(u.data(), nu),
        .health = {}
    };

    UpdateContext ctx;

    mlockall(MCL_CURRENT | MCL_FUTURE);

    // warmup caches, branch predictors, memory, first calls are noisy
    for (int k = 0; k < 10000; ++k) {
        ps.t += dt;
        ctx.plant = ps;
        ctx.sp = sp;
        (void)pid.update(ctx, res);
    }

    // Benchmark

    using clk = std::chrono::steady_clock;
    constexpr int BATCH = 64; // amortize timer cost

    auto run_loop = [&](bool do_pid) {
        std::vector<double> ns(static_cast<std::size_t>(iters));
        for (int k = 0; k < iters; ++k) {
            auto t0 = clk::now();
            for (int j = 0; j < BATCH; ++j){
                ps.t += dt;
                ctx.plant = ps;
                ctx.sp = sp;
                if (do_pid) (void)pid.update(ctx, res);
            }
            auto t1 = clk::now();
            ns[static_cast<std::size_t>(k)] =
                std::chrono::duration<double, std::nano>(t1 - t0).count() / static_cast<double>(BATCH);
        }
        return summarize(ns);
    };

    auto S_null = run_loop(false);
    auto S_pid  = run_loop(true);

    Stats S_net{
        S_pid.p50 - S_null.p50,
        S_pid.p95 - S_null.p95,
        S_pid.p99 - S_null.p99,
        S_pid.p999 - S_null.p999,
        S_pid.jmin - S_null.jmin,
        S_pid.jmax - S_null.jmax
    };

    if (!opt_no_header){
        std::puts("label, nu, dt_ns, iters, p50, p95, p99, p999, jmin, jmax, tag1, tag2, tag3, tag4, build");
    }

    auto report = [&](const Stats& S, const char* label){
        std::printf(
            "%s, %zu, %lld, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %s ,%s ,%s, %s, %s\n",
            label,
            nu,
            static_cast<long long>(dt),
            iters,
            S.p50, S.p95, S.p99, S.p999, S.jmin, S.jmax,
            "na", "na", "na", "na", "RelWithDebInfo"
        );
    };

    if (!opt_no_header) std::puts("raw (null loop):");
    report(S_null, "null");

    if (!opt_no_header) std::puts("pid (loop+timer):");
    report(S_pid,  "pid");
    
    if (!opt_no_header) std::puts("net (pid only approx):");
    report(S_net,  "net");

    return 0;
}