#include <chrono> // to measure time
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cinttypes>
#include <algorithm>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

// platform specific includes
#if defined(_WIN32)
    #include <windows.h>
#elif defined(__linux__)
    #include <pthread.h>
    #include <sched.h>
    #include <unistd.h>
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
struct  Stats{
    /*
    Latency Profile:

    p50 = median (50th Percentile). Half runs were faster, half were slower
    p95 = 95th Percentile, 95% faster rest slower
    p99 = 99th Percentile, Only 1% slower 
    p999 = 99.9th percentile, only 0.1% slower than this

    jmin = minimum latency
    jmax = maximum latency
    */
    double p50 , p95, p99, p999, jmin, jmax;
};

// // sort timing
static Stats summarize(std::vector<double>& ns){
    // ns = vector of all latencies  -> sorting them
    std::sort(ns.begin(), ns.end());

    //              p = percentile
    auto q = [&](double p) -> double{
                            // index into sorted array
        const double pos = p * static_cast<double>(ns.size() - 1);
        const std::size_t i = static_cast<std::size_t>(pos);

        return ns[i];
    };
    double jmin = ns.front(), jmax = ns.back();
    return {
        q(0.50),
        q(0.95),
        q(0.99),
        q(0.999),
        jmin,
        jmax
    };
}

int main(int argc, char** argv){
    // CLI args
    int nu_ = 1;         // no of control channels
    int iters = 200000; // steps
    long long dt_arg_ns = 1'000'000; // 1ms

                    // str to int
    if (argc>1) nu_ = std::stoi(argv[1]);
    if (argc>2) iters = std::stoi(argv[2]);
    if (argc>3) dt_arg_ns = std::strtoll(argv[3], nullptr, 10);

    const std::size_t nu = static_cast<std::size_t>(nu_);

    
    pin_thread_best_effort();
    
    Dims d{
        .ny = static_cast<std::size_t>(nu),
        .nu = static_cast<std::size_t>(nu),
        .nx=0
    };
    
    dt_ns dt = static_cast<dt_ns>(dt_arg_ns);
    
                // 1 shift left by 20 buts 1 << 20 => 2^20 = 1,048,576 / 1024 => 1024/1024 = 1 MB
    std::vector<std::byte> storage(1<<20);
    MemoryArena arena(storage.data(), storage.size());

    PIDCore pid;
    if (pid.init(d, dt, arena, {}) != Status::kOK) return 2;


    PIDConfig c{};

    // PID configs:

    std::vector<Scalar> Kp(nu, 1.5), Ki(nu, 0.5), Kd(nu, 0.1), beta(nu, 1.0), gamma(nu, 0.0), tf(nu, 0.01), bias(nu, 0.0);

    c.Kp = {Kp.data(), nu};
    c.Ki = {Ki.data(), nu};
    c.Kd = {Kd.data(), nu};
    c.beta = {beta.data(), nu};
    c.gamma = {gamma.data(), nu};
    c.tau_f = {tf.data(), nu};
    c.u_ff_bias = {bias.data(), nu};

    if (pid.configure(c) != Status::kOK) return 3;
    if (pid.start() != Status::kOK) return 4;

    // buffer
    std::vector<Scalar> y(nu, 0), r(nu, 1.0), u(nu, 0);

    PlantState ps{
        .y = std::span<const Scalar> (y.data(), nu),
        .xhat = {},
        .t = 0,
        .valid_bits = (nu >= 64 ?~0ull:((1ull<<nu)-1ull))
    };

    Setpoint sp{
        .r = std::span<const Scalar> (r.data(), nu),
        .preview_horizon_len = 0
    };

    Result res{
        .u = std::span<Scalar>(u.data(), nu),
        .health = {}
    };

    UpdateContext ctx;

    // warmup caches, branch predictors, memory, first calls are noisy
    for (int k=0; k<10000; ++k){
        ps.t += dt;
        ctx.plant = ps;
        ctx.sp = sp;

        [[maybe_unused]] auto st_warm = pid.update(ctx, res);
        assert(st_warm == Status::kOK);
    }

    // Benchmark

    using clk = std::chrono::steady_clock;
    
    std::vector<double> ns;
    ns.reserve(iters);

    for (int k=0; k<iters; ++k){
        auto t0 = clk::now();
        ps.t += dt;
        ctx.plant = ps;
        ctx.sp = sp;
        (void)pid.update(ctx, res);
        auto t1 = clk::now();
        ns.push_back(std::chrono::duration<double, std::nano>(t1-t0).count());
    }

    auto S = summarize(ns);

    std::printf("pid,%zu,%lld,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s\n",
              nu, static_cast<long long>(dt), iters, S.p50, S.p95, S.p99, S.p999, S.jmin, S.jmax,
              "na", "na", "na", "na", "RelWithDebInfo");

    return 0;
}