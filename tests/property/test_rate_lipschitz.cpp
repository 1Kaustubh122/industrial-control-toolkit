#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/safety/rate_limit.hpp"
#include "ictk/core/memory_arena.hpp"

using namespace ictk;

int main(){
    alignas(64) std::byte buf[4096];    // // 64 bit aligned buffer
    MemoryArena arena(buf, sizeof(buf));

    const dt_ns dt = 1'000'000; // 1ms

    safety::RateLimiter rl(10.0, dt, arena, 2); // rmax = 10, nu = 2
    std::vector<Scalar> u {0.0};                // start with size 1
    rl.reset(u);                                // send prev[] -> {0,0}

    // request a big jump
    std::vector<Scalar> cmd{100, -100};
    // // applt 5 ticks -> each tick u <- rmax*dt = 0.01

    for (int k=0; k<5; ++k){
        u = cmd;        // desired command per tick -> simulate fresh large request
        rl.apply(u);    // limiter clamps in place
        for ([[maybe_unused]] double ui: u) assert(std::abs(ui) <= (k+1) * 0.01 + 1e-12);
        /*
            each tick -> the limiter restricts the change from the previous emitted value by 0.01
            after k tick -> the magnitude cannot exceed (k+1)*0.01 from 0
        */
    }

    return 0;
}