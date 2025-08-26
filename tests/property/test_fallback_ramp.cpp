#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/safety/fallback.hpp"
#include "ictk/core/memory_arena.hpp"

using namespace ictk;

// // fallback output -> move towards safe_u
int main(){
    alignas(64) std::byte buf[4096];        // alloc buffer
    MemoryArena arena(buf, sizeof(buf));    // align arena for RT state
    const dt_ns dt = 1'000'000; // 1ms
    std::vector<Scalar> safe{0,0};          // safe tarhet 0 -> for both channel

    safety::FallbackPolicy fb(std::span<const Scalar>(safe.data(), safe.size()), 5.0, dt, arena, 2); // // rmax = 5/s
    std::vector<Scalar> u{100, -100};
    fb.reset_to(u);     // seed internal state to current ouput -> for bumpless entry
    fb.engage();

    for(int k=0; k<100; ++k){
        fb.apply(u);        // move each channel toward 0 by < 0.005 per tick
        assert(std::abs(u[0]) <= 100 - k*0.005 + 1e-9);
    }
    return 0;
}