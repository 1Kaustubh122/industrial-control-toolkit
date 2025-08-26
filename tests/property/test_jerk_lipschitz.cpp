#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/core/memory_arena.hpp"
#include "ictk/safety/jerk_limit.hpp"

using namespace ictk;

int main(){
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));
    const dt_ns dt = 1'000'000; // 1ms

    safety::JerkLimiter jl(10.0, 50.0, dt, arena, 1); // rmax =10, jmax = 50
    
    std::vector<Scalar> u(1, 0), cmd(1,0);
    jl.reset(u);

    // Alternative big commands; check u bounded by jmax*dt
    const Scalar jstep = 50.0 * 1e-3;
    Scalar prev_u = 0, prev_du = 0;

    for (int k=0; k<20; ++k){
        cmd[0] = (k%2==0) ? 100 : -100;
        u = cmd;
        jl.apply(u);
        Scalar du = u[0] - prev_u;
        Scalar d2u = du - prev_du;
        assert(std::abs(d2u) <= jstep + 1e-12);
        prev_du = du;
        prev_u = u[0];
    }

    return 0;
}