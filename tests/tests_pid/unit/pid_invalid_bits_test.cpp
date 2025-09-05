#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using namespace ictk::control::pid;

/*
to check: PID rejectes updates if some plan measurement channels are marked invalid for MIMO systems.
*/

int main(){
    // MIMO -> Multi Input Multi Output
    [[maybe_unused]] Dims d{
        .ny=2,
        .nu=2,
        .nx=0
    };

    dt_ns dt = 1'000'000; // 1ms

    // 4KB fixed
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    PIDCore pid;
    assert(pid.init(d, dt, arena, {}) == Status::kOK);

    PIDConfig c{};

    static Scalar z[]{0.0}, o[]{1.0};

    c.Kp={o,1}; // all channel
    c.Ki={z,1}; // no integrator
    c.Kd={z,1}; // no derivative 
    c.beta={o,1}; 
    c.gamma={z,1}; 
    c.u_ff_bias={z,1};

    assert(pid.configure(c)==Status::kOK); 
    assert(pid.start()==Status::kOK); 

    std::vector<Scalar> u(2,0), y{0,0}, r{1,1};

    Result res{
        .u=std::span<Scalar>(u.data(),2), 
        .health={}
    };

    PlantState ps{
        .y=std::span<const Scalar>(y.data(),2),
        .xhat={}, 
        .t=0, 
        .valid_bits=0x1 // second bit invalid (01 in binary)
    };

    Setpoint sp{
        .r=std::span<const Scalar>(r.data(),2), 
        .preview_horizon_len=0
    };

    ps.t += dt;
    [[maybe_unused]] auto st = pid.update({ps, sp}, res);
    assert(st == Status::kPreconditionFail);

    assert(u[0] == 0 && u[1]==0);
    return 0;
}