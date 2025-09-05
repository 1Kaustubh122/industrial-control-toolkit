#include <vector>
#include <cmath>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using namespace ictk::control::pid;

int main(){
    [[maybe_unused]] Dims d{
        .ny = 1,
        .nu = 1,
        .nx = 0
    };

    dt_ns dt = 1'000'000;

    alignas(64) std::byte buf[4096];

    MemoryArena arena(buf, sizeof(buf));

    PIDCore pid;
    assert(pid.init(d, dt, arena, {}) == Status::kOK);

    PIDConfig c{};

    static Scalar Kp[]{2.0}, Ki[]{1.0}, Kd[]{0.0}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0};

    c.Kp={Kp,1}; 
    c.Ki={Ki,1}; 
    c.Kd={Kd,1}; 
    c.beta={beta,1}; 
    c.gamma={gamma,1}; 
    c.u_ff_bias={bias,1};

    assert(pid.configure(c)==Status::kOK); 
    assert(pid.start()==Status::kOK);

    std::vector<Scalar> u(1,0), y(1,0), r(1,0);
    [[maybe_unused]] Result res{
        .u=std::span<Scalar>(u.data(),1), 
        .health={}
    };
    [[maybe_unused]] PlantState ps{
        .y=std::span<const Scalar>(y.data(),1), 
        .xhat={}, .t=0, .valid_bits=0x1
    };
    [[maybe_unused]] Setpoint sp{
        .r=std::span<const Scalar>(r.data(),1), 
        .preview_horizon_len=0
    };

    // Alighn hold = 0 at r0=y0=0 -< output should be 0 next tick
    pid.align_bumpless(std::span<const Scalar>(u.data(), 1), sp.r, ps.y);
    ps.t += dt;
    assert(pid.update({ps, sp}, res) == Status::kOK);
    assert(std::abs(u[0]) <= 1e-12);
    return 0;
}