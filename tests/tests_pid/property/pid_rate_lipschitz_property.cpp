#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using namespace ictk::control::pid;

int main() {
    // SISO
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    };

    dt_ns dt = 1'000'000; // 1ms

    const Scalar dt_s = Scalar(dt) * 1e-9;

    // fixed 4 KB
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    PIDCore pid; 
    
    assert(pid.init(d, dt, arena, {}) == Status::kOK);

    PIDConfig c{};

    // // PID CONFIGS:
    static Scalar Kp[]{100.0}, Ki[]{0.0}, Kd[]{0.0}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0};
    static Scalar du[]{5.0}; 

    c.Kp={Kp,1}; 
    c.Ki={Ki,1}; 
    c.Kd={Kd,1}; 
    c.beta={beta,1}; 
    c.gamma={gamma,1}; 
    c.u_ff_bias={bias,1};
    c.du_max={du,1}; // rate limiter ON

    assert(pid.configure(c)==Status::kOK);
    assert(pid.start()==Status::kOK);

                    // command, measurement, setpoint to 10 trigger
    std::vector<Scalar> u(1, 0), y(1, 0), r(1, 10.0); // big step

    Result res{ 
        .u=std::span<Scalar>(u.data(),1), .health={}
    };
    PlantState ps{
        .y=std::span<const Scalar>(y.data(),1), .xhat={}, .t=0, .valid_bits=0x1
    };
    Setpoint  sp{
        .r=std::span<const Scalar>(r.data(),1), .preview_horizon_len=0
    };

    Scalar u_prev = 0.0;
    const Scalar bound = du[0] * dt_s + 1e-12;

    for (int k=0; k<200; ++k) {
        ps.t += dt;
        
        auto st = pid.update({ps, sp}, res);
        assert(st == Status::kOK);

        const Scalar du_k = u[0] - u_prev;
        assert(std::fabs(du_k) <= bound);

        u_prev = u[0];
    }

    assert(res.health.rate_limit_hits > 0);
    return 0;
}
