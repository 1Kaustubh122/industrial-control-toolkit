#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"
#include <vector>
#include <cassert>
#include <cmath>

using namespace ictk;
using namespace ictk::control::pid;

int main() {
    // SISO with both rate and jerk limits
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    };

    dt_ns dt = 1'000'000; // 1ms

    const Scalar dt_s = Scalar(dt) * 1e-9;

    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    PIDCore pid; assert(pid.init(d, dt, arena, {}) == Status::kOK);

    PIDConfig c{};

                    // Bigger step pressure as kp = 200
    static Scalar Kp[]{200.0}, Ki[]{0.0}, Kd[]{0.0}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0};

    static Scalar du[]{8.0};  
    static Scalar ddu[]{50.0};

    c.Kp={Kp,1}; 
    c.Ki={Ki,1}; 
    c.Kd={Kd,1}; 
    c.beta={beta,1}; 
    c.gamma={gamma,1}; 
    c.u_ff_bias={bias,1};
    c.du_max={du,1};
    c.ddu_max={ddu,1}; 

    assert(pid.configure(c)==Status::kOK);
    assert(pid.start()==Status::kOK);

                                        // Larger step to engage both
    std::vector<Scalar> u(1, 0), y(1, 0), r(1, 20.0);

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
    Scalar du_prev = 0.0;

    const Scalar rate_bound = du[0] * dt_s + 1e-12;
    const Scalar jerk_bound = ddu[0] * dt_s + 1e-12;

    for (int k=0; k<300; ++k) {
        ps.t += dt;
        auto st = pid.update({ps, sp}, res);
        assert(st == Status::kOK);

        const Scalar du_k = u[0] - u_prev;
        const Scalar ddu_k = du_k - du_prev;

        // Rate Lipschitz
        assert(std::fabs(du_k) <= rate_bound);
        
        // Jerk Lipschitz
        assert(std::fabs(ddu_k) <= jerk_bound);

        du_prev = du_k;
        u_prev  = u[0];
    }

    // both limiters hit
    assert(res.health.rate_limit_hits > 0);
    assert(res.health.jerk_limit_hits > 0);
    return 0;
}