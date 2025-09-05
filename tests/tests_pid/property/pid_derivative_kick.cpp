#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using namespace ictk::control::pid;


int main(){
    // SISO
    [[maybe_unused]] Dims d{
        .ny = 1,
        .nu = 1,
        .nx = 0
    };

    dt_ns dt = 1'000'000;

    alignas(64) std::byte buf[4096];

    MemoryArena arena(buf, sizeof(buf));

    auto run = [&](Scalar gamma) -> double{
        PIDCore pid;
        assert(pid.init(d, dt, arena, {}) == Status::kOK);

        PIDConfig c{};

        static Scalar ki0[]{0.0};
        static Scalar Kp[]{2.0}, Kd[]{1.0}, beta[]{1.0}, bias[]{0.0}, tf[]{0.02};

        c.Kp={Kp,1}; 
        c.Ki = {ki0, 1};
        c.Kd={Kd,1}; 
        c.beta={beta,1}; 
        c.gamma={&gamma,1};  // use per call's gamma
        c.tau_f = {tf, 1};   // derivative filter
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
            .xhat={}, 
            .t=0, 
            .valid_bits=0x1
        };
        [[maybe_unused]] Setpoint sp{
            .r=std::span<const Scalar>(r.data(),1), 
            .preview_horizon_len=0
        };

        [[maybe_unused]] double u0 = u[0]; // baseline = 0
        r[0] = 1.0;       // step setpoint at first real tick
        ps.t += dt;
        assert(pid.update({ps, sp}, res) == Status::kOK);   // compute one ticl
        return u[0] - u0; // size of output jump for this gamam 
    };


    // // two cases: gamma = 0 and gamma = 1
    [[maybe_unused]] const double d0 = run(0.0);
    [[maybe_unused]] const double d1 = run(1.0);

    assert(d1 > d0);
    return 0;

}

