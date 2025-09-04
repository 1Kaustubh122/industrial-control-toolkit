#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"
#include "ictk/safety/anti_windup.hpp"

using namespace ictk;
using ictk::safety::AWMode;
using namespace ictk::control::pid;

int main(){
    // SISO
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    };

    dt_ns dt=1'000'000;

    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf,sizeof(buf));

    PIDCore pid; 
    assert(pid.init(d,dt,arena,{})==Status::kOK);

    PIDConfig c{};

    // test constants value
    static Scalar Kp[]{5.0}, Ki[]{2.0}, Kd[]{0.0}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0};
    static Scalar umin[]{-0.1}, umax[]{0.1};
    c.Kp={Kp,1}; 
    c.Ki={Ki,1}; 
    c.Kd={Kd,1}; 
    c.beta={beta,1}; 
    c.gamma={gamma,1}; 
    c.u_ff_bias={bias,1};
    c.umin={umin,1}; 
    c.umax={umax,1};

    c.aw_mode = AWMode::kBackCalc;    // enable back calc
    c.Kt = 0.5;

    assert(pid.configure(c)==Status::kOK); 
    assert(pid.start()==Status::kOK);

    std::vector<Scalar> u(1,0), y(1,0), r(1,10.0);
    Result res{
        .u=std::span<Scalar>(u.data(),1), 
        .health={}
    };

    PlantState ps{
        .y=std::span<const Scalar>(y.data(),1), 
        .xhat={}, 
        .t=0, 
        .valid_bits=0x1
    };

    Setpoint sp{
        .r=std::span<const Scalar>(r.data(),1), 
        .preview_horizon_len=0
    };

    ps.t += dt; 
    assert(pid.update({ps,sp},res)==Status::kOK);

    // anti windup
    const double expected = std::abs(0.1 - 50.0);
    assert(std::fabs(res.health.aw_term_mag - expected) < 1e-6);

    return 0;
}
