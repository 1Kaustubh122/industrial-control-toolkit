#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using namespace ictk::control::pid;

int main(){
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    };

    dt_ns dt=1'000'000; // 1ms

    // fix 4 KB
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf,sizeof(buf));

    PIDCore pid; 
    assert(pid.init(d, dt, arena, {})==Status::kOK);

    PIDConfig c{};

    static Scalar z[]{0.0}, o[]{1.0};
    static Scalar bp[]{0.0, 1.0};
    static Scalar kp_tab[]{1.0, 3.0};
    static Scalar ki_tab[]{0.0, 0.0};
    static Scalar kd_tab[]{0.0, 0.0};
    static Scalar beta_tab[]{1.0, 1.0};
    static Scalar gamma_tab[]{0.0, 0.0};

    c.Kp={o,1}; c.Ki={z,1}; c.Kd={z,1}; c.beta={o,1}; c.gamma={z,1}; c.u_ff_bias={z,1};

    c.sched = ScheduleConfig{
        {bp,2},
        {kp_tab,2},
        {ki_tab,2},
        {kd_tab,2},
        {beta_tab,2},
        {gamma_tab,2} 
    };
    assert(pid.configure(c)==Status::kOK);
    assert(pid.start()==Status::kOK);

    std::vector<Scalar> u(1,0), y(1,0), r(1,1.0);

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

    // At var=0.2 vs 0.3, Kp changes linearly; output difference bounded by |ΔKp|*|e|
    ps.t += dt; y[0] = 0.2; 
    assert(pid.update({ps,sp},res)==Status::kOK); 

    const double u1=u[0];
    ps.t += dt; y[0] = 0.3; 
    assert(pid.update({ps,sp},res)==Status::kOK); 
    const double u2=u[0];

    const double dKp = (3.0-1.0)*(0.1); // Δvar=0.1 over [0,1]
    const double bound = std::abs(dKp * (1.0 - 0.3)); // e = r - y
    assert(std::abs(u2 - u1) <= bound + 1e-9);
    return 0;

}