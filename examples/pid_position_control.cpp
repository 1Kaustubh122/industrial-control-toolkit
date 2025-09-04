#include <vector>
#include <cstdio>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/safety/anti_windup.hpp"
#include "ictk/control/pid/pid.hpp"

using namespace ictk;
using ictk::safety::AWMode;
using namespace ictk::control::pid;

/*
1. Compute P + I + D
2. Saturate to [-1, 1]
3. Rate limit to +- 0.005
4. Back calc AQ with Kt = 0.2
5. Send u
6. Plan integrates u
7. Repeat it 100x
*/
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

    // Controller configs:
    static Scalar Kp[]{2.0}, Ki[]{1.0}, Kd[]{0.1}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0}, tf[]{0.01};
    static Scalar umin[]{-1.0}, umax[]{1.0}, du[]{5.0};

    c.Kp={Kp,1}; 
    c.Ki={Ki,1}; 
    c.Kd={Kd,1}; 

    c.beta={beta,1}; 
    c.gamma={gamma,1};

    c.tau_f={tf,1}; 
    c.u_ff_bias={bias,1};

    // clamp [-1 to 1]
    c.umin={umin,1};
    c.umax={umax,1};

    c.du_max={du,1};

    c.aw_mode = AWMode::kBackCalc;
    c.Kt=0.2;

    assert(pid.configure(c)==Status::kOK);
    assert(pid.start()==Status::kOK);

    std::vector<Scalar> u(1,0), y(1,0), r(1,1.0);

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

    Result res{
        .u=std::span<Scalar>(u.data(),1),
        .health={}
    };

    for (int k=0;k<100;++k){
        ps.t += dt;

        UpdateContext ctx{
            ps,
            sp
        };

        assert(pid.update(ctx,res)==Status::kOK);

        const Scalar dt_s = (Scalar)dt * 1e-9; // 0.001
        y[0] += 20.0 * dt_s * u[0];            // scales correctly if dt changes

        std::printf("%d, u=%.6f, y=%.6f\n", k, (double)u[0], (double)y[0]);
    }
    return 0;
}