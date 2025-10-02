#include <vector>
#include <cstdio>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"
#include "ictk/safety/anti_windup.hpp"

using namespace ictk;
using ictk::safety::AWMode;
using namespace ictk::control::pid;

static inline void ok_or_die(Status s, const char* msg){
    if (s != Status::kOK){
        std::fprintf(stderr, "%s failed (code=%d)\n", msg, static_cast<int>(s));
        std::fflush(stderr);
        std::exit(1);
    }
}

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
    
    // 1ms
    dt_ns dt=1'000'000;

    alignas(64) std::byte buf[4096]; 
    MemoryArena arena(buf,sizeof(buf));

    PIDCore pid;
    
    ok_or_die(pid.init(d, dt, arena, {}), "pid.init");

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

    ok_or_die(pid.configure(c), "pid.configure");
    ok_or_die(pid.start(), "pid.start");

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

    for (int k=0; k<100; ++k){
        ps.t += dt;

        UpdateContext ctx{
            ps,
            sp
        };

        ok_or_die(pid.update(ctx, res), "pid.update");

        const Scalar dt_s = static_cast<Scalar>(dt) * static_cast<Scalar>(1e-9); // 0.001
        y[0] += static_cast<Scalar>(20.0) * dt_s * u[0];            // scales correctly if dt changes

        std::printf(
            "%d, u=%.6f, y=%.6f\n", 
            k,
            static_cast<double>(u[0]),
            static_cast<double>(y[0])
        );
    }
    return 0;
}