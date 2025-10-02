#include <cstdio>
#include <vector>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"

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

int main(){
    // SISO
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    }; 

    dt_ns dt = 1'000'000; // 1ms

    alignas(64) std::byte buf[8192];
    MemoryArena arena(buf, sizeof(buf));

    // // Two independent PIDs. Outer = position PI, Inner = speed PIDF 

    PIDCore outer, inner;
    
    ok_or_die(outer.init(d, dt, arena, {}), "outer.init");
    ok_or_die(inner.init(d, dt, arena, {}), "inner.init");

    PIDConfig po{}, pi{};

    // // Outer and inner configs:
    // outer PI (no D) -> avoid derivative noise on position
    // inner PIDF -> Derivative on measurement only (gamma = 0)
    static Scalar z[]{0.0};
    static Scalar Kp_o[]{1.0}, Ki_o[]{0.2}, beta[]{1.0}, gamma[]{0.0};
    static Scalar Kp_i[]{2.0}, Ki_i[]{1.0}, Kd_i[]{0.1}, tf_i[]{0.01};

    po.Kp = {Kp_o,1};
    po.Ki = {Ki_o,1};
    po.Kd = {z,1};

    po.beta = {beta,1};
    po.gamma = {gamma,1};
    po.u_ff_bias = {z,1};

    pi.Kp = {Kp_i,1};
    pi.Ki = {Ki_i,1};
    pi.Kd = {Kd_i,1};
    pi.beta = {beta,1};
    pi.gamma = {gamma,1};
    pi.tau_f = {tf_i,1};
    pi.u_ff_bias = {z,1};

    
    ok_or_die(outer.configure(po), "outer.configure");
    ok_or_die(inner.configure(pi), "inner.configure");

    ok_or_die(outer.start(), "outer.start");
    ok_or_die(inner.start(), "inner.start");

    // Buffers: actual command, measured speed, measured position, reference position 
    std::vector<Scalar> u(1,0), y_pos(1,0), y_spd(1,0), r_pos(1,1.0);
    std::vector<Scalar> v_ref(1,0);

    Result u_inner{
        .u=std::span<Scalar>(u.data(), 1),
        .health={}
    };

    PlantState ps_spd{
        .y=std::span<const Scalar>(y_spd.data(), 1),
        .xhat= {},
        .t= 0,
        .valid_bits= 0x1
    };

    Setpoint sp_spd{
        .r=std::span<const Scalar>(y_pos.data(),1), 
        .preview_horizon_len = 0
    };


    Result u_outer{
        .u=std::span<Scalar>(v_ref.data(),1),
        .health={}
    };

    PlantState ps_pos{
        .y=std::span<const Scalar>(y_pos.data(),1),
        .xhat={},
        .t=0,
        .valid_bits=0x1
    };

    Setpoint sp_pos{
        .r=std::span<const Scalar>(r_pos.data(),1),
        .preview_horizon_len=0
    };

    for (int k=0;k<200;++k){
        ps_pos.t += dt;
        ok_or_die(outer.update({ps_pos, sp_pos}, u_outer), "outer.update");

        sp_spd.r = std::span<const Scalar>(v_ref.data(),1);
    
        ps_spd.t += dt; // tick
        ok_or_die(inner.update({ps_spd, sp_spd}, u_inner), "inner.update");

        // // plant
        y_spd[0] += static_cast<Scalar>(0.05) * u[0];
        y_pos[0] += y_spd[0] * static_cast<Scalar>(0.001); // overwrite measurement 
        std::printf(
            "%d, u=%.6f, v=%.6f, x=%.6f\n",
            k,
            static_cast<double>(u[0]),
            static_cast<double>(y_spd[0]),
            static_cast<double>(y_pos[0])
        );
    }
    return 0;
}