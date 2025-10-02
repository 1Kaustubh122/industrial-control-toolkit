#include <cmath>
#include <cstdio>
#include <vector>
#include <filesystem>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"
#include "ictk/safety/anti_windup.hpp"

namespace fs = std::filesystem;

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

static fs::path find_repo_root(){
    fs::path p = fs::current_path();
    for (int i=0; i<20 && !p.empty(); ++i){
        if (fs::exists(p/".git") && fs::is_directory(p/"examples"))
            return p;
        p = p.parent_path();
    }
    return {};
}

static std::FILE* open_log_in_examples_pid(const char* filename){
    fs::path root = find_repo_root();
    fs::path out  = root.empty() ? (fs::current_path()/filename) : (root/"examples"/"pid"/"csv"/filename);
    if (!root.empty()) {
        std::error_code ec;
        fs::create_directories(out.parent_path(), ec);
    }
    std::FILE* f = std::fopen(out.string().c_str(),"w");
    if (!f){
        std::perror("fopen");
        std::fprintf(stderr,"path tried: %s\n", out.string().c_str());
        std::exit(1);
    }
    return f;
}

// PI synthesis for integrator plant: x[k+1]=x[k]+b*u[k] (double pole at exp(-dt/tau))
static inline void design_pi_integrator(Scalar b, Scalar dt_s, Scalar tau_s,
                                        Scalar& Kp, Scalar& Ki){
    const Scalar zc = std::exp(-dt_s / tau_s);
    Kp = 2*(1 - zc)/b;
    Ki = (zc - 1)*(zc - 1)/b;
}

int main(){
    // SISO
    Dims d{
        .ny=1,
        .nu=1,
        .nx=0
    };

    // 1 ms
    dt_ns dt = 1'000'000;

    alignas(64) std::byte buf[8192];
    MemoryArena arena(buf, sizeof(buf));

    //  INNER SPEED LOOP ONLY 
    PIDCore inner;
    ok_or_die(inner.init(d, dt, arena, {}), "inner.init");

    PIDConfig pi{};

    // Autotune PI for v[k+1] = v[k] + 0.05*u[k]
    static Scalar z[]{0.0}, gamma[]{0.0};
    const Scalar dt_s = 1e-3;
    const Scalar b_i  = 0.05;      // plant gain to speed
    const Scalar tau_i = 0.015;    // ~15 ms speed-loop time constant

    Scalar Kp_i_v{}, Ki_i_v{};
    design_pi_integrator(b_i, dt_s, tau_i, Kp_i_v, Ki_i_v);

    static Scalar beta_i[]{0.6};
    static Scalar Kp_i[]{Kp_i_v};
    static Scalar Ki_i[]{Ki_i_v};
    static Scalar umin_i[]{-1.0}, umax_i[]{1.0};
    static Scalar du_i[]{400.0};
    static Scalar tf_i[]{0.0};

    pi.Kp = {Kp_i,1};
    pi.Ki = {Ki_i,1};
    pi.Kd = {z,1};

    pi.beta = {beta_i,1};
    pi.gamma = {gamma,1};
    pi.tau_f = {tf_i,1};
    pi.u_ff_bias = {z,1};

    pi.umin = {umin_i,1};
    pi.umax = {umax_i,1};

    pi.du_max = {du_i,1};

    pi.aw_mode = AWMode::kBackCalc;
    pi.Kt = 0.12;

    ok_or_die(inner.configure(pi), "inner.configure");
    ok_or_die(inner.start(), "inner.start");

    // Buffers
    std::vector<Scalar> u(1,0), y_pos(1,0), y_spd(1,0), r_pos(1,1.0);
    std::vector<Scalar> v_ref(1,0);

    Result u_inner{
        .u=std::span<Scalar>(u.data(),1),
        .health={}
    };

    PlantState ps_spd{
        .y=std::span<const Scalar>(y_spd.data(), 1),
        .xhat={},
        .t=0,
        .valid_bits=0x1
    };

    Setpoint sp_spd{
        .r=std::span<const Scalar>(v_ref.data(), 1),
        .preview_horizon_len=0
    };

    // CSV
    std::FILE* f = open_log_in_examples_pid("pid_cascade_step.csv");
    std::fprintf(f,"k,t_ms,u,v,x,v_ref,r\n");

    // Trapezoidal speed profile parameters
    const Scalar amax = 40.0;   // <= 50 (dv/dt = 50*u max with u in [-1,1])
    const Scalar vmax = 6.0;    // peak speed cap
    static Scalar v_cmd = 0.0;

    for (int k=0; k<1000; ++k){
        //  generate v_ref from position error (single reference r_pos) 
        Scalar e = r_pos[0] - y_pos[0];
        Scalar sgn = (e >= 0) ? Scalar(1) : Scalar(-1);
        Scalar vmag = std::abs(v_cmd);
        Scalar brake = (vmag*vmag) / (2.0*amax);

        if (std::abs(e) > brake)
            vmag = std::min(vmag + amax*dt_s, vmax);
        else
            vmag = std::max(vmag - amax*dt_s, Scalar(0));

        v_cmd = sgn * vmag;
        v_ref[0] = v_cmd;

        // inner speed loop (ref = v_ref) 
        ps_spd.t += dt;
        ok_or_die(inner.update({ps_spd, sp_spd}, u_inner), "inner.update");

        // plant
        y_spd[0] += Scalar(0.05) * u[0];
        y_pos[0] += y_spd[0] * Scalar(0.001);

        std::printf("%d, u=%.6f, v=%.6f, x=%.6f\n",
                    k,
                    static_cast<double>(u[0]),
                    static_cast<double>(y_spd[0]),
                    static_cast<double>(y_pos[0]));

        std::fprintf(f, "%d,%d,%.9f,%.9f,%.9f,%.9f,1.0\n",
                        k, k,
                        static_cast<double>(u[0]),
                        static_cast<double>(y_spd[0]),
                        static_cast<double>(y_pos[0]),
                        static_cast<double>(v_ref[0]));
    }
    std::fclose(f);
    return 0;
}
