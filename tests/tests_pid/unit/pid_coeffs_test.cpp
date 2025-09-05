#include <cmath>
#include <array>
#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"


using namespace ictk;
using namespace ictk::control::pid;

/*
basic Single Input single Output (SISO) config block
kp = 2
ki = 1
kd = 0.5
Beta = 1
gamma = 0
tf = 0.01
u_bias = 0

*/
static PIDConfig siso_cfg_basic(){
    static Scalar Kp[]{2.0}, Kd[]{0.5}, Ki[]{1.0}, beta[]{1.0}, gamma[]{0.0};
    static Scalar tau_f[]{0.01}, u_bias[]{0.0};
    PIDConfig c{};
    c.Kp = {Kp, 1};
    c.Kd = {Kd, 1};
    c.Ki = {Ki, 1};

    c.beta = {beta, 1};
    c.gamma = {gamma, 1};
    c.tau_f = {tau_f, 1};
    c.u_ff_bias = {u_bias, 1};

    return c;
}

int main(){
    // SISO
    Dims d{.ny=1, .nu=1,.nx=0};
    
    // 1ms
    dt_ns dt = 1'000'000;

    // fixed 4KB of buffer
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    ictk::control::pid::PIDCore pid;

    // initialise (dims, dt_ns, MemoryArena, Hooks)
    [[maybe_unused]] auto st = pid.init(d, dt, arena, {});
    assert(st == Status::kOK);

    auto cfg = siso_cfg_basic();

    // Config 
    st = pid.configure(cfg);
    assert(st == Status::kOK);

    st = pid.start();
    assert(st == Status::kOK);

    // // DC = direct current = zero frequency component = a constant value over time 
    // // Derivative of constant is 0 -> so derivative term must ignore DC changes on setpoiny path when gamma = 0

    // Alloc on-element vectors for command (u), measurement (y), setpoint (r) -> all init to 0
    std::vector<Scalar> u(1, 0), y(1,0), r(1,0);

    [[maybe_unused]] Result res{
        .u = std::span<Scalar>(u.data(), 1), 
        .health = {}
    };

    [[maybe_unused]] PlantState ps{
        .y = std::span<Scalar>(y.data(), 1),
        .xhat = {},
        .t = 0,
        .valid_bits = 0x1
    };

    [[maybe_unused]] Setpoint sp{
        .r = std::span<const Scalar>(r.data(), 1),
        .preview_horizon_len = 0
    };

    /*
    first real tick at t = -1
    watchdog ignores the first dt because of it
    */
    // Tick 1: ready
    ps.t = dt;
    assert(pid.update({ps, sp}, res) == Status::kOK);
    const double u0 = u[0];

    // Tick 2: step in r, y unchanged, Only P & I should reach no D kick for y = 0
    // now 2 ms
    r[0] = 1.0;
    ps.t += dt;
    assert(pid.update({ps, sp}, res) == Status::kOK);
    
    // change in output 
    [[maybe_unused]] const double delta = u[0] - u0;

    // compute expected chagne from P + I only
    [[maybe_unused]] const double expected = 2.0 /*Kp*(Δr)*/ + 1.0 * 0.001 /*Ki*dt*e*/; // 2.001

    assert(std::abs(delta - expected) < 1e-6);      // no D contribution

    // ensure derivative didnt produce extra positive kick (bounded tightly)
    // with y = 0. derivative term uses only y; y unchanged => D=0 within numeric eps
    assert(res.health.aw_term_mag == 0.0); // health touched
    return 0;
}