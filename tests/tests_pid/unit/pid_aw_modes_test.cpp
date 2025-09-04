#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"
#include "ictk/safety/anti_windup.hpp"

using namespace ictk;
using ictk::safety::AWMode;
using namespace ictk::control::pid;

/*
Unit test fro anti-windup mode -> for PID controller
when the controller output is clamped -> the integrater would keep accumulating error forever -> cause overshoot
test:
    back calculation (bc) -> feed the difference between clamped vs unclamped output back into the integrator
    conditional integratin (ci) -> stop integrating when saturated and error is trying to push further into saturation
*/

// helper functions to add limits: declares saturation bounds {min, max} => +- 0.2
static void setup_limits(PIDConfig & c){
    static Scalar umin[]{-0.2}, umax[]{0.2};
    c.umin = {umin, 1};
    c.umax = {umax, 1};
}

int main(){
    // SISO => 1 input and 1 output
    Dims d{
        .ny = 1,
        .nu = 1,
        .nx = 0
    };

    dt_ns dt = 1'000'000; // 1ms tick period

    // 4KB
    alignas(64)std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    // pid_bc for back calc test
    PIDCore pid_bc;
    assert(pid_bc.init(d, dt, arena, {}) == Status::kOK);

    PIDConfig c{};

    // Setting constants value:

    static Scalar Kp[]{10.0}, Ki[]{5.0}, Kd[]{0.0}, beta[]{1.0}, gamma[]{0.0}, bias[]{0.0};

    c.Kp = {Kp, 1};
    c.Ki = {Ki, 1};
    c.Kd = {Kd, 1};
    c.beta = {beta, 1};
    c.gamma = {gamma,1};
    c.u_ff_bias = {bias, 1};

    setup_limits(c); // add +- 0.2 actuator clamp

    c.aw_mode = AWMode::kBackCalc;
    c.Kt = 0.5;  // back calc gain

    assert(pid_bc.configure(c) == Status::kOK);
    assert(pid_bc.start() == Status::kOK);

    // // configyre conditinoal integrator
    PIDCore pid_ci;

    assert(pid_ci.init(d, dt, arena, {}) == Status::kOK);

    c.aw_mode = AWMode::kConditional;
    c.Kt = 0.5;

    assert(pid_ci.configure(c) == Status::kOK);
    assert(pid_ci.start() == Status::kOK);

    /*
    u1-> output of bacl calc
    u2-> output of conditional PID
    y = 0 ->  plan measurement
    r=1 -> setpoint
    */
    std::vector<Scalar> u1(1,0), u2(1,0), y(1,0), r(1, 1.0);

    PlantState ps{
        .y = std::span<const Scalar>(y.data(), 1),
        .xhat = {},
        .t = 0,
        .valid_bits = 0x1
    };

    Setpoint sp{
        .r = std::span<const Scalar> (r.data(), 1),
        .preview_horizon_len = 0
    };

    // Two seperate result
    Result 
    res1{
        .u = std::span<Scalar>(u1.data(), 1),
        .health={}
    },
    res2{
        .u = std::span<Scalar>(u2.data(), 1),
        .health = {}
    };
    
    // Drive into saturation for 50 ticks (50 ms)
    for (int k=0; k<50; ++k){
        ps.t += dt;
        assert(pid_bc.update({ps, sp}, res1) == Status::kOK);
        assert(pid_ci.update({ps, sp}, res2) == Status::kOK);
    }

    /*
    Integral state should be different between nodes
    back calc integrated du term
    conditional gates e_aw
    can't read integ_ directly
    compare outputs should both be at clamp but AW trajectories differ (health.aw_term_mag same)
    */

    // both saturated at +0.2
    assert(std::abs(u1[0] - 0.2) < 1e-12);
    assert(std::abs(u2[0] - 0.2) < 1e-12);

    r[0] = 0.0;
    // back-calc should unwind I faster after demand is removed
    for (int k = 0; k < 50; ++k){
        ps.t += dt;
        (void)pid_bc.update({ps, sp}, res1);
        (void)pid_ci.update({ps, sp}, res2);
    }
    // BC should be closer to 0 than CI
    assert(std::abs(u1[0]) < std::abs(u2[0]));
    return 0;
}