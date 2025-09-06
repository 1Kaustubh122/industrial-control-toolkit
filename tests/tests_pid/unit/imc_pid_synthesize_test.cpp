#include <cmath>
#include <cassert>

#include "ictk/control/pid/imc_pid.hpp"

using namespace ictk;
using namespace ictk::control::pid::imc;

int main(){
    IMCInputs in{
    .K=2.0,         // process gain
    .tau=5.0,       // time constant 
    .theta=1.0,     // delay
    .lambda=2.0,    // IMC filter time
    .dt=1'000'000,  // 1ms
    .c=4.0          // Discrete floor factor
    };
        
    auto o1 = synthesize(in);

    // Sanity: signs and scaling
    assert(o1.Kp > 0 && o1.Ki > 0 && o1.tau_f > 0);

    // Larger lambda → smaller Kp and Ki (monotone)
    in.lambda = 10.0;

    auto o2 = synthesize(in);
    assert(o2.Kp < o1.Kp);
    assert(o2.Ki < o1.Ki);

    // Kd proportional to theta
    in.lambda = 2.0;
    in.theta = 2.0;
    auto o3 = synthesize(in);
    assert(o3.Kd > o1.Kd);

    // tau_f bounded by tau and proportional to (lambda+theta)
    assert(o1.tau_f <= in.tau + 1e-12);

    // Lambda floor: set very small lambda; should floor to max(theta, c*dt)
    IMCInputs in2{
        .K=2.0, .tau=5.0, .theta=1.0, .lambda=1e-6, .dt=1'000'000, .c=4.0
    };

    auto o4 = synthesize(in2); // effectively uses λ≈max(1.0, 0.004)=1.0

    // Compare against explicit λ=1.0 for parity
    IMCInputs in3{
        .K=2.0, .tau=5.0, .theta=1.0, .lambda=1.0, .dt=1'000'000, .c=4.0 
    };

    auto o5 = synthesize(in3);
    assert(std::abs(o4.Kp - o5.Kp) < 1e-12);
    assert(std::abs(o4.Ki - o5.Ki) < 1e-12);
    assert(std::abs(o4.Kd - o5.Kd) < 1e-12);
    assert(std::abs(o4.tau_f - o5.tau_f) < 1e-12);

    return 0;
}