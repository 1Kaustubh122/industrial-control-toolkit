#include <cmath>
#include <array>
#include <vector>
#include <cassert>

#include "ictk/all.hpp"
#include "ictk/control/pid/pid.hpp"


using namespace ictk;
using namespace ictk::control::pid;

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
    
}