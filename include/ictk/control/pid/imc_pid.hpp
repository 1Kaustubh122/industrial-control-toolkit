#pragma once
#include <cmath>
#include <algorithm>

#include "ictk/core/types.hpp"
#include "ictk/core/time.hpp"

// // IMC-PID synthesis for FOPDT (K, tau, 0). tustin-discrete-friendly outputs
// // Enforces Lmabda >= max(0, c.dt) to keep robustness and discretization sane.
namespace ictk::control::pid::imc{
    struct IMCInputs{
        Scalar K;       // // process gain
        Scalar tau;     // // time constant (>0)
        Scalar theta;   // // dead-time (>=0)
        Scalar lambda;  // // tuning (>0), overridden by max(0, c.dt) if smaller
        dt_ns dt;       // // controller dt (ns)
        Scalar c{4.0};  // // lambda floor multipler on dt
    };

    struct IMCOutputs{
        Scalar Kp, Kd, Ki, tau_f;
    };

    inline IMCOutputs synthesize(const IMCInputs& in) noexcept{
        const Scalar dt_s = static_cast<Scalar>(in.dt) * 1e-9;
        const Scalar lam = std::max({
            in.lambda,
            in.theta, 
            in.c*dt_s
        });

        /*
        Stndard IMC tuning for FOPDT
        Kp = (tau)/(K*(lam + theta))
        Kd = Kp * theta
        Ki = Kp / tau
        tau_f = min(tau, 0.1*(lam + theta))  <- conservative derivative filter
        */

        const Scalar denom = (lam + in.theta);
        IMCOutputs out{};
        out.Kp = (in.tau) / (in.K * (denom > 0 ? denom : Scalar(1)));
        out.Kd = out.Kp * in.theta;
        out.Ki = out.Kp / (in.tau > 0 ? in.tau : Scalar(1));
        out.tau_f = std::min(in.tau, Scalar(0.1) * denom);

        return out;
    }
    
} // namespace ictk::control::pid::imc
