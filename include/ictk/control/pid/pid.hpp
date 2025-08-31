#pragma once

#include <span>
#include <cstddef>
#include <cstdint>
#include <cassert>
#include <optional>
#include <algorithm>

#include "ictk/core/memory_arena.hpp"
#include "ictk/core/controller_base.hpp"

#include "ictk/safety/saturation.hpp"
#include "ictk/safety/rate_limit.hpp"
#include "ictk/safety/jerk_limit.hpp"
#include "ictk/safety/watchdog.hpp"
#include "ictk/safety/fallback.hpp"
#include "ictk/safety/anti_windup.hpp"

namespace ictk::control::pid{

    // // Anti Windup Mode
    enum class AwMode: std::uint8_t {
        kBackCalc,      // u_sat - u_unsat
        kConditional,   // stop integrating when saturated and error has wrong sign
        kOff            // no AW -> Only for testing
    };

    // picewise-linear gain scheduling 
    struct ScheduleConfig{
        // bp => strictly increasing breakpoints, *_tab => same length as bp, pre breakpoints value || interpolate at runtimeS
        std::span<const Scalar> bp, kp_tab, ki_tab, kd_tab, beta_tab, gamma_tab;
    };


    struct PIDConfig{
        // // gains
        std::span<const Scalar> Kp, Kd, Ki;
        // // setpoint weights 
        std::span<const Scalar> beta, gamma;
        // // derivative filter
        std::span<const Scalar> tau_f, N;
        // // feed forward bias
        std::span<const Scalar> u_ff_bias;
        // // safety => umin, umax -> saturation; du_max, ddu_max -> rate, jerk limits/ 
        std::span<const Scalar> umin, umax, du_max, ddu_max;

        // anti windup
        AwMode aw_mode{
            AwMode::kBackCalc
        };


        Scalar Kt{0.0}; // back calc gain

        // watchdog
        std::uint32_t miss_threshold{0}; 
        dt_ns watchdog_slack{0};

        // fallback targ 
        std::span<const Scalar> safe_u;

        // ramp speed
        Scalar fb_ramp_rate{0.0};

        // breakpoint and tables
        ScheduleConfig sched{};
    };


    class PIDCore final: public ControllerBase{
        public:
            PIDCore() = default;

            [[nodiscard]] Status init(const Dims& d, dt_ns dt_ns_i, MemoryArena& a, const Hooks& h = {}) noexcept override{
                // // Guard: d.nu == 0 -> No output; d.ny == 0-> no measurement; d.nx -> state size (unused in PID)
                if (d.nu == 0 || d.ny == 0 || d.nu != d.ny) return Status::kInvalidArg;
                return ControllerBase::init(d, dt_ns_i, a, h);
            }

            [[nodiscard]] Status configure(const PIDConfig& cfg) noexcept{
                const std::size_t nu = dims().nu;
                const Scalar dt_s = static_cast<Scalar>(dt()) * 1e-9;
                
                // gains
                kp_ = alloc(nu); // proportional gains
                kd_ = alloc(nu); // derivative gains
                ki_ = alloc(nu); // integral gains

                // setpoint weights -> for PIDF
                /*
                if B = 1: e = r - y
                if B < 1: reduce proportional action on step changes setpoint (less overshoot)
                gamma: weight of derivative on setpoint
                */
                beta_ = alloc(nu);
                gamma_ = alloc(nu);

                // feed forward bias
                uff_ = alloc(nu);
                
                // integrator state
                integ_=alloc(nu);

                // last samples for difference operations
                y_prev_=alloc(nu); // measurement
                r_prev_=alloc(nu); // reference

                // filtered derivatives or y and r
                dyf_=alloc(nu); // filtered derivative for reference y
                drf_=alloc(nu); // filtered derivative for reference r

                // filter coefficients for 1st order derivative filter
                a1_=alloc(nu);
                b_=alloc(nu);

                // scratch
                tmp_=alloc(nu);

                // cached Ki * dt_s per channel for fast integration
                kidt_=alloc(nu);    

                // // If any allocs fails: return kNoMem
                if (!kp_ || !ki_ || !kd_ ||!beta_ ||!gamma_ ||!uff_ ||!integ_ ||!y_prev_||!r_prev_||!dyf_||!drf_||!a1_||!b_ || !tmp_||!kidt_) return Status::kNoMem;

                // // fill_array(destination, source, default)
                fill_array(kp_, cfg.Kp, 0); 
                fill_array(kd_, cfg.Kd, 0);
                fill_array(ki_, cfg.Ki, 0); 

                fill_array(beta_, cfg.beta, 1); 
                fill_array(gamma_, cfg.gamma, 0); 
                fill_array(uff_, cfg.u_ff_bias, 0);

                for (std::size_t i=0; i<nu; ++i){
                    // // derivative filter time constant: control the derivative term
                    Scalar tf = 0;
                    const bool has_tf = (i < cfg.tau_f.size());
                    const bool has_N = (i < cfg.N.size());

                    if (has_tf) tf = cfg.tau_f[i];
                    else if (has_N){
                        const Scalar N = cfg.N[i];
                        tf = (N > 0) ? (Scalar(1)/N) : Scalar(0);
                    }

                    // // intermediate values in the bilinear (Tustin) discretization of the filter
                    const Scalar den = Scalar(2) * tf + dt_s;
                    const Scalar num = Scalar(2) * tf - dt_s;

                    // filter coefficient feedback
                    a1_[i] = (den > 0) ? (num / den) : Scalar(0);
                    // filter coefficient feed forward  
                    b_[i] = (den > 0) ? (Scalar(2) / den) : Scalar(0);

                    // β, γ ranges                      // default                  //Single Channel    // default values of beta and gamma
                    const Scalar b = (i<cfg.beta.size()? cfg.beta[i] : (cfg.beta.size()==1? cfg.beta[0] : Scalar(1)));
                    const Scalar g = (i<cfg.gamma.size()?cfg.gamma[i] : (cfg.gamma.size()==1?cfg.gamma[0] : Scalar(0)));

                    if (!(b>=0 && b<=1)) return Status::kInvalidArg;
                    if (g != Scalar(0))  return Status::kInvalidArg; 
                }

                // // Safety blocks
                // Saturation -> actuator limits
                if (!cfg.umin.empty() || !cfg.umax.empty()) sat_.emplace(cfg.umin, cfg.umax);

                // rate limit
                if (!cfg.du_max.empty()) rl_.emplace(cfg.du_max, dt(), arena(), nu);

                // jerk limit
                if (!cfg.ddu_max.empty()){
                    const Scalar rmax = (!cfg.du_max.empty() ? cfg.du_max[0] : Scalar(0)); // <- need to add in the docs
                    const Scalar jmax = cfg.ddu_max[0];
                    jl_.emplace(rmax, jmax,dt(), arena(), nu);
                }

                // validate safety allocation
                if ((rl_ && !rl_ -> valid()) || (jl_ && !jl_ -> valid())) return Status::kNoMem;

                aw_mode_ = cfg.aw_mode;
                Kt_ = cfg.Kt;

                // watchdog 
                if (cfg.miss_threshold > 0) wd_.emplace(dt(), cfg.miss_threshold, cfg.watchdog_slack);

                // fallback
                if (!cfg.safe_u.empty() && cfg.fb_ramp_rate > 0) fb_.emplace(cfg.safe_u, cfg.fb_ramp_rate, dt(), arena(), nu);

                // // Scheduling setup
                if(!cfg.sched.bp.empty()){
                    const std::size_t B = cfg.sched.bp.size();
                    if (B < 2) return Status::kInvalidArg; // Require at least 2 breakpoints, not possible to interpolate from 1 point

                    // // check strictly increasing breakpoints
                    for (std::size_t i=1; i<B; ++i) if (!(cfg.sched.bp[i] > cfg.sched.bp[i-1])) return Status::kInvalidArg; 

                    // // each table must align 1:1 with bp, mismatched lengths means undefined mapping
                    if (cfg.sched.kp_tab.size()!= B || cfg.sched.ki_tab.size()!= B || cfg.sched.kd_tab.size() != B ||
                        cfg.sched.beta_tab.size() !=B || cfg.sched.gamma_tab.size()!= B) return Status::kInvalidArg;
                    
                    // store the schedule config 
                    sched_ = cfg.sched;
                }

                for (std::size_t i=0; i<nu; ++i){ 
                    integ_[i] = 0;              // integrator sate reset
                    dyf_[i] = 0;                // filtered d(y)
                    drf_[i] = 0;                // filtered d(r) if used
                    y_prev_[i] = 0;             // last measurement
                    r_prev_[i] = 0;             // last setpoit
                    kidt_[i] = ki_[i] * dt_s;   // cache ki*dt per channel
                }

                dt_s_ = dt_s;
                return Status::kOK;
            }

            [[nodiscard]] Status start() noexcept override{
                return ControllerBase::start(); // started = true -> returns Status::kOk
            }

            // Bumpless transfer
            void align_bumpless(std::span<const Scalar> u_hold, std::span<const Scalar> r0, std::span<const Scalar> y0) noexcept{
                const std::size_t n = dims().nu;
                const std::size_t m = std::min({
                    u_hold.size(),
                    r0.size(),
                    y0.size()
                });

                for (std::size_t i=0; i<m; ++i){
                    const Scalar ydot0 = dyf_[i];   // simple consistent init; To DO: Back solve (reminder: check during gold cart impl)
                    const Scalar e0 = beta_[i] * r0[i] - y0[i];
                    integ_[i] = u_hold[i] - (kp_[i] * e0 - kd_[i]* ydot0 + uff_[i]);
                    y_prev_[i] = y0[i];
                    r_prev_[i] = r0[i];
                }
            }
            
        protected:
            [[nodiscard]] Status compute_core(const UpdateContext& ctx, std::span<Scalar> u) noexcept{
                // // no of outputs/channels
                const std::size_t n = dims().nu;

                if (n > 64) return Status::kInvalidArg;  // cannot validate >64 bits with a u64

                // check al n channels valid this tick
                const std::uint64_t mask = (n == 64) ? ~0ull : ((1ull << n) - 1ull);
                if ((ctx.plant.valid_bits & mask) != mask) return Status::kPreconditionFail;
                
                // fallback latch
                if (wd_){
                    if (wd_->tick(ctx.plant.t)) health().fallback_active = true;
                }
                
                // scheduling over y[0] -> to do: put it PID.md

                // // stage scheduled gains
                Scalar sKp = 0, sKi = 0, sKd = 0, sB=1, sG= 0;

                // def: no sched used
                bool use_sched = false;

                // if scheduled configured -> find upper index
                if (!sched_.bp.empty()){
                    const Scalar var = ctx.plant.y[0];
                    const std::size_t B = sched_.bp.size();
                    auto it = std::upper_bound(sched_.bp.begin(), sched_.bp.end(), var);
                    std::size_t i1 = std::clamp<std::size_t>(static_cast<std::size_t>(it - sched_.bp.begin()), 1, B - 1);


                    // parametic weight for interpolation between bp[i0] and bp[i1]
                    const std::size_t i0 = i1 - 1;
                    const Scalar x0 = sched_.bp[i0], x1 = sched_.bp[i1];
                    const Scalar t = (x1!=x0) ? (std::clamp(var, x0, x1) - x0) / (x1-x0) : Scalar(0);

                    // liner interpolate gains and weights
                    sKp = lerp(sched_.kp_tab[i0], sched_.kp_tab[i1], t);
                    sKd = lerp(sched_.kd_tab[i0], sched_.kd_tab[i1], t);
                    sKi = lerp(sched_.ki_tab[i0], sched_.ki_tab[i1], t);

                    sB = lerp(sched_.beta_tab[i0], sched_.beta_tab[i1], t);
                    sG = lerp(sched_.gamma_tab[i0], sched_.gamma_tab[i1], t);
                    use_sched = true;
                }

                // / Per channel PID form
                for (std::size_t i=0; i<n; ++i){
                    // pick scheduled gains if enabled, else per channel
                    const Scalar KP = use_sched ? sKp : kp_[i];
                    const Scalar KD = use_sched ? sKd : kd_[i];
                    const Scalar KI = use_sched ? sKi : ki_[i];

                    const Scalar B = use_sched ? sB : beta_[i];
                    const Scalar G = use_sched ? sG : gamma_[i];

                    const Scalar yk = ctx.plant.y[i];
                    const Scalar rk = ctx.sp.r[i];

                    const Scalar e = B * rk - yk;

                    const Scalar dy = b_[i] * (yk - y_prev_[i]) + a1_[i] * dyf_[i];
                    const Scalar dr = b_[i] * (rk - r_prev_[i]) + a1_[i] * drf_[i];

                    dyf_[i] = dy;
                    drf_[i] = dr;
                    y_prev_[i] = yk;
                    r_prev_[i] = rk;

                    const Scalar P = KP * e;
                    const Scalar D = (-KD * dy);
                    u[i] = P + integ_[i] + D + uff_[i];

                    tmp_[i] = e;
                    kidt_[i] = KI * dt_s_; 
                }
                return Status::kOK;
            }

            SatStep apply_saturation(std::span<Scalar> u) noexcept override{
                if (!sat_) return {};
                auto rep = sat_->apply(u);
                return {rep.hits, rep.saturation_pct};
            }

            std::uint64_t apply_rate_limit(std::span<Scalar> u) noexcept override{
                return (rl_? rl_->apply(u) : 0);
            }

            std::uint64_t apply_jerk_limit(std::span<Scalar> u) noexcept override{
                return (jl_? jl_->apply(u) : 0);
            }

            void anti_windup_update(
                const UpdateContext&, std::span<const Scalar> u_unsat,
                std::span<const Scalar> u_sat
            ) noexcept override{
                const std::size_t n = dims().nu;
                for (std::size_t i = 0; i < n; ++i) {
                    const bool saturated = (u_unsat[i] != u_sat[i]);
                    const Scalar e = tmp_[i]; // error stored in compute_core

                    switch(aw_mode_) {
                        case AwMode::kOff:
                            // no anti-windup, no integrator change
                            break;

                        case AwMode::kConditional:
                            if (!saturated) {
                                integ_[i] += kidt_[i] * e;
                            }
                            break;

                        case AwMode::kBackCalc:
                            integ_[i] += kidt_[i] * e;                 // normal I
                            integ_[i] += Kt_ * (u_sat[i] - u_unsat[i]);// back-calc term
                            break;
                    }
                }
            }


        private:
            Scalar* alloc(std::size_t n) noexcept{
                return static_cast<Scalar*>(arena().allocate(n*sizeof(Scalar), alignof(Scalar)));
            }
            static inline Scalar lerp(Scalar a, Scalar b, Scalar t) noexcept{
                return a + (b-a) * t;
            }

            /*
            solves user config -> PIDConfig -> provides gains and weights as span const scalar 
            */
            void fill_array(Scalar* dst, std::span<const Scalar> src, Scalar def) noexcept{
                const std::size_t n = dims().nu;
                if (src.size() == 1){
                    // brodcase one value to all channels
                    for (std::size_t i=0; i<n; ++i) dst[i] = src[0];
                    return;
                }
                if (!src.empty()){
                    const std::size_t m = std::min(n, src.size());
                    // copy size upto the provided value
                    for (std::size_t i=0; i<m; ++i) dst[i]= src[i];
                    // fill remaining values with default
                    for (std::size_t i=m; i<n; ++i) dst[i] = def;
                    return;
                }
                // if empty, fill all with def value
                for (std::size_t i=0; i<n; ++i) dst[i]=def;
            }

            Scalar *kp_{}, *kd_{}, *ki_{}, *beta_{}, *gamma_{}, *uff_{};
            Scalar *integ_{}, *y_prev_{}, *r_prev_{}, *dyf_{}, *drf_{}, *a1_{}, *b_{};
            Scalar *tmp_{}, *kidt_{};
            Scalar dt_s_{0};

            std::optional<safety::Saturation> sat_;
            std::optional<safety::RateLimiter> rl_;
            std::optional<safety::JerkLimiter> jl_;
            std::optional<safety::Watchdog> wd_;
            std::optional<safety::FallbackPolicy> fb_;

            AwMode aw_mode_{
                AwMode::kBackCalc
            };

            Scalar Kt_{0};
            ScheduleConfig sched_{};
    };
    
    using PController = PIDCore;    // // with ki=kd=0
    using PIController = PIDCore;   // // with kd=0
    using PDController = PIDCore;   // // with ki=0
    using PIDController = PIDCore;  // // standard
    using PIDFController = PIDCore; // // with tf>0 or N>0
    
} // namespace ictk::control::pid
