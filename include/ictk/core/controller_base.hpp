#pragma once

#include <span>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <algorithm>

#include "ictk/core/health.hpp"       // // Struct ControllerHealth -> track saturation, watchdog misses, etc
#include "ictk/core/controller.hpp"   // // Inherit IController
#include "ictk/core/memory_arena.hpp" // // include MemoryArena -> some controller may allcoate scratch state in init

namespace ictk{

    struct SatStep{
        std::uint64_t hits{0};  // how many channels saturated per tick
        double pct{0.0};        // hits/nu
    };
    
    // // Reusable base that locks safety order and the lifecycle
    // // Derived classes implemnet compute_core(); safety steps are overridable no-ops by default 
    class ControllerBase : public IController{  // // IController: init, start, stop, reset, update, mode (primary, residual, shadow, cooperative) <- lifecycle
        public:
            ControllerBase() = default;
            ~ControllerBase() override = default;

            [[nodiscard]] Status init(
                const Dims& dims,
                dt_ns dt,
                MemoryArena& arena,
                const Hooks& hooks = {}) noexcept override{

                    // // Guard: can't controll with zero inputs or outputs; nx(state dimension) can still be zero eg PID doesnt have state vector
                    if (dims.nu == 0 || dims.ny ==0) return Status::kInvalidArg;
                    

                    // // store dimension, timesteps, hooks and reset lifecycle flags as not started yet.
                    dims_ = dims;     // .nu = no of outputs (actuator channels), .ny (no. of measurements), .nx(optional state estimate size)
                    dt_ = dt;         // fixed tick in ns
                    hooks_ = hooks;   // optional callbacks
                    arena_ = &arena;  // store arena pointer -> pre allocated memory pool

                    // Buffers
                    // // preallocating working buffers -> to preserve post pre clamp snapshot even for large nu
                    // snapshot of unsaturated command after core (raw commands): for anti windup
                    pre_buf_ = static_cast<Scalar*>(arena_->allocate(dims_.nu * sizeof(Scalar), alignof(Scalar)));  
                    // mutable comamnds 
                    work_buf_ = static_cast<Scalar*>(arena_->allocate(dims_.nu * sizeof(Scalar), alignof(Scalar))); 
                    // per stage to compute that stage's del mag
                    stage_buf_ = static_cast<Scalar*>(arena_->allocate(dims_.nu * sizeof(Scalar), alignof(Scalar)));

                    // // Guard: Alloc failure
                    if (!pre_buf_ || !work_buf_ || !stage_buf_) return Status::kNoMem;

                    started_ = false;
                    last_t_ = -1;  // Sentinel: no previous timestamp yet
                    health_ = {};  // zero out the ControllerHealth struct
                    
                    return Status::kOK;  // Success
            }

            [[nodiscard]] Status start() noexcept override{
                started_ = true;  // // flag that controller is now "armed"
                last_t_ = -1;     // // reset timestamp to sentinel, avoids accidental deadline miss

                // // Derived may need warmup; ensure no alloc after start
                return Status::kOK;
            }

            [[nodiscard]] Status stop() noexcept override{
                started_ = false;   // // disarmed -> signals that no more update(), and should run until start() is called back again
                return Status::kOK;
            }

            [[nodiscard]] Status reset() noexcept override{
                // // resetting the state without tearing down the entire controller
                last_t_ = -1;            // // wipe previous timestamp
                health_.clear_runtime(); // // clears per cycle counter -> saturations and rate limit hits
                return Status::kOK;
            }

            [[nodiscard]] Status update(const UpdateContext& ctx, Result& out) noexcept override{
                // // avoid stale values if a stage make no change
                health_.clear_runtime();

                // // Guard lifecycle -> no updates before start()
                if (!started_) return Status::kNotReady;

                // // Hot path shape checks, xhat may be absent; if present it must match nx
                if (out.u.size() != dims_.nu) return Status::kInvalidArg;
                if (ctx.plant.y.size() != dims_.ny) return Status::kInvalidArg;
                if (ctx.sp.r.size() != dims_.ny) return Status::kInvalidArg;
                if (!ctx.plant.xhat.empty() && ctx.plant.xhat.size() != dims_.nx) return Status::kInvalidArg;

                // // Watchdog; enforce fixed del t. any deviation increments the cumulative miss counter
                const auto d = ctx.plant.t - last_t_;
                if (last_t_ >= 0 && d != dt_) health_.deadline_miss_count += std::max<t_ns>(1, d / dt_) - 1;        // count how many periods were skipped, handle -d
                last_t_ = ctx.plant.t;

                // // 1- core control law
                Status st = compute_core(ctx, out.u); // algo controller (eg: PID)
                if (st != Status::kOK) return st;
                
                // // 2- pre output clamp hook
                if (hooks_.pre_clamp) hooks_.pre_clamp(out.u, hooks_.user);

                // snapshot after pre clamp, before safety -> taking unsafe commands
                std::memcpy(pre_buf_, out.u.data(), dims_.nu * sizeof(Scalar)); // copy to pre buf
                std::span<const Scalar> u_pre(pre_buf_, dims_.nu);

                // // 3- safery chain on a seprate work buffer
                std::memcpy(work_buf_, pre_buf_, dims_.nu * sizeof(Scalar));
                std::span<Scalar> u_work(work_buf_, dims_.nu);  // // Mutable vector -> without touching u_pre

                // SAT stage
                std::memcpy(stage_buf_, work_buf_, dims_.nu * sizeof(Scalar));
                // // clamp to actuators limits
                SatStep sat = apply_saturation({work_buf_, dims_.nu});
                double clamp_mag = 0.0; // change by saturation
                for (std::size_t i=0;i<dims_.nu;++i) clamp_mag = std::max(clamp_mag, std::abs(double(work_buf_[i]-stage_buf_[i])));
                health_.last_clamp_mag = clamp_mag; 

                // RATE stage
                std::memcpy(stage_buf_, work_buf_, dims_.nu * sizeof(Scalar));
                // // limiting the spikes
                std::uint64_t rate_hits = apply_rate_limit({work_buf_, dims_.nu});
                double rate_mag = 0.0;
                for (std::size_t i=0;i<dims_.nu;++i) rate_mag = std::max(rate_mag, std::abs(double(work_buf_[i]-stage_buf_[i])));
                health_.last_rate_clip_mag = rate_mag;

                // JERK stage
                std::memcpy(stage_buf_, work_buf_, dims_.nu * sizeof(Scalar));
                // // reducing mechanical shock
                std::uint64_t jerk_hits = apply_jerk_limit({work_buf_, dims_.nu});
                double jerk_mag = 0.0;
                for (std::size_t i=0;i<dims_.nu;++i) jerk_mag = std::max(jerk_mag, std::abs(double(work_buf_[i]-stage_buf_[i])));
                health_.last_jerk_clip_mag = jerk_mag;

                // // 4- Anti windup uses
                anti_windup_update(ctx, u_pre, u_work); // // inform integrators/observers about clamping so they don't wind up

                // // 5- Health wiring
                health_.saturation_pct = sat.pct;
                health_.rate_limit_hits += rate_hits;
                health_.jerk_limit_hits += jerk_hits;
                double aw_sum = 0.0;

                for (std::size_t i=0; i<dims_.nu; ++i) aw_sum += std::abs(static_cast<double>(u_work[i] - u_pre[i]));
                health_.aw_term_mag = aw_sum;

                // // copy safety result to output buffer
                std::memcpy(out.u.data(), work_buf_, dims_.nu * sizeof(Scalar));

                // // 6- post output arbitration sees the post-pre clamp sanpshot -> u_pre
                if (hooks_.post_arbitrate) hooks_.post_arbitrate(u_pre, out.u, hooks_.user);
                
                // // 7- attach health
                out.health = health_;

                return Status::kOK;
            }

            CommandMode mode() const noexcept override{ 
                return CommandMode::Primary; 
            }

        protected:
            // // required: implement core control law
            virtual Status compute_core(const UpdateContext& ctx, std::span<Scalar> u) noexcept = 0;

            // // overridables: defaults are no-ops; derived controllers can implement.
            virtual SatStep apply_saturation(std::span<Scalar> /*u*/) noexcept{ return {}; } 
            virtual std::uint64_t apply_rate_limit(std::span<Scalar> /*u*/) noexcept{ return 0; }
            virtual std::uint64_t apply_jerk_limit(std::span<Scalar> /*u*/) noexcept{ return 0; }

            virtual void anti_windup_update(const UpdateContext& /*ctx*/,
                                            [[maybe_unused]] std::span<const Scalar> u_unsat,
                                            [[maybe_unused]] std::span<const Scalar> u_sat) noexcept {}

            // // Helpers from derived classes (no ownership)
            const Dims &dims() const noexcept {
                return dims_;
            }
            dt_ns dt() const noexcept{
                return dt_;
            }
            MemoryArena& arena() noexcept{
                return *arena_;
            }
            ControllerHealth &health() noexcept{
                return health_;
            }

        private:
            Dims             dims_{};
            dt_ns            dt_{0};
            Hooks            hooks_{};
            MemoryArena*     arena_{nullptr};
            bool             started_{false};
            t_ns             last_t_{-1};
            ControllerHealth health_{};

            // // buffer to preserve post pre clamp snapshot 
            Scalar* pre_buf_{nullptr};
            Scalar* work_buf_{nullptr};
            Scalar* stage_buf_{nullptr};
           
    };
    
} // namespace ictk