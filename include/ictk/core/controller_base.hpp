#pragma once

#include <span>
#include <cstddef>

#include "ictk/core/controller.hpp"   // // Inherit IController
#include "ictk/core/health.hpp"       // // Struct ControllerHealth -> track saturation, watchdog misses, etc
#include "ictk/core/memory_arena.hpp" // // include MemoryArena -> some controller may allcoate scratch state in init

namespace ictk{
    // // Reusable base that locks safety order and the lifecycle
    // // Derived classes implemnet compute_core(); safety steps are overridable no-ops by default 

    class ControllerBase : public IController{
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
                    dims_ = dims;
                    dt_ = dt;
                    hooks_ = hooks;
                    arena_ = &arena;  // store arena pointer
                    started_ = false;
                    last_t_ = -1;  // no previous timestamp yet
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
                // // Guard lifecycle -> no updates before start()
                if (!started_) return Status::kNotReady;

                // // Hot path shape checks, xhat may be absent; if present it must match nx
                if (out.u.size() != dims_.nu) return Status::kInvalidArg;
                if (ctx.plant.y.size() != dims_.ny) return Status::kInvalidArg;
                if (ctx.sp.r.size() != dims_.ny) return Status::kInvalidArg;
                if (!ctx.plant.xhat.empty() && ctx.plant.xhat.size() != dims_.nx) return Status::kInvalidArg;

                // // Watchdog; enforce fixed del t. any deviation increments the cumulative miss counter
                if (last_t_ >= 0 && (ctx.plant.t - last_t_) != dt_){
                    ++ health_.deadline_miss_count;  // // refinment later -> count multiples
                }
                last_t_ = ctx.plant.t;

                Status st = compute_core(ctx, out.u);
                if (st != Status::kOK) return st;
                
                if (hooks_.pre_clamp) hooks_.pre_clamp(out.u, hooks_.user);

                apply_saturation(out.u);        // // clamp to hard min/max update
                apply_rate_limit(out.u);        // // clamp dv/dt
                apply_jerk_limit(out.u);        // // clamp acceleration charge
                anti_windup_update(ctx, out.u); // // inform integrators/observers about clamping so they don't wind up

                if (hooks_.post_arbitrate){
                    // // small nu stack snapshot
                    constexpr std::size_t kMaxSnap = 16;
                    Scalar snap[kMaxSnap];
                    std::span<const Scalar> u_core_view = out.u;  

                    if (dims_.nu <= kMaxSnap){
                        for (std::size_t i = 0; i < dims_.nu; ++i) snap[i] = out.u[i];
                        u_core_view = {snap, dims_.nu}; // a read only snapshot
                    }
                    hooks_.post_arbitrate(u_core_view, out.u, hooks_.user);
                }

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
            virtual void apply_saturation(std::span<Scalar> /*u*/) noexcept{}
            virtual void apply_rate_limit(std::span<Scalar> /*u*/) noexcept{}
            virtual void apply_jerk_limit(std::span<Scalar> /*u*/) noexcept{}
            virtual void anti_windup_update(const UpdateContext& /*ctx*/,
                                            std::span<const Scalar> /*u*/) noexcept {}

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
           
    };
    
} // namespace ictk