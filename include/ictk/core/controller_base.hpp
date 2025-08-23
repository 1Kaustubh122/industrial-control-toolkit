#pragma once

#include <span>
#include <cstddef>

#include "ictk/core/controller.hpp"
#include "ictk/core/health.hpp"
#include "ictk/core/memory_arena.hpp"

namespace ictk{
    // // Reusable base that locks safety order and the lifecycle
    // // Derived classes implemnet compute_core(); safety setps are overridable no-ops by default 

    class ControllerBase : public IController{
        public:
            ControllerBase() = default;
            ~ControllerBase() override = default;

            [[nodiscard]] Status init(
                const Dims& dims,
                dt_ns dt,
                MemoryArena& arena,
                const Hooks& hooks = {}) noexcept override{

                    if (dims.nu == 0 || dims.ny ==0){
                        return Status::kInvalidArg;
                    }

                    dims_ = dims;
                    dt_ = dt;
                    hooks_ = hooks;
                    started_ = false;
                    last_t_ = -1;
                    health_ = {};
                    
                    return Status::kOK;
            }

            [[nodiscard]] Status update(const UpdateContext& ctx, Result& out) noexcept override{
                if (!started_) return Status::kNotReady;
                if (out.u.size() != dims_.nu) return Status::kInvalidArg;
                if (ctx.plant.y.size() != dims_.ny) return Status::kInvalidArg;
                if (!ctx.plant.xhat.empty() && ctx.plant.xhat.size() != dims_.nx) return Status::kInvalidArg;

                
            }

        protected:
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
