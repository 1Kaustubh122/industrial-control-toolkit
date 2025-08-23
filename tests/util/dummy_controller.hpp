#pragma once

#include "ictk/core/controller.hpp"
#include "ictk/core/memory_arena.hpp"

namespace ictk_test{
    // // using the controller interface, status enums,types and the memory arena type and forbiding future inheritance.
    class DummyController final: public ictk::IController{
        public:
            DummyController() = default;  // // default ctor, no allocations, members keep their in class initializers
            
            // // init function 
            // // must inspect the returned status -> nodiscard
            [[nodiscard]] ictk::Status init(

                // // use the struct with sizes ny{}, nu{}, nx{}
                const ictk::Dims &dims,
                      // // time
                      ictk::dt_ns dt,
                      // // passed by ref to enable the pre-alloc. // // name comomented to silence warning (unused)
                      ictk::MemoryArena& /*arena*/,  // ignoring for now
                const ictk::Hooks &hooks = {}) noexcept override{

                    dims_ = dims;
                    dt_ = dt;
                    hooks_ = hooks;
                    started_ = false;
                    
                    return ictk::Status::kOK;
                } // // init function returns ictk::status::kOK
            
            // // Start function 
            [[nodiscard]] ictk::Status start() noexcept override{
                started_ = true;
                ticks_ = 0;
                return ictk::Status::kOK;
            }

            // // Stop function
            [[nodiscard]] ictk::Status stop() noexcept override{
                started_ = false;
                return ictk::Status::kOK;
            }

            // // reset function
            [[nodiscard]] ictk::Status reset() noexcept override{
                ticks_= 0;
                return ictk::Status::kOK;
            }

            // // update function    
            [[nodiscard]] ictk::Status update(const ictk::UpdateContext& ctx, ictk::Result &out) noexcept override{

                if (!started_) return ictk::Status::kNotReady;  // // if not started -> not ready

                // // writes zeros to first nu slots
                for (std::size_t i=0; i < dims_.nu; ++i){
                    out.u[i] = 0.0;
                }

                if (hooks_.pre_clamp){
                    hooks_.pre_clamp(out.u, hooks_.user);
                }

                (void)ctx; // // unused -> prevents warning 

                if (hooks_.post_arbitarte){
                    hooks_.post_arbitarte(out.u, out.u, hooks_.user);
                }

                // // not checking health, just bumping the internal counter to ensure runtime path is working
                ticks_++;
                return ictk::Status::kOK;
            }

            ictk::CommandMode mode() const noexcept override{
                return ictk::CommandMode::Primary;
            }
        
        private:
            ictk::Dims dims_{};
            ictk::dt_ns dt_{0};
            ictk::Hooks hooks_{};
            bool started_{false};
            std::uint64_t ticks_{0};
    };
} // namespace ictk_test