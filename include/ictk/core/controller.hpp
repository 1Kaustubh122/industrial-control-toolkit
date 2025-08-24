#pragma once

#include <cstddef>
#include <span>
#include "ictk/core/types.hpp"
#include "ictk/core/status.hpp"
#include "ictk/core/time.hpp"
#include "ictk/core/result.hpp"
#include "ictk/core/update_context.hpp"

namespace ictk{
    class MemoryArena;

    // //                                u = command vector
    using PreClampHook = void(*)(std::span<Scalar> u,void* user);

    // //                            u_core = raw command from controller
    using PostArbHook = void(*)(std::span<const Scalar> u_core, std::span<Scalar> u_out, void* user);

    // //  Optional callbacks: Default No callback, optional, no state attached
    struct Hooks{
        PreClampHook pre_clamp{nullptr};
        PostArbHook post_arbitrate{nullptr};
        void* user{nullptr};
    };

    class IController{

        public:
            virtual ~IController() = default;

            // // Set up controller before use
            [[nodiscard]] virtual Status init(
                // // Params:
                // // dims tell the controller how many states, inputs, outputs, etc basically problem size are there.
                const Dims& dims,

                // // dt is basically timestamp in nanoseconds. Fixed rate at which update() will be called
                dt_ns dt,

                // // preallocated memory pool, avoid heap allocations during runtime
                MemoryArena& arena,

                // // Optional callbacks -> Struct declared 
                const Hooks& hooks = {}
            ) noexcept = 0;

            // // nodiscard -> forces caller to check return Status, which prevents ignoring errors like "init failed due to bad dims"
            // // noexcept -> Gurantee no exceptions critical for RT safety

            // // Activate controller after init -> Used before the first tick update
            [[nodiscard]] virtual Status start() noexcept = 0;

            // // Disables the controller 
            [[nodiscard]] virtual Status stop() noexcept = 0;

            // // Reset the controller state without re-initalizing everything
            [[nodiscard]] virtual Status reset() noexcept = 0;

            // // Called every cycle (every dt)
            [[nodiscard]] virtual Status update(
                const UpdateContext& ctx, Result& out
            ) noexcept = 0;

            // // Query what mode this controller is in
            virtual CommandMode mode() const noexcept = 0;
    };
} // namespace ictk
