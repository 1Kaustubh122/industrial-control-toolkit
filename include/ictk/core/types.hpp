#pragma once
#include <cstddef>
#include <type_traits>

namespace ictk{
    #if defined(ICTK_SCALAR_FLOAT)
        // // float for embedded/aarch64 tests
        using Scalar = float;
    #else
        using Scalar = double;
        static_assert(!std::is_same_v<Scalar, float>, "use double by default");
    #endif
    
    struct Dims{
        std::size_t ny{}, nu{}, nx{};
    };
    
    // // Choose Command mode across 4 different types
    enum class CommandMode : unsigned char {
        Primary=0,
        Residual=1,
        Shadow=2,
        Cooperative=3
    };

    static_assert(sizeof(CommandMode) == 1, "CommandMode must be 1 byte");


    inline constexpr CommandMode kPrimary =     CommandMode::Primary;     // Controller output goes straight to actuators
    inline constexpr CommandMode kResidual =    CommandMode::Residual;    // controllers adds corrections on top of the primary
    inline constexpr CommandMode kShadow =      CommandMode::Shadow;      // controllers run in parallel, output logged but not applied
    inline constexpr CommandMode kCooperative = CommandMode::Cooperative; // controller blends with others in a shared arbitration schema
    
} // namespace ictk
