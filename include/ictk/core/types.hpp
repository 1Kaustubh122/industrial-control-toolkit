#pragma once
#include <cstddef>

// // Choose Command mode across 4 different types
namespace ictk{
    using Scalar = double;

    struct Dims{
        std:: size_t ny{}, nu{}, nx{};
    };

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
