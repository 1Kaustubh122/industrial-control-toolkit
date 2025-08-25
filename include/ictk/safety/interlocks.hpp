#pragma once
#include <cstdint>

// // allocation free bitmask gate -> actuation is allowed iff all required bits are ON.
namespace ictk::safety{
    // // bitmask interlocks: all required bits must be set to allow actuation
    class Interlocks{
        public:
            // // preventing implicit conversion int to InterLocks: Interlocks il(...) ; not Interlocks il = X; X = some numer 1,2,3...
            explicit Interlocks(
                std::uint64_t required_mask = ~0ull  // // all 64 bits set to 1
            ) noexcept : required_(required_mask) {}

            // // set which bits are mandatory
            void set_required(std::uint64_t mask) noexcept{
                required_ = mask;
            }

            // // | bitwise OR assignment -> turns on the given bit 
            void set(std::uint64_t mask) noexcept{
                bits_ |= mask;
            }

            // // & bitwise AND assignment | ~mask: bitwise NOT, turns 1->0 and 0->1, so &= ~mask turns off the given bit
            void clear(std::uint64_t mask) noexcept{
                bits_ &= ~mask;
            }

            // // set or clear by condition
            void write(std::uint64_t mask, bool on) noexcept{
                on ? set(mask) : clear(mask);
            }

            // // ensures all required bits are present 
            bool ok() const noexcept{
                return (bits_ & required_) == required_;
            }

            // // helper functions
            std::uint64_t bits() const noexcept{
                return bits_;
            }

            std::uint64_t required() const noexcept{
                return required_;
            }

        private:
            std::uint64_t bits_{0}, required_{0};
            /*
            bits: current state of all interlocks (1 = satisfied)
            required : which bits must be 1 to allow action
            */
    };
} // namespace ictk::safety
