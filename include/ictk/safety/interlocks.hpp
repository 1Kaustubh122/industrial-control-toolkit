#pragma once
#include <cstdint>

namespace ictk::safety{
    // // bitmask interlocks: all required bits must be set to allow actuation
    class Interlocks{
        public:
            explicit Interlocks(
                std::uint64_t required_mask = ~0ull
            ) noexcept : required_(required_mask) {}

            void set_required(std::uint64_t mask) noexcept{
                required_ = mask;
            }

            void set(std::uint64_t mask) noexcept{
                bits_ |= mask;
            }

            void clear(std::uint64_t mask) noexcept{
                bits_ &= ~mask;
            }

            void write(std::uint64_t mask, bool on) noexcept{
                on ? set(mask) : clear(mask);
            }

            bool ok() const noexcept{
                return (bits_ & required_) == required_;
            }

            std::uint64_t bits() const noexcept{
                return bits_;
            }

            std::uint64_t required() const noexcept{
                return required_;
            }

        private:
            std::uint64_t bits_{0}, required_{0};
    };
} // namespace ictk::safety
