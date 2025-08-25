#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>

// // Allocate chunks inside a fixed memory block, check usage and reset
namespace ictk{
    class MemoryArena{
        public:
            MemoryArena(void *base, std::size_t bytes) noexcept 
                : base_(static_cast<std::byte*>(base)), cap_(bytes) {}

            inline void *allocate(std::size_t bytes, std::size_t align = alignof(std::max_align_t)) noexcept{

                // // basic visibility
                if (!base_ || bytes ==0) return nullptr;
                if (align ==0 || (align & (align-1)) != 0) return nullptr;  // // must be pwoer of 2

                const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(base_);
                const std::uintptr_t curr = base + offset_;

                // // align up: (x+a-1) & ~(a-1)
                const std::uintptr_t aligned = (curr + (align - 1)) & ~(static_cast<std::uintptr_t>(align) - 1u);

                // distance from base (fits in size_t because cap_ is size_t)
                const std::size_t head = static_cast<std::size_t>(aligned - base);

                // // overflow -safe capacity check: head + bytes <= cap
                if (head > cap_ || bytes > cap_ - head) return nullptr;  // nullptr returned cause RT can't throw

                offset_ = head + bytes;

                return reinterpret_cast<void*>(aligned);
            }

            inline void reset() noexcept{
                offset_ = 0;
            }

            inline std::size_t capacity() const noexcept {
                return cap_;
            }

            inline std::size_t used() const noexcept{
                return offset_;
            }

        private:
            std::byte* base_{nullptr};
            std::size_t offset_{0};
            std::size_t cap_{0};
            
    };
} // namespace ictk
