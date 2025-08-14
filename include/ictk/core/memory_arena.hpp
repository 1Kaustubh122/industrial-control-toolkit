#pragma once
#include <cstddef>
#include <cstdint>

// // Allocate chunks inside a fixed memory block, check usage and reset
namespace ictk{
    class MemoryArena{
        public:
            MemoryArena(void *base, std::size_t bytes) noexcept : base_(static_cast<std::byte*>(base)), cap_(bytes) {}

            void *allocate(std::size_t bytes, std::size_t align = alignof(std::max_align_t)) noexcept;

            void reset() noexcept{
                offset_ = 0;
            }

            std::size_t capacity() const noexcept {
                return cap_;
            }

            std::size_t used() const noexcept{
                return offset_;
            }

        private:
            std::byte* base_{nullptr};
            std::size_t offset_{0};
            std::size_t cap_{0};
            
    };
} // namespace ictk
