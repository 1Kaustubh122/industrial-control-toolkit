#pragma once

#include <cstddef>
#include <cstring>

#include "ictk/visibility.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/status.hpp"
#include "ictk/core/memory_arena.hpp"

namespace ictk::models{
    class ICTK_API FifoDelay{
        public:
            FifoDelay() = default;

            // n_steps: requested delay (>=0). Buffer capacity becomes next power of two >= (n_step+1)
            FifoDelay(std::size_t n_steps, MemoryArena& arena) noexcept{
                (void)init(n_steps, arena);
            }

            /*
            init: set up the FIFO delay buffer
            Return: returns a Status code
            */
            [[nodiscard]] Status init(std::size_t n_steps, MemoryArena& arena) noexcept{
                n_step_ = n_steps; // store requested delay
                const std::size_t need = n_step_ + 1; // needs delay + 1 slots (+1 for incoming samples)
                cap_ = next_pow_2_(need);   // compute the buffer capacity as the next pow of 2 i.e >= need : make wrapping index cheap with & mask

                // guard: if input was 0 ret Invalid
                if (cap_ == 0) return Status::kInvalidArg;

                // // Request a block of memory for cap_ samples of type Scalar 
                data_ = static_cast<Scalar*>(arena.allocate(sizeof(Scalar) * cap_, alignof(Scalar)));

                // if alloc fails
                if (!data_) return Status::kNoMem;

                // // Initialise the buffer to zeros (all delay output starts at 0)
                std::memset(data_, 0, sizeof(Scalar)*cap_);

                // since capacity is pow of 2, mask will be binary, hence widx & mask eqv to widx % cap
                mask_ = cap_ - 1;

                widx_ = 0; // index starts at 0
                return Status::kOK;
            }

            // non-copyable, moveable only
            FifoDelay(const FifoDelay&) = delete;
            FifoDelay& operator = (const FifoDelay&) = delete;

            // // Move ctor
            FifoDelay(FifoDelay&& o) noexcept
                : data_(o.data_), cap_(o.cap_), mask_(o.mask_), widx_(o.widx_), n_step_(o.n_step_){
                    o.data_ = nullptr;
                    o.cap_ = o.mask_ = o.widx_ = o.n_step_ = 0;
            }

            // // Move assignment 
            FifoDelay& operator = (FifoDelay&& o) noexcept{
                if (this == &o) return *this;
                data_ = o.data_;
                cap_ = o.cap_;
                mask_ = o.mask_;
                widx_ = o.widx_;
                n_step_ = o.n_step_;
                o.data_ = nullptr;
                o.cap_ = o.mask_ = o.widx_ = o.n_step_ = 0;
                return *this;
            }

            // clear contents of buffer to 0; reset the write index
            void reset() noexcept{
                if (data_) std::memset(data_, 0, sizeof(Scalar) * cap_);
                widx_ = 0;
            }

            // // returns how many steps of delay this obj is configured for
            std::size_t delay() const noexcept{
                return n_step_;
            }

            // Push x, return oldest (delayed) sample
            Scalar push(Scalar x) noexcept{
                // oldest element (protects: underflow)
                const std::size_t ridx = (widx_ + cap_ - n_step_) & mask_;

                // read delayed output
                const Scalar y = data_[ridx];

                // write new output at the wrapped write index
                data_[widx_ & mask_] = x;
                ++widx_;    // Wraps naturally by mask on use
                return y;
            }

            // Peek kth-oldest where k in [0, n_step_); k=0 is oldest, k=nstep-1 newest
            Scalar peek(std::size_t k) const noexcept{
                if (k>=n_step_){
                    #if defined(NDEBUG)
                        // if asked to peek outside the valid window -> BUG (In Debug mode)
                        k = (n_step_ == 0) ? 0 : (n_step_ - 1);
                    #else
                        // IN RELEASE MODE ->
                        #if defined(_MSC_VER)
                            __debugbreak(); // trigger a breakpoint
                        #else
                            __builtin_trap(); // abort
                        #endif
                    #endif
                }

                /*
                widx + cap_  protection from underflow
                Base of delay window : widx - n_step_ (oldest sample waiting)
                +k moves k steps forward inside the window
                then mask (since cap is pow of 2)
                */
                const std::size_t idx = (widx_ + cap_ - n_step_ + k) & mask_; 

                return data_[idx];
            }
    
        private:
            // find next pow of 2 (must be greater than or equal to x)
            static std::size_t next_pow_2_(std::size_t x) noexcept{

                // invalid capacity
                if (x==0) return 0;

                // ensure that if x is already a power of two then don't round up
                --x;

                /*
                Each one takes the current value of x and "OR" it with a shited value of itself
                */
                x |= x>>1;
                x |= x>>2;
                x |= x>>4;
                x |= x>>8;
                x |= x>>16;

                #if INTPTR_MAX == INT64_MAX
                    x |= x>>32; 
                #endif

                return x + 1;
            }

            // pointer to ring buffer memory holding samples
            Scalar* data_{nullptr}; 
            
            // capacity of buffer in elements
            std::size_t cap_{0};
            
            // used to wrap indicies fast
            std::size_t mask_{0};

            // monotonic write counter
            std::size_t widx_{0};

            // requested delay length in samples
            std::size_t n_step_{0};
    };
    
} // namespace ictk::models