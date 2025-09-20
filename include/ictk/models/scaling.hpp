#pragma once

#include <span>
#include <cstddef>
#include <type_traits>

#include "ictk/visibility.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/status.hpp"

namespace ictk::models{
    struct ICTK_API AffineScale{
        std::span<const Scalar> s; // scale factor
        std::span<const Scalar> b; // bias value

        [[nodiscard]] Status validate(std::size_t n) const noexcept{
            // // check: that spans are legal for a vector of lenth n
            const bool s_ok = (s.size() == 1) || (s.size() == n);
            const bool b_ok = (b.size() == 1) || (b.size() == n);
            return (s_ok && b_ok) ? Status::kOK : Status::kInvalidArg;
        }

        // x = real only span input values, y = writable span of output values
        [[nodiscard]] Status apply(std::span<const Scalar> x, std::span<Scalar> y) const noexcept{

            // // n = number of elements in the input vector
            const std::size_t n = x.size();

            // // Guard: validate n 
            if (y.size() != n) return Status::kInvalidArg;
            if (validate(n) != Status::kOK) return Status::kInvalidArg;

            // // flags to check if scale s and bias b are scalars (broadcast to all) reuse or per-element arrays
            const bool s1 = (s.size() == 1);
            const bool b1 = (b.size() == 1);

            for (std::size_t i=0; i<n; ++i){
                // // iterate over every element of the input vector
                const Scalar si = s1 ? s[0] : s[i];
                const Scalar bi = b1 ? b[0] : b[i];

                // // apply affine transform elementwise
                y[i] = si * x[i] + bi;
            }
            return Status::kOK;
        }

        // y = observed or scaled values (input to this fuction), x = recovered "original" values (output)
        [[nodiscard]] Status invert(std::span<const Scalar> y, std::span<Scalar> x) const noexcept{
            // size of input
            const std::size_t n = y.size();

            // // Guards: x and y are of same size
            if (x.size() != n) return Status::kInvalidArg;
            if (validate(n) != Status::kOK) return Status::kInvalidArg;

            // Broadcast flag: reuse or per element 
            const bool s1 = (s.size() == 1);
            const bool b1 = (b.size() == 1);

            // for each element
            for (std::size_t i=0; i<n; i++){
                // // pick appropriate scale and bias
                const Scalar si = s1 ? s[0] : s[i];
                const Scalar bi = b1 ? b[0] : b[i];

                // // DEBUG Guard against divide by zero
                #ifndef NDEBUG
                    #  if defined(_MSC_VER)
                        if (si == Scalar(0)) __debugbreak();   // debug
                    #  else
                        if (si == Scalar(0)) __builtin_trap(); // abort
                    #  endif
                #endif

                // // Compute the inverse
                x[i] = (si != Scalar(0)) ? (y[i] - bi) / si : Scalar(0);
            }
            return Status::kOK;
        }
    };
} // namespace ictk::models