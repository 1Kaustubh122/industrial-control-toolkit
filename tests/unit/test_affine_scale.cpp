// tests/unit/test_affine_scale.cpp
#include <array>
#include <cstddef>

#include "ictk/core/types.hpp"
#include "ictk/core/status.hpp"
#include "ictk/models/scaling.hpp"
#include "util/alloc_interposer.hpp"

using namespace ictk;
using namespace ictk::models;

template <std::size_t N>
static bool arrays_eq(const std::array<Scalar,N>& a, const std::array<Scalar,N>& b) {
    for (std::size_t i = 0; i < N; ++i) if (a[i] != b[i]) return false;
    return true;
}

int main() {
    // Case 1: scalar s, scalar b - round trip
    {
        const std::array<Scalar,3> xin{1,2,3};
        std::array<Scalar,3> y{}, xrec{};
        const Scalar s_data[1]{Scalar(2)};
        const Scalar b_data[1]{Scalar(1)};
        AffineScale A{ std::span<const Scalar>(s_data,1),
                       std::span<const Scalar>(b_data,1) };

        ictk_test::reset_alloc_stats();
        if (A.apply(xin, y) != Status::kOK) return 1;
        const std::array<Scalar,3> y_ref{3,5,7};
        if (!arrays_eq(y, y_ref)) return 2;
        if (A.invert(y, xrec) != Status::kOK) return 3;
        if (!arrays_eq(xrec, xin)) return 4;
        if (ictk_test::new_count() || ictk_test::new_aligned_count()
            || ictk_test::delete_count() || ictk_test::delete_aligned_count()) return 5;
    }

    // Case 2: vector s, scalar b
    {
        const std::array<Scalar,3> xin{1,1,1};
        std::array<Scalar,3> y{};
        const Scalar s_data[3]{Scalar(1),Scalar(2),Scalar(3)};
        const Scalar b_data[1]{Scalar(1)};
        AffineScale A{ std::span<const Scalar>(s_data,3),
                       std::span<const Scalar>(b_data,1) };
        if (A.apply(xin, y) != Status::kOK) return 6;
        const std::array<Scalar,3> y_ref{2,3,4};
        if (!arrays_eq(y, y_ref)) return 7;
    }

    // Case 3: scalar s, vector b
    {
        const std::array<Scalar,3> xin{1,1,1};
        std::array<Scalar,3> y{};
        const Scalar s_data[1]{Scalar(2)};
        const Scalar b_data[3]{Scalar(1),Scalar(2),Scalar(3)};
        AffineScale A{ std::span<const Scalar>(s_data,1),
                       std::span<const Scalar>(b_data,3) };
        if (A.apply(xin, y) != Status::kOK) return 8;
        const std::array<Scalar,3> y_ref{3,4,5};
        if (!arrays_eq(y, y_ref)) return 9;
    }

    // Case 4: vector s, vector b - round trip
    {
        const std::array<Scalar,2> xin{1,2};
        std::array<Scalar,2> y{}, xrec{};
        const Scalar s_data[2]{Scalar(2),Scalar(3)};
        const Scalar b_data[2]{Scalar(1),Scalar(1)};
        AffineScale A{ std::span<const Scalar>(s_data,2),
                       std::span<const Scalar>(b_data,2) };
        if (A.apply(xin, y) != Status::kOK) return 10;
        const std::array<Scalar,2> y_ref{3,7};
        if (!arrays_eq(y, y_ref)) return 11;
        if (A.invert(y, xrec) != Status::kOK) return 12;
        if (!arrays_eq(xrec, xin)) return 13;
    }

    // Case 5: in-place apply and invert
    {
        std::array<Scalar,3> v{1,2,3};
        const Scalar s_data[1]{Scalar(3)};
        const Scalar b_data[1]{Scalar(10)};
        AffineScale A{ std::span<const Scalar>(s_data,1),
                       std::span<const Scalar>(b_data,1) };
        if (A.apply(std::span<const Scalar>(v), std::span<Scalar>(v)) != Status::kOK) return 14;
        const std::array<Scalar,3> v_after{13,16,19};
        if (!arrays_eq(v, v_after)) return 15;
        if (A.invert(std::span<const Scalar>(v), std::span<Scalar>(v)) != Status::kOK) return 16;
        const std::array<Scalar,3> v_back{1,2,3};
        if (!arrays_eq(v, v_back)) return 17;
    }

    // Case 6: mismatched span sizes -> InvalidArg
    {
        const std::array<Scalar,3> xin{1,2,3};
        std::array<Scalar,3> y{};
        const Scalar s_data[2]{Scalar(1),Scalar(1)}; // size 2 vs n=3
        const Scalar b_data[1]{Scalar(0)};
        AffineScale A{ std::span<const Scalar>(s_data,2),
                       std::span<const Scalar>(b_data,1) };
        if (A.apply(xin, y) != Status::kInvalidArg) return 18;
    }

    // Case 7: zero scale behavior
    {
        const std::array<Scalar,1> xin{5};
        std::array<Scalar,1> y{}, xrec{};
        const Scalar s_data[1]{Scalar(0)};
        const Scalar b_data[1]{Scalar(1)};
        AffineScale A{ std::span<const Scalar>(s_data,1),
                       std::span<const Scalar>(b_data,1) };
        if (A.apply(xin, y) != Status::kOK) return 19;
#if defined(NDEBUG)
        // Release: invert clamps to 0 when si==0 by contract
        if (A.invert(y, xrec) != Status::kOK) return 20;
        if (xrec[0] != Scalar(0)) return 21;
#endif
        // In Debug, invert would trap; we skip calling it to avoid abort.
        (void)xrec;
    }

    return 0;
}
