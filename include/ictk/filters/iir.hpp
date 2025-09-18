#pragma once
#include <span>
#include <cmath>
#include <limits>
#include <cstddef>
#include <cstdint>
#include <algorithm>

#include "ictk/visibility.hpp"
#include "ictk/core/types.hpp"
#include "ictk/core/status.hpp"
#include "ictk/core/memory_arena.hpp"
#include "ictk/core/expected.hpp"

namespace ictk::filter{
    // // Normalised a0=1 biquad (Direct form II Transposed)
    struct Biquad{
        Scalar b0{}, b1{}, b2{};
        Scalar a1{}, a2{};  // a(z) = 1 + a1 z^-1 + a2 z^-2
    };
    
    
    
    class ICTK_API IIR {
        public:
            IIR() = default;
            IIR(const IIR&) = delete;
            IIR& operator = (const IIR&) = delete;
            IIR(IIR&&) = default;
            IIR& operator = (IIR&&) = default;


            static Expected<IIR> from_sos(std::span<const Biquad> sos, MemoryArena& arena, bool flush_denormals=false) noexcept{
                if (sos.empty()) return Expected<IIR>::failure(Status::kInvalidArg);

                // Validate stability (poles strictly inside unit circle by margin)
                constexpr Scalar margin = Scalar(1) - Scalar(1e-7);

                for (const auto& s:sos){
                    // // check finite
                    auto finite = [](Scalar v){
                        return std::isfinite(static_cast<double>(v));
                    };

                    if (!finite(s.b0) || !finite(s.b1) || !finite(s.a1) || !finite(s.a2)){
                        return Expected<IIR>::failure(Status::kInvalidArg);
                    }

                    // // Roots of z^2 + a1 z + a2
                    const long double a1 = static_cast<long double>(s.a1);
                    const long double a2 = static_cast<long double>(s.a2);
                    const long double disc = a1 * a1 - 4.0L * a2;
                    long double r1_re, r1_im, r2_re, r2_im;

                    if (disc >= 0){
                        const long double sq = std::sqrt(disc);
                        r1_re = (-a1 + sq)/2.0L;
                        r2_re = (-a1/2.0L);
                        r1_im = sq/2.0L;
                        r2_im = -sq/2.0L;
                    } else{
                        const long double sq = std::sqrt(-disc);
                        r1_re = -a1/2.0L;
                        r2_re = -a1/2.0L;
                        r1_im = sq/2.0L;
                        r2_im = -sq/2.0L;
                    }

                    const long double m1 = std::hypot(r1_re, r1_im);
                    const long double m2 = std::hypot(r2_re, r2_im);

                    if (!(m1 < margin && m2 < margin)) return Expected<IIR>::failure(Status::kInvalidArg);
                }

                const std::size_t nsec = sos.size();

                // // Allocate contiguous state (w1, w2, per section)
                void* mem = arena.allocate(sizeof(State) * nsec, alignof(State));
                if (!mem) return Expected<IIR>::failure(Status::kNoMem);

                IIR f;
                f.flush_denormals_ = flush_denormals;
                f.nsec_ = nsec;
                f.s_ = static_cast<State*>(mem);

                // // Placement init
                for (std::size_t i=0; i<nsec; ++i){
                    new (&f.s_[i]) State();
                    f.s_[i].b = sos[i];
                    f.s_[i].z1 = Scalar(0);
                    f.s_[i].z2 = Scalar(0);
                }
                return Expected<IIR>::success(std::move(f));
            }
            
            void reset() noexcept{
                for (std::size_t i=0; i<nsec_; ++i){
                    s_[i].z1 = Scalar(0);
                    s_[i].z2 = Scalar(0);
                }
            }

            void set_flush_denormals(bool on) noexcept{
                flush_denormals_ = on;
            }

            Scalar step(Scalar x) noexcept{
                // // SF2T cascade
                Scalar y = x;
                
                const Scalar tiny = denorm_epsilon_();
                for (std::size_t i=0; i<nsec_; ++i){
                    auto& st = s_[i];
                    Scalar out = st.b.b0 * y + st.z1;
                    Scalar z1n = st.b.b1 * y  - st.b.a1 * out + st.z2;
                    Scalar z2n = st.b.b2 * y  - st.b.a2 * out;

                    if (flush_denormals_){
                        if (std::abs(z1n) < tiny) z1n = Scalar(0);
                        if (std::abs(z2n) < tiny) z2n = Scalar(0);
                        if (!std::isfinite(static_cast<double>(out))) out = Scalar(0);
                    }
                    st.z1 = z1n;
                    st.z2 = z2n;
                    y = out;
                }
                return y;
            }

            std::size_t sections() const noexcept{
                return nsec_;
            }
        
        private:
            struct State{
                Biquad b{};
                Scalar z1{0}, z2{0};
            };

            static constexpr Scalar denorm_epsilon_() noexcept{
                // // ~10 ULP above min subnormal for the active Scalar
                if constexpr (std::is_same_v<Scalar, float>) return Scalar(1e-30f);
                else return Scalar(1e-300);
            }

            State* s_{nullptr};
            std::size_t nsec_{0};
            bool flush_denormals_{false};
            
    };
            
} // namespace ictk::filter