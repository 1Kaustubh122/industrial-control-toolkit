#include <vector>
#include <cassert>
#include <cstring>

#include "ictk/all.hpp"
#include "ictk/filters/iir.hpp"
#include "util/alloc_interposer.hpp"

using namespace ictk;
using namespace ictk::filters;

int main(){
    // alloc 4096 bytes raw buffer aligned to 64 bytes 
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));
    
    // Biquad sos (second order section): stable low pass filter
    Biquad sos[] = {
        Biquad{0.2929, 0.5858, 0.2929, -0.0, 0.1716}
    };

    // // build IIR filter from SOS array
    auto fexp = IIR::from_sos(std::span<const Biquad>(sos, 1), arena, true);
    // ensures construction succeeded
    assert(fexp.has_value());
    // move the IIR out of the wrapper
    auto f =fexp.take();

    // // in: input sample: lenth 1024; a and b are output from two passes
    std::vector<Scalar> in(1024), a, b;
    // reserve: ensures no realloc during push back
    a.reserve(1024); 
    b.reserve(1024);

    // // fills input sequence: every 31st sample is a 1 (impulse), other ramp 0.01 * i: Basic stress test
    for (size_t i=0; i<in.size(); ++i) in[i] = (i % 31 == 0) ? Scalar(1) : Scalar(0.01) * Scalar(i);

    ictk_test::reset_alloc_stats();

    // // run the filter once, producing a -> reset to clear filter states -> run the same test again producing b
    for (auto x: in) a.push_back(f.step(x));
    f.reset();
    for (auto x: in) b.push_back(f.step(x));

    // // check both output are same
    assert(a.size() == b.size());

    // // compare raw bytes of both output vectors
    assert(std::memcmp(a.data(), b.data(), a.size() * sizeof(Scalar)) == 0);

    // // check filter operation did not call new -> confirms zero heap after init
    assert(ictk_test::new_count()==0);
}