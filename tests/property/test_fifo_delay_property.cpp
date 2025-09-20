#include <vector>
#include <cassert>
#include <cstddef>

#include "ictk/all.hpp"
#include "ictk/models/dead_time.hpp"
#include "util/alloc_interposer.hpp"

using namespace ictk;
using namespace ictk::models;

int main(){
    // // 64 bye alignment
    alignas(64) std::byte buf[4096];
    MemoryArena arena(buf, sizeof(buf));

    // create delay of 8 samples, internal ring buffer is craved from arena
    FifoDelay d(8, arena);

    ictk_test::reset_alloc_stats(); // start counting

    // Push 1e6 samples and verify exact k-delay
    
    Scalar out=0; // last output from push
    for (int i=0; i < 1'000'000; ++i){
        /*
        Generates a deterministic input sequence
        an impulse every 7 ticks
        push into the delay
        get the delayed output
        */
        const Scalar x = (i % 7 == 0)? Scalar(1) : Scalar(0);
        out = d.push(x);

        /*
        Check delay property
        After the pipeline fill (i>=8): output mist be equal the input from exactly 8 steps ago
        */
        if (i>=8){
            const Scalar expect = ((i - 8) % 7 == 0) ? Scalar(1) : Scalar(0);
            if (out != expect) return 1;
        }
    }

    // wrap & peek coverage
    for (size_t k=0; k<d.delay(); ++k) (void)d.peek(k);

    // // assets zero heap activity after construction
    // Gurantees "no heap after init" during push, peek
    assert(ictk_test::new_count() == 0);
    assert(ictk_test::new_aligned_count() == 0);
    assert(ictk_test::delete_count() == 0);
    assert(ictk_test::delete_aligned_count() == 0);

    return 0;
}
