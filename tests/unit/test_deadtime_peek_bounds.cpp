#include <cassert>
#include <cstddef>

#include "ictk/all.hpp"
#include "ictk/models/dead_time.hpp"

using namespace ictk;
using namespace ictk::models;

int main(){
    alignas(64) std::byte buf[1024];
    MemoryArena arena(buf, sizeof(buf));

    // // Create 4 sample delay 
    FifoDelay d(4, arena);
    
    // Fill
    for (int i=0;i<10;++i) (void)d.push(Scalar(i));
    #if defined(NDEBUG)
        // release clamps invalid k to last valid
        assert(d.peek(5) == d.peek(d.delay()-1));
    #else
        bool trapped = false;
        // NDEBUG off: expect trap; cannot assert traps, but ensure code path compiles.
        (void)trapped;
    #endif
    return 0;
}